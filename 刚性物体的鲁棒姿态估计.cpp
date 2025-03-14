#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

// 首先定义类型，以免使代码混乱
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

// 将刚性对象与具有杂波和遮挡的场景对齐
int main(int argc, char** argv)
{
    // 实例化必要的数据容器，检查输入参数并加载对象和场景点云
    PointCloudT::Ptr object(new PointCloudT);
    PointCloudT::Ptr object_aligned(new PointCloudT);
    PointCloudT::Ptr scene(new PointCloudT);
    FeatureCloudT::Ptr object_features(new FeatureCloudT);
    FeatureCloudT::Ptr scene_features(new FeatureCloudT);

    //// 获取输入对象和场景
    //if (argc != 3)
    //{
    //    pcl::console::print_error("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
    //    return (1);
    //}

    // 加载对象和场景
    pcl::console::print_highlight("Loading point clouds...\n");
    if (pcl::io::loadPCDFile<PointNT>("G:/vsdata/PCLlearn/PCDdata/chef.pcd", *object) < 0 ||
        pcl::io::loadPCDFile<PointNT>("G:/vsdata/PCLlearn/PCDdata/rs1.pcd", *scene) < 0)
    {
        pcl::console::print_error("Error loading object/scene file!\n");
        return (1);
    }

    // 为了加快处理速度，使用PCL的：pcl::VoxelGrid类将对象和场景点云的采样率下采样至5 mm。
    pcl::console::print_highlight("Downsampling...\n");
    pcl::VoxelGrid<PointNT> grid;
    const float leaf = 0.005f;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(object);
    grid.filter(*object);
    grid.setInputCloud(scene);
    grid.filter(*scene);

    // 估计场景的法线
    pcl::console::print_highlight("Estimating scene normals...\n");
    pcl::NormalEstimationOMP<PointNT, PointNT> nest;
    nest.setRadiusSearch(0.01);
    nest.setInputCloud(scene);
    nest.compute(*scene);

    // 特征估计
    // 对于下采样点云中的每个点，使用PCL的pcl::FPFHEstimationOMP<>类来计算对齐过程中用于匹配的快速点特征直方图（FPFH）描述符。
    pcl::console::print_highlight("Estimating features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch(0.025);
    fest.setInputCloud(object);
    fest.setInputNormals(object);
    fest.compute(*object_features);
    fest.setInputCloud(scene);
    fest.setInputNormals(scene);
    fest.compute(*scene_features);

    // 对齐配准对象创建与配置
    // SampleConsensusPrerejective 实现了有效的RANSAC姿态估计循环
    pcl::console::print_highlight("Starting alignment...\n");
    pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
    align.setInputSource(object);
    align.setSourceFeatures(object_features);
    align.setInputTarget(scene);
    align.setTargetFeatures(scene_features);
    align.setMaximumIterations(50000); // RANSAC 迭代次数
    align.setNumberOfSamples(3); // 样本数-在对象和场景之间进行采样的点对应数。至少需要3个点才能计算姿态
    align.setCorrespondenceRandomness(5); // 对应随机性-我们可以在N个最佳匹配之间随机选择，而不是将每个对象FPFH描述符匹配到场景中最接近的匹配特征。这增加了必要的迭代，但也使算法对异常匹配具有鲁棒性
    align.setSimilarityThreshold(0.9f); // 多边形相似度阈值-对齐类使用pcl::registration::CorrespondenceRejectorPoly类，根据采样点之间距离的位置不变的几何一致性，尽早消除不良姿态。在物体和场景上， 将该值设置为越接近1，则贪婪程度越高，从而使算法变得更快。但是，这也会增加存在噪音时消除好姿态的风险
    align.setMaxCorrespondenceDistance(2.5f * leaf); // 内在阈值-这是欧几里德距离阈值，用于确定变换后的对象点是否正确对齐到最近的场景点。在此示例中，我们使用的启发式值为点云分辨率的2.5倍
    align.setInlierFraction(0.25f); // Inlier分数-在许多实际情况下，由于杂波，遮挡或两者兼而有之，场景中观察到的对象的大部分都不可见。在这种情况下，我们需要考虑不会将所有对象点都用于对准场景的姿态假设（猜想）。正确对齐的点的绝对数量是使用inlier阈值确定的，并且如果该数量与对象中总点数的比大于指定的inlier分数，则我们接受姿态假设（猜想）为有效
    // 执行配准并输出结果
    {
        pcl::ScopeTime t("Alignment");
        align.align(*object_aligned);
    }//对齐的对象存储在点云object_aligned中。如果找到了一个具有足够inliers的姿态（占对象点总数的25％以上），则该算法会收敛，并且我们可以打印并可视化结果

    if (align.hasConverged())
    {
        // 打印结果
        printf("\n");
        Eigen::Matrix4f transformation = align.getFinalTransformation();
        pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
        pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
        pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
        pcl::console::print_info("\n");
        pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
        pcl::console::print_info("\n");
        pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), object->size());

        // 显示对齐方式
        pcl::visualization::PCLVisualizer visu("Alignment");
        visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene");
        visu.addPointCloud(object_aligned, ColorHandlerT(object_aligned, 0.0, 0.0, 255.0), "object_aligned");
        visu.spin();
    }
    else
    {
        pcl::console::print_error("Alignment failed!\n");
        return (1);
    }

    return (0);
}