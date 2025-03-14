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

// ���ȶ������ͣ�����ʹ�������
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

// �����Զ���������Ӳ����ڵ��ĳ�������
int main(int argc, char** argv)
{
    // ʵ������Ҫ���������������������������ض���ͳ�������
    PointCloudT::Ptr object(new PointCloudT);
    PointCloudT::Ptr object_aligned(new PointCloudT);
    PointCloudT::Ptr scene(new PointCloudT);
    FeatureCloudT::Ptr object_features(new FeatureCloudT);
    FeatureCloudT::Ptr scene_features(new FeatureCloudT);

    //// ��ȡ�������ͳ���
    //if (argc != 3)
    //{
    //    pcl::console::print_error("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
    //    return (1);
    //}

    // ���ض���ͳ���
    pcl::console::print_highlight("Loading point clouds...\n");
    if (pcl::io::loadPCDFile<PointNT>("G:/vsdata/PCLlearn/PCDdata/chef.pcd", *object) < 0 ||
        pcl::io::loadPCDFile<PointNT>("G:/vsdata/PCLlearn/PCDdata/rs1.pcd", *scene) < 0)
    {
        pcl::console::print_error("Error loading object/scene file!\n");
        return (1);
    }

    // Ϊ�˼ӿ촦���ٶȣ�ʹ��PCL�ģ�pcl::VoxelGrid�ཫ����ͳ������ƵĲ������²�����5 mm��
    pcl::console::print_highlight("Downsampling...\n");
    pcl::VoxelGrid<PointNT> grid;
    const float leaf = 0.005f;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(object);
    grid.filter(*object);
    grid.setInputCloud(scene);
    grid.filter(*scene);

    // ���Ƴ����ķ���
    pcl::console::print_highlight("Estimating scene normals...\n");
    pcl::NormalEstimationOMP<PointNT, PointNT> nest;
    nest.setRadiusSearch(0.01);
    nest.setInputCloud(scene);
    nest.compute(*scene);

    // ��������
    // �����²��������е�ÿ���㣬ʹ��PCL��pcl::FPFHEstimationOMP<>��������������������ƥ��Ŀ��ٵ�����ֱ��ͼ��FPFH����������
    pcl::console::print_highlight("Estimating features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch(0.025);
    fest.setInputCloud(object);
    fest.setInputNormals(object);
    fest.compute(*object_features);
    fest.setInputCloud(scene);
    fest.setInputNormals(scene);
    fest.compute(*scene_features);

    // ������׼���󴴽�������
    // SampleConsensusPrerejective ʵ������Ч��RANSAC��̬����ѭ��
    pcl::console::print_highlight("Starting alignment...\n");
    pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
    align.setInputSource(object);
    align.setSourceFeatures(object_features);
    align.setInputTarget(scene);
    align.setTargetFeatures(scene_features);
    align.setMaximumIterations(50000); // RANSAC ��������
    align.setNumberOfSamples(3); // ������-�ڶ���ͳ���֮����в����ĵ��Ӧ����������Ҫ3������ܼ�����̬
    align.setCorrespondenceRandomness(5); // ��Ӧ�����-���ǿ�����N�����ƥ��֮�����ѡ�񣬶����ǽ�ÿ������FPFH������ƥ�䵽��������ӽ���ƥ���������������˱�Ҫ�ĵ�������Ҳʹ�㷨���쳣ƥ�����³����
    align.setSimilarityThreshold(0.9f); // ��������ƶ���ֵ-������ʹ��pcl::registration::CorrespondenceRejectorPoly�࣬���ݲ�����֮������λ�ò���ļ���һ���ԣ���������������̬��������ͳ����ϣ� ����ֵ����ΪԽ�ӽ�1����̰���̶�Խ�ߣ��Ӷ�ʹ�㷨��ø��졣���ǣ���Ҳ�����Ӵ�������ʱ��������̬�ķ���
    align.setMaxCorrespondenceDistance(2.5f * leaf); // ������ֵ-����ŷ����¾�����ֵ������ȷ���任��Ķ�����Ƿ���ȷ���뵽����ĳ����㡣�ڴ�ʾ���У�����ʹ�õ�����ʽֵΪ���Ʒֱ��ʵ�2.5��
    align.setInlierFraction(0.25f); // Inlier����-�����ʵ������£������Ӳ����ڵ������߼����֮�������й۲쵽�Ķ���Ĵ󲿷ֶ����ɼ�������������£�������Ҫ���ǲ��Ὣ���ж���㶼���ڶ�׼��������̬���裨���룩����ȷ����ĵ�ľ���������ʹ��inlier��ֵȷ���ģ����������������������ܵ����ıȴ���ָ����inlier�����������ǽ�����̬���裨���룩Ϊ��Ч
    // ִ����׼��������
    {
        pcl::ScopeTime t("Alignment");
        align.align(*object_aligned);
    }//����Ķ���洢�ڵ���object_aligned�С�����ҵ���һ�������㹻inliers����̬��ռ�����������25�����ϣ�������㷨���������������ǿ��Դ�ӡ�����ӻ����

    if (align.hasConverged())
    {
        // ��ӡ���
        printf("\n");
        Eigen::Matrix4f transformation = align.getFinalTransformation();
        pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
        pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
        pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
        pcl::console::print_info("\n");
        pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
        pcl::console::print_info("\n");
        pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), object->size());

        // ��ʾ���뷽ʽ
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