#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
//使用【正态分布变换算法】和【用来过滤数据的过滤器】对应的头文件，这个过滤器可以用其他过滤器来替换, 
//但是使用体素网格过滤器(approximate voxel filter)处理结果较好
#include <pcl/registration/ndt.h> // ndt配准头文件
#include <pcl/filters/approximate_voxel_grid.h>

using namespace std::chrono_literals;

int main(int argc, char** argv) 
{
    // 加载首次的房间扫描数据作为目标点云 target_cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("G:/vsdata/PCLlearn/PCDdata/room_scan1.pcd", *target_cloud) == -1) {
        PCL_ERROR("Couldn't read file room_scan1.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << target_cloud->size() << " data points from room_scan1.pcd" << std::endl;

    // 加载从新的视角得到的房间第二次扫描数据作为输入源点云 input_cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("G:/vsdata/PCLlearn/PCDdata/room_scan2.pcd", *input_cloud) == -1) {
        PCL_ERROR("Couldn't read file room_scan2.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << input_cloud->size() << " data points from room_scan2.pcd" << std::endl;
/*-------------------------------------------------------------------------------------------------------------------------
以上代码加载了两个 PCD 文件到共享指针，配准操作是完成 【源点云input_cloud】到【目标点云target_cloud】坐标系变换矩阵的估算，
即求出input_cloud变换到target_cloud的变换矩阵
-------------------------------------------------------------------------------------------------------------------------*/
    
    // 过滤输入点云到原始大小的约10%，以提高配准速度
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud(input_cloud);
    approximate_voxel_filter.filter(*filtered_cloud);
    std::cout << "Filtered cloud contains " << filtered_cloud->size()
        << " data points from room_scan2.pcd" << std::endl;
/*-------------------------------------------------------------------------------------------------------------------------
以上代码将过滤输入点云到原始大小的约10%，以提高配准速度。这里用任何其他均匀过滤器都可以，目标点云target_cloud不需要进行滤波
处理，因为NDT算法在目标点云对应的体素Voxel网格数据结构计算时，不使用单个点，而是使用体素的点。即已做了降采样处理。
-------------------------------------------------------------------------------------------------------------------------*/

    // 使用默认值创建NDT对象。内部数据结构直到稍后才会初始化。
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // 根据输入数据的尺度设置NDT相关参数
    // 设置终止条件的最小转换差
    ndt.setTransformationEpsilon(0.01);
    // 设置More Thuente行搜索的最大步长
    ndt.setStepSize(0.1);
    // 设置NDT网格结构的分辨率（VoxelGridCovariance）
    ndt.setResolution(1.0);
/*-------------------------------------------------------------------------------------------------------------------------
以上代码设置一些尺度相关的参数,因为 NDT 算法使用一个体素化数据结构和More-Thuente行搜索，因此需要缩放一些参数来适应数据集。
以上参数看起来在我们使用的房间尺寸比例下运行地很好，但是它们如果需要处理例如一个咖啡杯的扫描之类更小物体，需要对参数进行很
大程度的缩小。在变换中 Epsilon 参数分别从长度和弧度，定义了变换矢量[ x, y, z,roll,pitch, yaw]的最小许可的递增量，一旦递增量
减小到这个临界值以下 ，那么配准算法就将终止。步长StepSize参数定义了 More-Thuente 行搜索允许的最大步长，这个行搜索算法确定了
最大值以下的最佳步长，当靠近最优解时该算法会缩短迭代步长，在更大的最大步长将会在较少的迭代次数下遍历较大的距离，但是却有过
度迭代和在不符合要求的局部最小值处结束的风险。
-------------------------------------------------------------------------------------------------------------------------*/

    // 设置配准迭代的最大次数
    ndt.setMaximumIterations(35);
/*-------------------------------------------------------------------------------------------------------------------------
这个MaximumIterations参数控制了优化程序运行的最大迭代次数，一般来说，在达到这个限制值之前优化程序就会在 epsilon 变换阈值下
终止。添加此最大迭代次数限制能够增加程序鲁棒性,阻止了它在错误的方向运行太久。
-------------------------------------------------------------------------------------------------------------------------*/

    // 设置过滤后的输入源点云（第二次扫描数据）
    ndt.setInputSource(filtered_cloud);
    // 设置目标点云（第一次扫描数据），作为对其配准的目标
    ndt.setInputTarget(target_cloud);
/*-------------------------------------------------------------------------------------------------------------------------
这里，我们把点云赋给 NDT 配准对象，目标点云的坐标系是被匹配的输入点云的参考坐标系，匹配完成后输入点云将被变换到与目标点云同
一坐标系下，当加载目标点云后，NDT 算法的内部数据结构被初始化。
-------------------------------------------------------------------------------------------------------------------------*/

    // 设置使用机器人测距法得到的粗略初始变换矩阵
    Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(1.79387, 0, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
/*-------------------------------------------------------------------------------------------------------------------------
以上代码块中创建了一个点云配准变换矩阵的初始估计，虽然算法运行并不需要这样的一个初始变换矩阵，但是有了它容易得到更好的结果，
尤其是当参考坐标系之间有较大差异时（本例即是），在机器人应用程序（例如用于生成此数据集的应用程序）中，通常使用里程表数据生
成初始转换。
-------------------------------------------------------------------------------------------------------------------------*/

    // 计算所需的刚体变换，以使输入点云与目标点云对齐
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*output_cloud, init_guess);
    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
        << " score: " << ndt.getFitnessScore() << std::endl;
/*-------------------------------------------------------------------------------------------------------------------------
最后，我们准备对齐点云。生成的转换后的输入点云存储在输出点云中。然后，我们显示对齐的结果以及欧几里得适合度得分FitnessScore，
该分数计算为从输出云到目标云中最近点的距离的平方。
-------------------------------------------------------------------------------------------------------------------------*/

    // 使用找到的变换矩阵，来对未过滤的输入点云进行变换
    pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());

    // 保存变换后的输入点云
    pcl::io::savePCDFileASCII("room_scan2_transformed.pcd", *output_cloud);
/*-------------------------------------------------------------------------------------------------------------------------
在对齐之后，输出点云output_cloud将立即包含过滤后的输入点云的转换版本，因为我们向算法传递了过滤后的点云，而不是原始输入点云。
为了获得原始点云的对齐版本，从NDT算法中提取最终转换矩阵并转换原始输入点云。现在，可以将该云保存到文件room_scan2_transformed.pcd
中，以备将来使用。
-------------------------------------------------------------------------------------------------------------------------*/

    // 初始化点云可视化工具
    pcl::visualization::PCLVisualizer::Ptr
        viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    // 着色并可视化目标点云（红色）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        target_color(target_cloud, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        1, "target cloud");

    // 着色并可视化转换后的输入点云（绿色）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        output_color(output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        1, "output cloud");

    // 可视化
    viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();
    while (!viewer_final->wasStopped()) {
        viewer_final->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }

    return (0);
}