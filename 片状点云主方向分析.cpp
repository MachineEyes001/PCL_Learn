#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/pca.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("F:/Paper/test/12-29-38-cloud-stand-ECE-GR.pcd", *pointCloud) == -1) //* 加载文件
    {
    	return false;
    }//调试

    // 获取质心和半径
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;// 创建惯性矩估算对象，设置输入点云，并进行计算
    feature_extractor.setInputCloud(pointCloud);
    feature_extractor.compute();
    Eigen::Vector3f mass_center;//质心
    feature_extractor.getMassCenter(mass_center);
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    float radius = (max_point_OBB.x - min_point_OBB.x + max_point_OBB.y - min_point_OBB.y) / 4;//半径

    // 计算主平面法线
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);// 法线对象
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation; // 用于法线估计的对象
    normalEstimation.setInputCloud(pointCloud);
    normalEstimation.setRadiusSearch(0.3); // 对于每个点，使用半径为SearchRadius的所有邻域
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);// 通过kd树使搜索更加高效
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);  // 计算法线

    Eigen::Vector3f mainDirection(0.0, 0.0, 0.0);
    // 计算所有法线方向的总和
    for (size_t i = 0; i < normals->size(); ++i)
    {
        mainDirection.x() += normals->points[i].normal_x;
        mainDirection.y() += normals->points[i].normal_y;
        mainDirection.z() += normals->points[i].normal_z;
    }
    // 归一化得到平均法线方向作为主方向
    mainDirection.normalize();
    // 输出主方向
    std::cout << "主方向：" << mainDirection.x() << ", " << mainDirection.y() << ", " << mainDirection.z() << std::endl;
    
    // 打印结果
    std::cout << "质心坐标：" << mass_center(0) << ", " << mass_center(1) << ", " << mass_center(2) << std::endl;
    std::cout << "半径：" << radius << std::endl;

    pcl::visualization::PCLVisualizer viewer;
    // 高亮显示质心
    pcl::PointXYZRGB selected_point;
    selected_point.x = mass_center(0);
    selected_point.y = mass_center(1);
    selected_point.z = mass_center(2);
    selected_point.r = 255;
    selected_point.g = 0;
    selected_point.b = 0;
    viewer.addSphere(selected_point, 0.005, 255, 0, 0, "center_point");
    viewer.addPointCloud(pointCloud);
    viewer.addCoordinateSystem();
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    return 0;
}
