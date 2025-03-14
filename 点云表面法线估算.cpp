#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

/**
 * 法向量估计
 */
int main() {
    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("G:/vsdata/PCLlearn/PCDdata/bun0.pcd", *cloud);

    // 法线对象
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // 用于法线估计的对象
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    // 对于每个点，使用半径为3cm的所有邻域
    normalEstimation.setRadiusSearch(0.03);
    // 通过kd树使搜索更加高效
    // 法线估计对象将使用它来查找最近的邻点
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    // 计算法线
    normalEstimation.compute(*normals);
    // 法线可视化
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    // 参数int level=2 表示每2个点绘制一个法向量
    // 参数float scale=0.01 表示法向量长度缩放为0.01倍
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 2, 0.01, "normals");
    // viewer.addCoordinateSystem(0.1); //添加坐标系

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
    return 0;
}