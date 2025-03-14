#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

/**
 * ����������
 */
int main() {
    // ���ص���
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("G:/vsdata/PCLlearn/PCDdata/bun0.pcd", *cloud);

    // ���߶���
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // ���ڷ��߹��ƵĶ���
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    // ����ÿ���㣬ʹ�ð뾶Ϊ3cm����������
    normalEstimation.setRadiusSearch(0.03);
    // ͨ��kd��ʹ�������Ӹ�Ч
    // ���߹��ƶ���ʹ����������������ڵ�
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    // ���㷨��
    normalEstimation.compute(*normals);
    // ���߿��ӻ�
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    // ����int level=2 ��ʾÿ2�������һ��������
    // ����float scale=0.01 ��ʾ��������������Ϊ0.01��
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 2, 0.01, "normals");
    // viewer.addCoordinateSystem(0.1); //�������ϵ

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
    return 0;
}