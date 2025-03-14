#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv) 
{
    // ��ϵͳʱ���ʼ���������
    srand(time(NULL));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // ���ɵ�������1000��
    cloud->width = 1000;
    cloud->height = 1;  // 1 ��ʾ����Ϊ�������
    cloud->points.resize(cloud->width * cloud->height);

    // ������������� 0 - 1023
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }

    // ����KdTree��ʵ����KdTreeFLANN (���ƽ��ڿ��ٿ�)
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // pcl::search::KdTree<pcl::PointXYZ> kdtree;
    // ���������ռ䣬��cloud��Ϊ����
    kdtree.setInputCloud(cloud);

    // ��ʼ��һ������ĵ㣬��Ϊ��ѯ��
    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

// ��ʽһ������K�������

    // ����K����������������������������
    // K = 10 ��ʾ����10���ٽ���
    // pointIdxNKNSearch        �������������ٽ��������
    // pointNKNSquaredDistance  �����Ӧ�ٽ���ľ����ƽ��
    int K = 10;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    std::cout << "K nearest neighbor search at (" << searchPoint.x
        << " " << searchPoint.y
        << " " << searchPoint.z
        << ") with K=" << K << std::endl;

    if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
            << " " << cloud->points[pointIdxNKNSearch[i]].y
            << " " << cloud->points[pointIdxNKNSearch[i]].z
            << " (����ƽ��: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }

// ��ʽ����ͨ��ָ���뾶����

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    // ����һ�����[0,256)�İ뾶ֵ
    float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

    std::cout << "Neighbors within radius search at (" << searchPoint.x
        << " " << searchPoint.y
        << " " << searchPoint.z
        << ") with radius=" << radius << std::endl;


    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
            << " " << cloud->points[pointIdxRadiusSearch[i]].y
            << " " << cloud->points[pointIdxRadiusSearch[i]].z
            << " (����ƽ��:: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }

//���ӻ�
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");

    pcl::PointXYZ originPoint(0.0, 0.0, 0.0);
    // ��Ӵ�ԭ�㵽��������߶�
    viewer.addLine(originPoint, searchPoint);
    // ���һ����������ΪԲ�ģ������뾶Ϊ�뾶������
    viewer.addSphere(searchPoint, radius, "sphere", 0);
    // ���һ���ŵ�200���������ϵ
    viewer.addCoordinateSystem(200);

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    return 0;
}