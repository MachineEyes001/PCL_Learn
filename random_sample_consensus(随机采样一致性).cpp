#include <iostream>
#include <thread>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr final = nullptr) {
    // --------------------------------------------
    // ------------��3D��ͼ����ӵ���------------
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

    if (final != nullptr) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(final, 255, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(final, color_handler, "final cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "final cloud");
    }

    viewer->addCoordinateSystem(1.0, "global");
    viewer->initCameraParameters();
    return (viewer);
}
/**
 * ʹ�÷�����
 * random_sample_consensus     ���������ⲿ���ƽ��
 * random_sample_consensus -f  ���������ⲿ���ƽ�棬������ƽ���ڲ���
 * random_sample_consensus -s  ���������ⲿ�������
 * random_sample_consensus -sf ���������ⲿ������壬�����������ڲ���
 */
int main(int argc, char** argv) 
{
    std::string select = "";
    std::cout << "please input '-f' or '-sf':";
    std::cin >> select;
    // ��ʼ������
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

    // �õ����PointCloud
    cloud->width = 5000;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    for (std::size_t i = 0; i < cloud->points.size(); ++i) {
        if (select == "-s" || select == "-sf") {
            cloud->points[i].x = 10 * rand() / (RAND_MAX + 1.0);
            cloud->points[i].y = 10 * rand() / (RAND_MAX + 1.0);
            if (i % 5 == 0)     // ���ܻ�ɢ����������
                cloud->points[i].z = 10 * rand() / (RAND_MAX + 1.0);  //�˴���Ӧ�ĵ�Ϊ�����
            else if (i % 2 == 0)// ��������������
                cloud->points[i].z = sqrt(1 - (cloud->points[i].x * cloud->points[i].x)
                    - (cloud->points[i].y * cloud->points[i].y));
            else // �����帺������
                cloud->points[i].z = -sqrt(1 - (cloud->points[i].x * cloud->points[i].x)
                    - (cloud->points[i].y * cloud->points[i].y));
        }
        else
        {   //��x+y+z=1����һ���ֵ������ݣ���ʱ��������ɵ�����ƽ����Ϊ�ڵ�
            cloud->points[i].x = 10 * rand() / (RAND_MAX + 1.0);
            cloud->points[i].y = 10 * rand() / (RAND_MAX + 1.0);
            if (i % 2 == 0)
                cloud->points[i].z = 10 * rand() / (RAND_MAX + 1.0);   //��Ӧ�ľ����
            else
                cloud->points[i].z = 1 * (cloud->points[i].x + cloud->points[i].y);
        }
    }
    std::vector<int> inliers;

    // ����RandomSampleConsensus���󲢼����ʵ���ģ��
    // Բ��
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
        model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
    // ƽ��
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
        model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    if (select == "-f") {
        //���������в���������������Ӧƽ��ģ�ͣ����洢���Ƶľ��ڵ�
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
        ransac.setDistanceThreshold(0.5);    //��ƽ�����С��0.01 �ĵ��Ϊ���ڵ㿼��
        ransac.computeModel();                   //ִ�������������
        ransac.getInliers(inliers);                 //�洢�������õľ��ڵ�
    }
    else if (select == "-sf") {
        //���������в���  ����������Ӧ��Բ��ģ�ͣ��洢���Ƶ��ڵ�
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
        ransac.setDistanceThreshold(0.5);
        ransac.computeModel();
        ransac.getInliers(inliers);
    }

    // �������ģ�͵�����inlier���Ƶ�final������
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

    // �������ӻ��������ԭʼ���ƻ�����inliers
    // ȡ����ָ���������в���
    pcl::visualization::PCLVisualizer::Ptr viewer;
    if (select == "-f" || select == "-sf")
        viewer = simpleVis(cloud, final);
    else
        viewer = simpleVis(cloud);
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
    return 0;
}