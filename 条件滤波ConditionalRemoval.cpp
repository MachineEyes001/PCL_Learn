#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointType;  //���ͱ���

void showPointClouds(const pcl::PointCloud<PointType>::Ptr& cloud, const pcl::PointCloud<PointType>::Ptr& cloud2) {// ����PCLVisualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    // ���ñ���ɫΪ��ɫ
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0);

    // ���һ����ͨ���� (��������ָ����ɫ��Ҳ����ȥ��single_color����������)
    pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<PointType>(cloud, single_color, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

    // ��ӵڶ������� (��������ָ����ɫ��Ҳ����ȥ��single_color2����������)
    pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color2(cloud, 255, 0, 0);
    viewer->addPointCloud<PointType>(cloud2, single_color2, "sample cloud 2");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud 2");
    viewer->addCoordinateSystem(1.0);

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}
int main(int argc, char** argv) {
    std::string select = "";
    std::cout << "please input 'r' or 'c':";
    std::cin >> select;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // ����������
    cloud->width = 100;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = 10 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 10 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 10 * rand() / (RAND_MAX + 1.0f);
    }

    if (select=="r") {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        // �����˲���
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(3);
        outrem.setMinNeighborsInRadius(10);
        // ����˲���
        outrem.filter(*cloud_filtered);
    }
    else if (select=="c") {
        // ����ɸѡ����
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));
        // �����˲���
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition(range_cond);
        condrem.setInputCloud(cloud);
        condrem.setKeepOrganized(true);
        // ����˲���
        condrem.filter(*cloud_filtered);
    }
    else {
        std::cerr << "please specify command line arg 'r' or 'c'" << std::endl;
        exit(0);
    }
    std::cerr << "Cloud before filtering: " << std::endl;
    for (size_t i = 0; i < cloud->points.size(); ++i)
        std::cerr << "    " << cloud->points[i].x << " "
        << cloud->points[i].y << " "
        << cloud->points[i].z << std::endl;
    // ��ʾ�˲���ĵ���
    std::cerr << "Cloud after filtering: " << std::endl;
    for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
        std::cerr << "    " << cloud_filtered->points[i].x << " "
        << cloud_filtered->points[i].y << " "
        << cloud_filtered->points[i].z << std::endl;

    showPointClouds(cloud, cloud_filtered);

    return (0);
}