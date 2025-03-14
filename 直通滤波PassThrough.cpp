#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZ PointT;
int main(int argc, char** argv) {
    srand((unsigned int)time(NULL));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // ����������
    cloud->width = 10;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = 10 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 10 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 10 * rand() / (RAND_MAX + 1.0f);
    }
    std::cerr << "Cloud before filtering: " << std::endl;
    for (size_t i = 0; i < cloud->points.size(); ++i)
        std::cerr << "    " << cloud->points[i].x << " "
        << cloud->points[i].y << " "
        << cloud->points[i].z << std::endl;
    /************************************************************************************
     ����ֱͨ�˲����Ķ��������������˲��ֶ���������ΪZ�᷽�򣬿ɽ��ܵķ�ΧΪ��0.0��5.0��
     �������������е��Z�����겻�ڸ÷�Χ�ڵĵ���˵������������ǹ��˵����ɺ���setFilterLimitsNegative�趨
     ***********************************************************************************/
    // �����˲���
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);          // 1. ��������Դ
    pass.setFilterFieldName("z");       // 2. ���ù�������
    pass.setFilterLimits(0.0, 5.0);     // 3. ���ù��˷�Χ
    //pass.setFilterLimitsNegative(true); // ���û�ȡLimits֮�������
    pass.filter(*cloud_filtered);       // 4. ִ�й��ˣ�������������cloud_filtered

    std::cerr << "Cloud after filtering: " << std::endl;
    for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
        std::cerr << "    " << cloud_filtered->points[i].x << " "
        << cloud_filtered->points[i].y << " "
        << cloud_filtered->points[i].z << std::endl;
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    // ���ñ���ɫΪ��ɫ
    viewer.setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud_filtered, "z");//����z�ֶν�����Ⱦ
    viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, fildColor, "sample");//��ʾ���ƣ�����fildColorΪ��ɫ��ʾ
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "sample");//���õ��ƴ�С
    // ���һ���Ŵ�5���������ϵ
    viewer.addCoordinateSystem(5);
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
    return (0);
}