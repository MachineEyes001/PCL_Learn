#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZ PointT;
int main(int argc, char** argv) {
    srand((unsigned int)time(NULL));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 填充点云数据
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
     创建直通滤波器的对象，设立参数，滤波字段名被设置为Z轴方向，可接受的范围为（0.0，5.0）
     即将点云中所有点的Z轴坐标不在该范围内的点过滤掉或保留，这里是过滤掉，由函数setFilterLimitsNegative设定
     ***********************************************************************************/
    // 创建滤波器
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);          // 1. 设置输入源
    pass.setFilterFieldName("z");       // 2. 设置过滤域名
    pass.setFilterLimits(0.0, 5.0);     // 3. 设置过滤范围
    //pass.setFilterLimitsNegative(true); // 设置获取Limits之外的内容
    pass.filter(*cloud_filtered);       // 4. 执行过滤，并将结果输出到cloud_filtered

    std::cerr << "Cloud after filtering: " << std::endl;
    for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
        std::cerr << "    " << cloud_filtered->points[i].x << " "
        << cloud_filtered->points[i].y << " "
        << cloud_filtered->points[i].z << std::endl;
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    // 设置背景色为黑色
    viewer.setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud_filtered, "z");//按照z字段进行渲染
    viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, fildColor, "sample");//显示点云，其中fildColor为颜色显示
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "sample");//设置点云大小
    // 添加一个放大5倍后的坐标系
    viewer.addCoordinateSystem(5);
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
    return (0);
}