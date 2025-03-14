#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointType;

int main()
{
    // 读取原始点云数据
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    pcl::PCDReader reader;
    reader.read("G:/vsdata/PCLlearn/PCDdata/test.pcd", *cloud);

    // 定义圆心和半径
    float centerX = 0.0f;
    float centerY = 0.0f;
    float radius = 9.0f;
    float minZ = -10.0f;
    float maxZ = 10.0f;

    // 创建一个滤波器对象
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");  // 设置滤波字段为 X 轴
    pass.setFilterLimits(centerX - radius, centerX + radius);  // 设置滤波范围
    pcl::PointCloud<PointType>::Ptr filteredCloud(new pcl::PointCloud<PointType>);
    pass.filter(*filteredCloud);

    pass.setInputCloud(filteredCloud);
    pass.setFilterFieldName("y");  // 设置滤波字段为 Y 轴
    pass.setFilterLimits(centerY - radius, centerY + radius);  // 设置滤波范围
    pass.filter(*filteredCloud);

    pass.setInputCloud(filteredCloud);
    pass.setFilterFieldName("z");  // 设置滤波字段为 Z 轴
    pass.setFilterLimits(minZ, maxZ);  // 设置滤波范围
    pass.filter(*filteredCloud);


    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singelColor(filteredCloud, 255, 0, 0);
    viewer.addPointCloud(filteredCloud, singelColor, "filter", 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, "filter");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}