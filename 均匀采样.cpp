#include <pcl/io/pcd_io.h>
#include <pcl/filters/uniform_sampling.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 从文件读取点云图
    pcl::PCDReader reader;
    reader.read("G:/vsdata/PCLlearn/PCDdata/table_scene_lms400.pcd", *cloud);
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
        << " data points (" << pcl::getFieldsList(*cloud) << ").";
    // 统一采样对象
    pcl::UniformSampling<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setRadiusSearch(0.01f);
    filter.filter(*filteredCloud);

    std::cerr << "PointCloud after filtering: " << filteredCloud->width * filteredCloud->height
        << " data points (" << pcl::getFieldsList(*filteredCloud) << ").";
}