#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>

int main(int argc, char** argv)
{
    // 新建点云存储对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 从文件读取点云图
    pcl::PCDReader reader;
    reader.read("G:/vsdata/PCLlearn/PCDdata/bun0.pcd", *cloud);
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
        << " data points (" << pcl::getFieldsList(*cloud) << ").";

    // 滤波对象
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    //建立搜索对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    filter.setSearchMethod(kdtree);
    //设置搜索邻域的半径为3cm
    filter.setSearchRadius(0.03);
    // Upsampling 采样的方法有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
    filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    // 采样的半径是
    filter.setUpsamplingRadius(0.03);
    // 采样步数的大小
    filter.setUpsamplingStepSize(0.02);
    filter.process(*filteredCloud);

    std::cerr << "PointCloud after filtering: " << filteredCloud->width * filteredCloud->height
        << " data points (" << pcl::getFieldsList(*filteredCloud) << ").";
}