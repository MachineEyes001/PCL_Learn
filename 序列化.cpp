#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // 构造点云数据
    cloud.width = 50;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    std::cout << rand() << std::endl;
    std::cout << rand() / (RAND_MAX + 1.0f) << std::endl;
    std::cout << 1024 * rand() / (RAND_MAX + 1.0f) << std::endl;

    // 随机生成50个点
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    //// 有时为了节省空间，提高读写效率，还会以binary的格式进行序列化，即将save操作改为：
    //pcl::io::savePCDFileBinary("test_pcd_binary.pcd", cloud);
    //// 或
    //pcl::io::savePCDFile("test_pcd_binary.pcd", cloud, true);
    std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;

    for (size_t i = 0; i < cloud.points.size(); ++i)
        std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

    return (0);
}