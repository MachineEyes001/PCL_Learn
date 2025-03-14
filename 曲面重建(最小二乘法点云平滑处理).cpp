#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>  //kd-tree搜索对象的类定义的头文件
#include <pcl/surface/mls.h>        //最小二乘法平滑处理类定义头文件
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

int main(int argc, char** argv) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // 从文件读取点云图
    pcl::PCDReader reader;
    reader.read("G:/vsdata/PCLlearn/PCDdata/bun0.pcd", *cloud);

    // 创建 KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // 输出具有PointNormal类型，以便存储MLS计算的法线
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);

    // Init对象（第二种点类型用于法线，即使未使用）
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    // 设置参数
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.1);

    // 重建
    mls.process(*mls_points);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("before"));
    viewer1->addPointCloud<pcl::PointXYZ>(cloud, "before");
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("smoothed"));
    viewer->addPointCloud<pcl::PointNormal>(mls_points, "smoothed");
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }
    // 保存
    pcl::io::savePCDFile("bun0-mls.pcd", *mls_points);
}