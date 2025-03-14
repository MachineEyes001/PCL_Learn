#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/search/kdtree.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>);

    // 从文件读取点云图
    pcl::PCDReader reader;
    reader.read("G:/vsdata/PCLlearn/PCDdata/bun0.pcd", *cloud);

    // 平滑对象（选择要作为输入和输出的点类型）
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> filter;
    filter.setInputCloud(cloud);
    // 使用半径为5厘米的所有邻域
    filter.setSearchRadius(0.05);
    // 设置参数
    filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);//设置上采样方法为局部平面采样，即根据局部平面拟合来增加新的点
    filter.setUpsamplingRadius(0.03);//设置上采样半径，用于确定采样时考虑的邻域范围
    filter.setUpsamplingStepSize(0.02);//设置上采样步长，用于确定新生成点的间隔距离
    filter.setComputeNormals(true);// 设置是否计算平滑后的点云的法线信息（可选）
    // 创建并设置用于搜索的kd树对象，即设置用于确定点云邻域的搜索方法
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    filter.setSearchMethod(kdtree);
    filter.process(*smoothedCloud);//对输入点云进行平滑处理和曲面重建，并将结果存储到smoothedCloud中
    // 可视化
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("before"));
    viewer1->addPointCloud<pcl::PointXYZ>(cloud, "before");
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("smoothed"));
    viewer->addPointCloud<pcl::PointNormal>(smoothedCloud, "smoothed");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }
}