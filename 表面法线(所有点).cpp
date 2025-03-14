#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

int main(int argc, char** argv) 
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 从文件读取点云图
	pcl::PCDReader reader;
	reader.read("G:/vsdata/PCLlearn/PCDdata/bun0.pcd", *cloud);

	// 创建法向量估算类，传递输入数据集
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	// 创建一个空的kdtree，将值传递给法向量估算对象
	// 这个tree对象将会在ne内部根据输入的数据集进行填充（这里设置没有其他的search surface）
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	// 定义输出数据集
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	// 使用一个半径为3cm的球体中的所有邻居点
	ne.setRadiusSearch(0.03);

	// 计算特征
	ne.compute(*cloud_normals);

	// cloud_normals->points.size () 输出特征的点个数应当定于输入的点个数 cloud->points.size ()

		// 可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("before"));
	viewer1->addPointCloud<pcl::PointXYZ>(cloud, "before");
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("normal"));
	viewer->addPointCloud<pcl::Normal>(cloud_normals, "normal");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}
}