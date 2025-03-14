#include <pcl/ModelCoefficients.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h> //投影滤波
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
using namespace std;

int
main(int argc, char** argv)
{
	//------------------------------读取点云数据---------------------------------
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ >);
	pcl::PCDReader reader;
	reader.read("G:/vsdata/PCLvision/CloudData/test.PCD", *cloud);
	cout << "PointCloud has: " << cloud->points.size() << " data points." << endl;
	//--------------------------------直通滤波-----------------------------------
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ >);
	pcl::PassThrough<pcl::PointXYZ > pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("y");//将Y轴不在（0，1.5）范围内的点过滤掉
	pass.setFilterLimits(0, 1.5);
	pass.filter(*cloud_filtered);//剩余的点储存在cloud_filtered中后续使用
	cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << endl;
	//-----------------------------存储点云到输出文件----------------------------
	pcl::PCDWriter writer;
	writer.write("laptop.pcd", *cloud_filtered, false);

	// --------------------------计算AABB包围盒顶点坐标-------------------------
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*cloud_filtered, minPt, maxPt);
	cout << "笔记本外壳的宽为" << maxPt.z - minPt.z << "cm" << endl;
	cout << "笔记本外壳的长为" << maxPt.x - minPt.x << "cm" << endl;
	cout << "笔记本外壳的高为" << maxPt.y - minPt.y << "cm"<<endl;

	//--------------------------------计算法线-----------------------------------
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ >::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ >);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	n.setSearchMethod(tree);
	n.setInputCloud(cloud_filtered);
	n.setKSearch(50);
	n.compute(*normals);
	//------------------------------创建分割对象---------------------------------
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
	//-------------------------------提取圆柱体模型------------------------------
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ >);
	pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
	//为圆柱体分割创建分割对象，并设置参数
	seg.setOptimizeCoefficients(true);        //设置对估计的模型系数需要进行优化
	seg.setModelType(pcl::SACMODEL_CYLINDER); //设置分割模型为圆柱型
	seg.setMethodType(pcl::SAC_RANSAC);       //设置采用RANSAC作为算法的参数估计方法
	seg.setNormalDistanceWeight(0.1);         //设置表面法线权重系数
	seg.setMaxIterations(5000);               //设置迭代的最大次数
	seg.setDistanceThreshold(1);           //设置内点到模型的距离允许最大值 
	seg.setRadiusLimits(50, 55);              //设置估计出圆柱模型的半径范围
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(normals);
	//获取圆柱模型系数和圆柱上的点
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	cout << "Cylinder coefficients: " << *coefficients_cylinder << endl;
	//-----------------------------存储点云到输出文件----------------------------
	pcl::ExtractIndices<pcl::PointXYZ > extract;
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ >);
	extract.filter(*cloud_cylinder);
	cout << "孔的半径为" << coefficients_cylinder->values[6] <<"cm"<< endl;
	// -------------------------------结果可视化--------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->setWindowName("计算点云的AABB包围盒");
	//viewer->addCoordinateSystem(1.0);
	//viewer->initCameraParameters();
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_filtered, 0, 255, 0); // green
	viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, single_color, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
	viewer->addPointCloud(cloud_cylinder, "cloud_cylinder");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_cylinder");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_cylinder");
	//viewer->addCube(minPt.x, maxPt.x, minPt.y, maxPt.y, minPt.z, maxPt.z, 1.0, 0.0, 0.0, "AABB");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "AABB");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "AABB");


	//// -------------------------------结果可视化--------------------------------
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D"));
	//viewer->setWindowName("计算点云的AABB包围盒");
	//int v1(0);
	//viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	//viewer->setBackgroundColor(0, 0, 0, v1);
	//viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);
	//viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "sample cloud", v1);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
	////viewer->addCoordinateSystem(1.0);
	////viewer->initCameraParameters();
	//int v2(0);
	//viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
	//viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
	//viewer->addText("AABB包围盒", 10, 10, "v2_text", v2);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_filtered, 0, 255, 0); // green
	//viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, single_color, "sample cloud",v2);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud",v2);
	//viewer->addCube(minPt.x, maxPt.x, minPt.y, maxPt.y, minPt.z, maxPt.z, 0.0, 0.0, 1.0, "AABB",v2);
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "AABB",v2);
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "AABB",v2);



	
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}