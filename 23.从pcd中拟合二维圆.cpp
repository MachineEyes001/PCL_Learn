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
#include <pcl/filters/project_inliers.h> //ͶӰ�˲�
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
	//------------------------------��ȡ��������---------------------------------
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ >);
	pcl::PCDReader reader;
	reader.read("G:/vsdata/PCLvision/CloudData/test.PCD", *cloud);
	cout << "PointCloud has: " << cloud->points.size() << " data points." << endl;
	//--------------------------------ֱͨ�˲�-----------------------------------
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ >);
	pcl::PassThrough<pcl::PointXYZ > pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("y");//��Y�᲻�ڣ�0��1.5����Χ�ڵĵ���˵�
	pass.setFilterLimits(0, 1.5);
	pass.filter(*cloud_filtered);//ʣ��ĵ㴢����cloud_filtered�к���ʹ��
	cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << endl;
	//-----------------------------�洢���Ƶ�����ļ�----------------------------
	pcl::PCDWriter writer;
	writer.write("laptop.pcd", *cloud_filtered, false);

	// --------------------------����AABB��Χ�ж�������-------------------------
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*cloud_filtered, minPt, maxPt);
	cout << "�ʼǱ���ǵĿ�Ϊ" << maxPt.z - minPt.z << "cm" << endl;
	cout << "�ʼǱ���ǵĳ�Ϊ" << maxPt.x - minPt.x << "cm" << endl;
	cout << "�ʼǱ���ǵĸ�Ϊ" << maxPt.y - minPt.y << "cm"<<endl;

	//--------------------------------���㷨��-----------------------------------
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ >::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ >);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	n.setSearchMethod(tree);
	n.setInputCloud(cloud_filtered);
	n.setKSearch(50);
	n.compute(*normals);
	//------------------------------�����ָ����---------------------------------
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
	//-------------------------------��ȡԲ����ģ��------------------------------
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ >);
	pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
	//ΪԲ����ָ���ָ���󣬲����ò���
	seg.setOptimizeCoefficients(true);        //���öԹ��Ƶ�ģ��ϵ����Ҫ�����Ż�
	seg.setModelType(pcl::SACMODEL_CYLINDER); //���÷ָ�ģ��ΪԲ����
	seg.setMethodType(pcl::SAC_RANSAC);       //���ò���RANSAC��Ϊ�㷨�Ĳ������Ʒ���
	seg.setNormalDistanceWeight(0.1);         //���ñ��淨��Ȩ��ϵ��
	seg.setMaxIterations(5000);               //���õ�����������
	seg.setDistanceThreshold(1);           //�����ڵ㵽ģ�͵ľ����������ֵ 
	seg.setRadiusLimits(50, 55);              //���ù��Ƴ�Բ��ģ�͵İ뾶��Χ
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(normals);
	//��ȡԲ��ģ��ϵ����Բ���ϵĵ�
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	cout << "Cylinder coefficients: " << *coefficients_cylinder << endl;
	//-----------------------------�洢���Ƶ�����ļ�----------------------------
	pcl::ExtractIndices<pcl::PointXYZ > extract;
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ >);
	extract.filter(*cloud_cylinder);
	cout << "�׵İ뾶Ϊ" << coefficients_cylinder->values[6] <<"cm"<< endl;
	// -------------------------------������ӻ�--------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->setWindowName("������Ƶ�AABB��Χ��");
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


	//// -------------------------------������ӻ�--------------------------------
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D"));
	//viewer->setWindowName("������Ƶ�AABB��Χ��");
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
	//viewer->addText("AABB��Χ��", 10, 10, "v2_text", v2);
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