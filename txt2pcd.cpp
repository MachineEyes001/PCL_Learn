#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr readCloudTxt(char* Filename)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	std::ifstream Points_in(Filename);
	pcl::PointXYZ tmpoint;
	if (Points_in.is_open())
	{
		while (!Points_in.eof())   //��δ�����ļ���β
		{
			Points_in >> tmpoint.x >> tmpoint.y >> tmpoint.z;
			basic_cloud_ptr->points.push_back(tmpoint);
		}
	}
	basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
	basic_cloud_ptr->height = 1;

	return basic_cloud_ptr;
}