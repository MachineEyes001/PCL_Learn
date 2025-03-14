#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void pcd2txt(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, char* filename)
{
	FILE* wc = fopen(filename, "w");

	int sizepcd = cloud->points.size();
	for (int i = 0; i < sizepcd; i++)
	{
		//fprintf(stdout, "%f\t%f\t%f\n", cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);  \t �൱�ڿո�
		fprintf(wc, "%f\t%f\t%f\n", cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);  //��׼�����   stdio.h   д
	}

	fclose(wc);
}