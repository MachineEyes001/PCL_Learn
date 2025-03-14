#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv)
{
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

    // ���ļ���ȡ����ͼ
    pcl::PCDReader reader;
    reader.read("G:/vsdata/PCLlearn/PCDdata/table_scene_lms400.pcd", *cloud);

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
        << " data points (" << pcl::getFieldsList(*cloud) << ").";

    // ����һ������߷ֱ���1cm�����ع�������cloud��Ϊ�������ݣ�cloud_filtered��Ϊ�������
    float leftSize = 0.01f;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leftSize, leftSize, leftSize);
    sor.filter(*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
        << " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";

    // �����������ļ�
    pcl::PCDWriter writer;
    writer.write("G:/vsdata/PCLlearn/PCDdata/table_scene_lms400_downsampled.pcd", *cloud_filtered);

    return (0);
}