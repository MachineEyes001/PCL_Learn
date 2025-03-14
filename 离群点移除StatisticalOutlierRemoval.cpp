#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // ���ļ���ȡ����
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("G:/vsdata/PCLlearn/PCDdata/table_scene_lms400.pcd", *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // ������������ÿ�����������ʱ���ǵ�����ھӸ���Ϊ50����
    // ���ñ�׼����ֵΪ1������ζ�����о����ѯ���ƽ������ı�׼ƫ�������1����׼ƫ������е㶼�������Ϊ��Ⱥֵ��ɾ����
    // �������������洢��cloud_filtered��

    // �����˲���
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    // ����ƽ��������Ƶ�����ڵ�����K
    sor.setMeanK(50);
    // ���ñ�׼����ֵϵ��
    sor.setStddevMulThresh(1.0);
    // ִ�й���
    sor.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;
    // ���������ĵ㱣�浽��׺Ϊ_inliers.pcd���ļ�
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("G:/vsdata/PCLlearn/PCDdata/table_scene_lms400_inliers.pcd", *cloud_filtered, false);

    // ʹ�ø���ͬ�Ĺ����������Ƕ�������ȡ������õ���Щ�����˵��ĵ㣬���浽_outliers.pcd�ļ�
    sor.setNegative(true);
    sor.filter(*cloud_filtered);
    writer.write<pcl::PointXYZ>("G:/vsdata/PCLlearn/PCDdata/table_scene_lms400_outliers.pcd", *cloud_filtered, false);

    return (0);
}