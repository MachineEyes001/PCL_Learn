#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>

int main(int argc, char** argv)
{
    // �½����ƴ洢����
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // ���ļ���ȡ����ͼ
    pcl::PCDReader reader;
    reader.read("G:/vsdata/PCLlearn/PCDdata/bun0.pcd", *cloud);
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
        << " data points (" << pcl::getFieldsList(*cloud) << ").";

    // �˲�����
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    //������������
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    filter.setSearchMethod(kdtree);
    //������������İ뾶Ϊ3cm
    filter.setSearchRadius(0.03);
    // Upsampling �����ķ����� DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
    filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    // �����İ뾶��
    filter.setUpsamplingRadius(0.03);
    // ���������Ĵ�С
    filter.setUpsamplingStepSize(0.02);
    filter.process(*filteredCloud);

    std::cerr << "PointCloud after filtering: " << filteredCloud->width * filteredCloud->height
        << " data points (" << pcl::getFieldsList(*filteredCloud) << ").";
}