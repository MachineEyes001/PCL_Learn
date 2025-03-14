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

    // ���ļ���ȡ����ͼ
    pcl::PCDReader reader;
    reader.read("G:/vsdata/PCLlearn/PCDdata/bun0.pcd", *cloud);

    // ƽ������ѡ��Ҫ��Ϊ���������ĵ����ͣ�
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> filter;
    filter.setInputCloud(cloud);
    // ʹ�ð뾶Ϊ5���׵���������
    filter.setSearchRadius(0.05);
    // ���ò���
    filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);//�����ϲ�������Ϊ�ֲ�ƽ������������ݾֲ�ƽ������������µĵ�
    filter.setUpsamplingRadius(0.03);//�����ϲ����뾶������ȷ������ʱ���ǵ�����Χ
    filter.setUpsamplingStepSize(0.02);//�����ϲ�������������ȷ�������ɵ�ļ������
    filter.setComputeNormals(true);// �����Ƿ����ƽ����ĵ��Ƶķ�����Ϣ����ѡ��
    // ��������������������kd�����󣬼���������ȷ�������������������
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    filter.setSearchMethod(kdtree);
    filter.process(*smoothedCloud);//��������ƽ���ƽ������������ؽ�����������洢��smoothedCloud��
    // ���ӻ�
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