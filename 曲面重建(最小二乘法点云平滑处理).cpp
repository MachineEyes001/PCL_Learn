#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>  //kd-tree����������ඨ���ͷ�ļ�
#include <pcl/surface/mls.h>        //��С���˷�ƽ�������ඨ��ͷ�ļ�
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

int main(int argc, char** argv) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // ���ļ���ȡ����ͼ
    pcl::PCDReader reader;
    reader.read("G:/vsdata/PCLlearn/PCDdata/bun0.pcd", *cloud);

    // ���� KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // �������PointNormal���ͣ��Ա�洢MLS����ķ���
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);

    // Init���󣨵ڶ��ֵ��������ڷ��ߣ���ʹδʹ�ã�
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    // ���ò���
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.1);

    // �ؽ�
    mls.process(*mls_points);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("before"));
    viewer1->addPointCloud<pcl::PointXYZ>(cloud, "before");
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("smoothed"));
    viewer->addPointCloud<pcl::PointNormal>(mls_points, "smoothed");
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }
    // ����
    pcl::io::savePCDFile("bun0-mls.pcd", *mls_points);
}