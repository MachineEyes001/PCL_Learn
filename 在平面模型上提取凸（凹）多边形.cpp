#include <pcl/ModelCoefficients.h>           //����һ����ģ�������ͷ�ļ�
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>          //�˲������ͷ�ļ�
#include <pcl/segmentation/sac_segmentation.h>   //���ڲ���һ���Էָ��ඨ���ͷ�ļ�
#include <pcl/surface/concave_hull.h>                 //������������ඨ��ͷ�ļ�
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read("G:/vsdata/PCLlearn/PCDdata/table_scene_mug_stereo_textured.pcd", *cloud);
    // ����������������ɢ��NaN
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);                  //�����������
    pass.setFilterFieldName("z");             //���÷ָ��ֶ�Ϊz����
    pass.setFilterLimits(0, 1.1);             //���÷ָֵΪ(0, 1.1)
    pass.filter(*cloud_filtered);
    std::cerr << "PointCloud after filtering has: "
        << cloud_filtered->points.size() << " data points." << std::endl;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);   //inliers�洢�ָ��ĵ���
    // �����ָ����
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // �����Ż�ϵ�����ò���Ϊ��ѡ����
    seg.setOptimizeCoefficients(true);
    // ǿ���Ե�
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    std::cerr << "PointCloud after segmentation has: "
        << inliers->indices.size() << " inliers." << std::endl;

    // ͶӰģ��inliers
    pcl::ProjectInliers<pcl::PointXYZ> proj;//����ͶӰ�˲�ģ��
    proj.setModelType(pcl::SACMODEL_PLANE); //����ͶӰģ��
    proj.setIndices(inliers);
    proj.setInputCloud(cloud_filtered);
    proj.setModelCoefficients(coefficients);      //�����Ƶõ���ƽ��coefficients��������ΪͶӰƽ��ģ��ϵ��
    proj.filter(*cloud_projected);            //�õ�ͶӰ��ĵ���
    std::cerr << "PointCloud after projection has: "
        << cloud_projected->points.size() << " data points." << std::endl;

    // �洢��ȡ������ϵĵ���
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;        //�����������ȡ����
    chull.setInputCloud(cloud_projected);       //�����������Ϊ��ȡ�����
    chull.setAlpha(0.1);
    chull.reconstruct(*cloud_hull);   //������ȡ�����������

    std::cerr << "Concave hull has: " << cloud_hull->points.size()
        << " data points." << std::endl;

    pcl::PCDWriter writer;
    writer.write("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);
    writer.write("table_scene_mug_stereo_cloud_filtered.pcd", *cloud_filtered, false);
    writer.write("table_scene_mug_stereo_cloud_projected.pcd", *cloud_projected, false);
    // ���ӻ�
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("before"));
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "before");
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("filtered"));
    viewer1->addPointCloud<pcl::PointXYZ>(cloud_filtered, "filtered");
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("projected"));
    viewer2->addPointCloud<pcl::PointXYZ>(cloud_projected, "projected");
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("hull"));
    viewer3->addPointCloud<pcl::PointXYZ>(cloud_hull, "hull");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }
    return (0);
}