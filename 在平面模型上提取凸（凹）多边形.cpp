#include <pcl/ModelCoefficients.h>           //采样一致性模型相关类头文件
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>          //滤波相关类头文件
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割类定义的头文件
#include <pcl/surface/concave_hull.h>                 //创建凹多边形类定义头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read("G:/vsdata/PCLlearn/PCDdata/table_scene_mug_stereo_textured.pcd", *cloud);
    // 创建过滤器消除杂散的NaN
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);                  //设置输入点云
    pass.setFilterFieldName("z");             //设置分割字段为z坐标
    pass.setFilterLimits(0, 1.1);             //设置分割阀值为(0, 1.1)
    pass.filter(*cloud_filtered);
    std::cerr << "PointCloud after filtering has: "
        << cloud_filtered->points.size() << " data points." << std::endl;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);   //inliers存储分割后的点云
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 设置优化系数，该参数为可选参数
    seg.setOptimizeCoefficients(true);
    // 强制性的
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    std::cerr << "PointCloud after segmentation has: "
        << inliers->indices.size() << " inliers." << std::endl;

    // 投影模型inliers
    pcl::ProjectInliers<pcl::PointXYZ> proj;//点云投影滤波模型
    proj.setModelType(pcl::SACMODEL_PLANE); //设置投影模型
    proj.setIndices(inliers);
    proj.setInputCloud(cloud_filtered);
    proj.setModelCoefficients(coefficients);      //将估计得到的平面coefficients参数设置为投影平面模型系数
    proj.filter(*cloud_projected);            //得到投影后的点云
    std::cerr << "PointCloud after projection has: "
        << cloud_projected->points.size() << " data points." << std::endl;

    // 存储提取多边形上的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;        //创建多边形提取对象
    chull.setInputCloud(cloud_projected);       //设置输入点云为提取后点云
    chull.setAlpha(0.1);
    chull.reconstruct(*cloud_hull);   //创建提取创建凹多边形

    std::cerr << "Concave hull has: " << cloud_hull->points.size()
        << " data points." << std::endl;

    pcl::PCDWriter writer;
    writer.write("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);
    writer.write("table_scene_mug_stereo_cloud_filtered.pcd", *cloud_filtered, false);
    writer.write("table_scene_mug_stereo_cloud_projected.pcd", *cloud_projected, false);
    // 可视化
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