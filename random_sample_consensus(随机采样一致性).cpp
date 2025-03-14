#include <iostream>
#include <thread>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr final = nullptr) {
    // --------------------------------------------
    // ------------打开3D视图并添加点云------------
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

    if (final != nullptr) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(final, 255, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(final, color_handler, "final cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "final cloud");
    }

    viewer->addCoordinateSystem(1.0, "global");
    viewer->initCameraParameters();
    return (viewer);
}
/**
 * 使用方法：
 * random_sample_consensus     创建包含外部点的平面
 * random_sample_consensus -f  创建包含外部点的平面，并计算平面内部点
 * random_sample_consensus -s  创建包含外部点的球体
 * random_sample_consensus -sf 创建包含外部点的球体，并计算球体内部点
 */
int main(int argc, char** argv) 
{
    std::string select = "";
    std::cout << "please input '-f' or '-sf':";
    std::cin >> select;
    // 初始化点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

    // 用点填充PointCloud
    cloud->width = 5000;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    for (std::size_t i = 0; i < cloud->points.size(); ++i) {
        if (select == "-s" || select == "-sf") {
            cloud->points[i].x = 10 * rand() / (RAND_MAX + 1.0);
            cloud->points[i].y = 10 * rand() / (RAND_MAX + 1.0);
            if (i % 5 == 0)     // 可能会散落在球体外
                cloud->points[i].z = 10 * rand() / (RAND_MAX + 1.0);  //此处对应的点为局外点
            else if (i % 2 == 0)// 在球体正方向内
                cloud->points[i].z = sqrt(1 - (cloud->points[i].x * cloud->points[i].x)
                    - (cloud->points[i].y * cloud->points[i].y));
            else // 在球体负方向内
                cloud->points[i].z = -sqrt(1 - (cloud->points[i].x * cloud->points[i].x)
                    - (cloud->points[i].y * cloud->points[i].y));
        }
        else
        {   //用x+y+z=1设置一部分点云数据，此时地拿云组成的菱形平面作为内点
            cloud->points[i].x = 10 * rand() / (RAND_MAX + 1.0);
            cloud->points[i].y = 10 * rand() / (RAND_MAX + 1.0);
            if (i % 2 == 0)
                cloud->points[i].z = 10 * rand() / (RAND_MAX + 1.0);   //对应的局外点
            else
                cloud->points[i].z = 1 * (cloud->points[i].x + cloud->points[i].y);
        }
    }
    std::vector<int> inliers;

    // 创建RandomSampleConsensus对象并计算适当的模型
    // 圆形
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
        model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
    // 平面
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
        model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    if (select == "-f") {
        //根据命令行参数，来随机估算对应平面模型，并存储估计的局内点
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
        ransac.setDistanceThreshold(0.5);    //与平面距离小于0.01 的点称为局内点考虑
        ransac.computeModel();                   //执行随机参数估计
        ransac.getInliers(inliers);                 //存储估计所得的局内点
    }
    else if (select == "-sf") {
        //根据命令行参数  来随机估算对应的圆球模型，存储估计的内点
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
        ransac.setDistanceThreshold(0.5);
        ransac.computeModel();
        ransac.getInliers(inliers);
    }

    // 将计算的模型的所有inlier复制到final点云中
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

    // 创建可视化对象并添加原始点云或所有inliers
    // 取决于指定的命令行参数
    pcl::visualization::PCLVisualizer::Ptr viewer;
    if (select == "-f" || select == "-sf")
        viewer = simpleVis(cloud, final);
    else
        viewer = simpleVis(cloud);
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
    return 0;
}