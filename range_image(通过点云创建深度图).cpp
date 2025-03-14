#include <pcl/range_image/range_image.h>    //深度图像的头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>& pointCloud = *pointCloudPtr;

    // 循环产生点云的数据
    for (float y = -0.5f; y <= 0.5f; y += 0.01f) {
        for (float z = -0.5f; z <= 0.5f; z += 0.01f) {
            pcl::PointXYZ point;
            point.x = 2.0f - y;
            point.y = y;
            point.z = z;
            pointCloud.points.push_back(point); //循环添加点数据到点云对象
        }
    }
    pointCloud.width = (uint32_t)pointCloud.points.size();
    pointCloud.height = 1; //设置点云对象的头信息
    pcl::PCDWriter writer;
    writer.write("depthtest.pcd", *pointCloudPtr, true);
    //根据之前得到的点云图，通过1deg的分辨率生成深度图
    //angular_resolution为模拟的深度传感器的角度分辨率，即深度图像中一个像素对应的角度大小
    float angularResolution = (float)(1.0f * (M_PI / 180.0f));  // 弧度1°
    //max_angle_width为模拟的深度传感器的水平最大采样角度
    float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));  // 弧度360°
    //max_angle_height为模拟传感器的垂直方向最大采样角度,都转为弧度
    float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 弧度180°
    //传感器的采集位置
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    //深度图像遵循坐标系统
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel = 0.00;    //noise_level获取深度图像深度时，近邻点对查询点距离值的影响水平
    float minRange = 0.0f;    //min_range设置最小的获取距离，小于最小获取距离的位置为传感器的盲区
    int borderSize = 1;     //border_size获得深度图像的边缘的宽度

    boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage& rangeImage = *range_image_ptr;
    rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    std::cout << rangeImage << "\n";

    // --------------------------------------------
    // ----------------显示3D点云------------------
    // --------------------------------------------
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(1, 1, 1);
    // 添加原始点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> org_image_color_handler(pointCloudPtr, 255, 100, 0);
    viewer.addPointCloud(pointCloudPtr, org_image_color_handler, "orginal image");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "orginal image");
    viewer.initCameraParameters();
    viewer.addCoordinateSystem(1.0);

    // --------------------------
    // -----显示深度图-----
    // --------------------------
    //用以图像的方式可视化深度图像，图像的颜色取决于深度值
    pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
    range_image_widget.showRangeImage(rangeImage);      //图像可视化方式显示深度图像
    range_image_widget.setSize(800, 600);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return (0);
}
