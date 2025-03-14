//#include <iostream>
//#include <boost/thread/thread.hpp>
//#include <pcl/range_image/range_image.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/range_image_visualizer.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/features/range_image_border_extractor.h>
//#include <pcl/keypoints/narf_keypoint.h>
//#include <pcl/console/parse.h>
//#include <pcl/common/file_io.h>
//
//typedef pcl::PointXYZ PointType;
//
//// --------------------
//// --------参数--------
//// --------------------
//float angular_resolution = 0.5f;
//float support_size = 0.2f;
//pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
//bool setUnseenToMaxRange = false;
//
//// --------------
//// -----帮助-----
//// --------------
//void printUsage(const char* progName) {
//    std::cout << "\n\nUsage: " << progName << " [options] <scene.pcd>\n\n"
//        << "Options:\n"
//        << "-------------------------------------------\n"
//        << "-r <float>   angular resolution in degrees (default " << angular_resolution << ")\n"
//        << "-c <int>     coordinate frame (default " << (int)coordinate_frame << ")\n"
//        << "-m           Treat all unseen points as maximum range readings\n"
//        << "-s <float>   support size for the interest points (diameter of the used sphere - "
//        << "default " << support_size << ")\n"
//        << "-h           this help\n"
//        << "\n\n";
//}
//
//int main(int argc, char** argv) 
//{
//    std::string select = "";
//    std::cout << "please input '-h' or '-m':";
//    std::cin >> select;
//    // --------------------------------------
//    // --------------命令行参数--------------
//    // --------------------------------------
//    if (select == "-h")
//    {
//        printUsage(argv[0]);
//        return 0;
//    }
//    if (select == "-m")
//    {
//        setUnseenToMaxRange = true;
//        cout << "Setting unseen values in range image to maximum range readings.\n";
//    }
//    int tmp_coordinate_frame;
//    if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0) {
//        coordinate_frame = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
//        cout << "Using coordinate frame " << (int)coordinate_frame << ".\n";
//    }
//    if (pcl::console::parse(argc, argv, "-s", support_size) >= 0)
//        cout << "Setting support size to " << support_size << ".\n";
//    if (pcl::console::parse(argc, argv, "-r", angular_resolution) >= 0)
//        cout << "Setting angular resolution to " << angular_resolution << "deg.\n";
//    angular_resolution = pcl::deg2rad(angular_resolution);
//
//    // ------------------------------------------------------------------
//    // -------------读取pcd文件或创建示例点云（如果未给出）--------------
//    // ------------------------------------------------------------------
//    pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
//    pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
//    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
//    Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
//    std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");
//    if (!pcd_filename_indices.empty())
//    {
//        std::string filename = argv[pcd_filename_indices[0]];
//        if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
//        {
//            cout << "Was not able to open file \"" << filename << "\".\n";
//            printUsage(argv[0]);
//            return 0;
//        }
//        scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
//            point_cloud.sensor_origin_[1],
//            point_cloud.sensor_origin_[2])) *
//            Eigen::Affine3f(point_cloud.sensor_orientation_);
//
//        std::string far_ranges_filename = pcl::getFilenameWithoutExtension(filename) + "_far_ranges.pcd";
//        if (pcl::io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1)
//            std::cout << "Far ranges file \"" << far_ranges_filename << "\" does not exists.\n";
//    }
//    else
//    {
//        //cout << "\nNo *.pcd file given => Generating example point cloud.\n\n";
//        //for (float x = -0.5f; x <= 0.5f; x += 0.01f)
//        //{
//        //    for (float y = -0.5f; y <= 0.5f; y += 0.01f)
//        //    {
//        //        PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
//        //        point_cloud.points.push_back(point);
//        //    }
//        //}
//        //point_cloud.width = (int)point_cloud.points.size();  point_cloud.height = 1;
//        std::string filename = "G:/vsdata/PCLlearn/PCDdata/table_scene_lms400_downsampled.pcd";
//        if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
//        {
//            cout << "Was not able to open file \"" << filename << "\".\n";
//            printUsage(argv[0]);
//            return 0;
//        }
//        scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
//            point_cloud.sensor_origin_[1],
//            point_cloud.sensor_origin_[2])) *
//            Eigen::Affine3f(point_cloud.sensor_orientation_);
//
//        std::string far_ranges_filename = pcl::getFilenameWithoutExtension(filename) + "_far_ranges.pcd";
//        if (pcl::io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1)
//            std::cout << "Far ranges file \"" << far_ranges_filename << "\" does not exists.\n";
//    }
//
//    // -----------------------------------------------
//    // ---------------从点云创建深度图----------------
//    // -----------------------------------------------
//    float noise_level = 0.0;
//    float min_range = 0.0f;
//    int border_size = 1;
//    boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
//    pcl::RangeImage& range_image = *range_image_ptr;
//    range_image.createFromPointCloud(point_cloud, angular_resolution,
//        pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
//        scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
//    range_image.integrateFarRanges(far_ranges);
//    if (setUnseenToMaxRange)
//        range_image.setUnseenToMaxRange();
//
//    // --------------------------------------------
//    // ------------打开3D视图并添加点云------------
//    // --------------------------------------------
//    pcl::visualization::PCLVisualizer viewer("3D Viewer");
//    viewer.setBackgroundColor(1, 1, 1);
//    viewer.addCoordinateSystem(1.0f, "global");
//    pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
//    viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
//    viewer.initCameraParameters();
//
//    // --------------------------
//    // --------显示深度图--------
//    // --------------------------
//    pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
//    range_image_widget.showRangeImage(range_image);
//
//    // --------------------------------
//    // --------提取 NARF 关键点--------
//    // --------------------------------
//    pcl::RangeImageBorderExtractor range_image_border_extractor;  //创建了一个RangeImageBorderExtractor对象，用于提取边界特征
//    pcl::NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);  //创建了一个NarfKeypoint对象，用于进行NARF关键点检测。通过将之前创建的RangeImageBorderExtractor对象传递给构造函数，指定了在关键点检测过程中使用的边界提取器
//    narf_keypoint_detector.setRangeImage(&range_image);  //将输入的范围图像range_image设置为关键点检测器的范围图像
//    narf_keypoint_detector.getParameters().support_size = support_size;  //设置关键点检测器的参数中的support_size属性，表示关键点周围的支持区域大小
//    narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;  //设置关键点检测器的参数中的add_points_on_straight_edges属性为true，表示在直线边缘上添加额外的关键点
//    narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;  //设置关键点检测器的参数中的distance_for_additional_points属性，表示添加额外关键点的最大距离
//
//    pcl::PointCloud<int> keypoint_indices;
//    narf_keypoint_detector.compute(keypoint_indices);
//    std::cout << "Found " << keypoint_indices.points.size() << " key points.\n";
//
//    // ----------------------------------------------
//    // ------------显示深度图窗中的关键点------------
//    // ----------------------------------------------
//    pcl::visualization::Vector3ub fg_color(255, 0, 0);   // 前景颜色为红色
//    pcl::visualization::Vector3ub bg_color(255, 255, 255);   // 背景颜色为白色
//    double radius = 3.0;    // 半径为3.0
//    std::string layer_id = "points";    // 图层ID
//    double opacity = 1.0;    // 不透明度为1.0
//    for (size_t i = 0; i < keypoint_indices.points.size(); ++i)
//    {
//        /*在PCL中，深度图像（Range Image）的坐标系通常以图像的左上角为原点，横轴向右延伸，纵轴向下延伸。而在一些其他计算机视觉库或者图像处理软件中，常见的坐标系定义方式是以图像的左下角为原点，横轴向右延伸，纵轴向上延伸。
//        因此，在使用markPoint()函数标记点时，如果将范围图像中的x坐标和y坐标直接传递给该函数的参数，就会导致标记的点与实际点相对位置上下颠倒。
//        因此，此处将范围图像中的y坐标减去其高度后再传递给markPoint()函数，即：range_image.height - keypoint_indices.points[i] / range_image.width。这样可以保证标记的点与实际点在y轴上保持一致。*/
//        range_image_widget.markPoint(keypoint_indices.points[i] % range_image.width, range_image.height - keypoint_indices.points[i] / range_image.width, fg_color, bg_color, radius, layer_id, opacity);
//    }
//    // -------------------------------------
//    // ---------在3D视图中显示关键点--------
//    // -------------------------------------
//    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
//    keypoints.points.resize(keypoint_indices.points.size());
//    for (size_t i = 0; i < keypoint_indices.points.size(); ++i)
//        keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(keypoints_ptr, 0, 255, 0);
//    viewer.addPointCloud<pcl::PointXYZ>(keypoints_ptr, keypoints_color_handler, "keypoints");
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");
//
//    while (!viewer.wasStopped()) {
//        range_image_widget.spinOnce();  // 处理GUI事件
//        viewer.spinOnce();
//        pcl_sleep(0.01);
//    }
//}

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
using namespace std;

namespace pcl
{
    template<> struct SIFTKeypointFieldSelector<PointXYZ>
    {
        inline float
            operator () (const PointXYZ& p) const
        {
            return p.z;
        }
    };
}

int main(int argc, char* argv[])
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("G:/vsdata/PCLlearn/PCDdata/table_scene_lms400_downsampled.pcd", *cloud_xyz);

    const float min_scale = 0.01;
    const int n_octaves = 6;
    const int n_scales_per_octave = 4;
    const float min_contrast = 0.01;

    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift; //创建sift关键点检测对象
    pcl::PointCloud<pcl::PointWithScale> result;
    sift.setInputCloud(cloud_xyz);                              //设置输入点云
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());    //创建一个空的kd树对象tree
    sift.setSearchMethod(tree);                                 //把kd树对象传递给sift检测对象
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);  //指定搜索关键点的尺度范围
    sift.setMinimumContrast(min_contrast);                      //设置限制关键点检测的阈值
    sift.compute(result);                                       //执行sift关键点检测，保存结果在result

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(result, *cloud_temp);                        //将点类型pcl::PointWithScale的数据转换为点类型pcl::PointXYZ的数据

    //可视化输入点云和关键点
    pcl::visualization::PCLVisualizer viewer("Sift keypoint");
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addPointCloud(cloud_xyz, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "cloud");
    viewer.addPointCloud(cloud_temp, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "keypoints");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}
