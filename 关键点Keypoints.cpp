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
//// --------����--------
//// --------------------
//float angular_resolution = 0.5f;
//float support_size = 0.2f;
//pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
//bool setUnseenToMaxRange = false;
//
//// --------------
//// -----����-----
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
//    // --------------�����в���--------------
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
//    // -------------��ȡpcd�ļ��򴴽�ʾ�����ƣ����δ������--------------
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
//    // ---------------�ӵ��ƴ������ͼ----------------
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
//    // ------------��3D��ͼ����ӵ���------------
//    // --------------------------------------------
//    pcl::visualization::PCLVisualizer viewer("3D Viewer");
//    viewer.setBackgroundColor(1, 1, 1);
//    viewer.addCoordinateSystem(1.0f, "global");
//    pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
//    viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
//    viewer.initCameraParameters();
//
//    // --------------------------
//    // --------��ʾ���ͼ--------
//    // --------------------------
//    pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
//    range_image_widget.showRangeImage(range_image);
//
//    // --------------------------------
//    // --------��ȡ NARF �ؼ���--------
//    // --------------------------------
//    pcl::RangeImageBorderExtractor range_image_border_extractor;  //������һ��RangeImageBorderExtractor����������ȡ�߽�����
//    pcl::NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);  //������һ��NarfKeypoint�������ڽ���NARF�ؼ����⡣ͨ����֮ǰ������RangeImageBorderExtractor���󴫵ݸ����캯����ָ�����ڹؼ����������ʹ�õı߽���ȡ��
//    narf_keypoint_detector.setRangeImage(&range_image);  //������ķ�Χͼ��range_image����Ϊ�ؼ��������ķ�Χͼ��
//    narf_keypoint_detector.getParameters().support_size = support_size;  //���ùؼ��������Ĳ����е�support_size���ԣ���ʾ�ؼ�����Χ��֧�������С
//    narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;  //���ùؼ��������Ĳ����е�add_points_on_straight_edges����Ϊtrue����ʾ��ֱ�߱�Ե����Ӷ���Ĺؼ���
//    narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;  //���ùؼ��������Ĳ����е�distance_for_additional_points���ԣ���ʾ��Ӷ���ؼ����������
//
//    pcl::PointCloud<int> keypoint_indices;
//    narf_keypoint_detector.compute(keypoint_indices);
//    std::cout << "Found " << keypoint_indices.points.size() << " key points.\n";
//
//    // ----------------------------------------------
//    // ------------��ʾ���ͼ���еĹؼ���------------
//    // ----------------------------------------------
//    pcl::visualization::Vector3ub fg_color(255, 0, 0);   // ǰ����ɫΪ��ɫ
//    pcl::visualization::Vector3ub bg_color(255, 255, 255);   // ������ɫΪ��ɫ
//    double radius = 3.0;    // �뾶Ϊ3.0
//    std::string layer_id = "points";    // ͼ��ID
//    double opacity = 1.0;    // ��͸����Ϊ1.0
//    for (size_t i = 0; i < keypoint_indices.points.size(); ++i)
//    {
//        /*��PCL�У����ͼ��Range Image��������ϵͨ����ͼ������Ͻ�Ϊԭ�㣬�����������죬�����������졣����һЩ����������Ӿ������ͼ��������У�����������ϵ���巽ʽ����ͼ������½�Ϊԭ�㣬�����������죬�����������졣
//        ��ˣ���ʹ��markPoint()������ǵ�ʱ���������Χͼ���е�x�����y����ֱ�Ӵ��ݸ��ú����Ĳ������ͻᵼ�±�ǵĵ���ʵ�ʵ����λ�����µߵ���
//        ��ˣ��˴�����Χͼ���е�y�����ȥ��߶Ⱥ��ٴ��ݸ�markPoint()����������range_image.height - keypoint_indices.points[i] / range_image.width���������Ա�֤��ǵĵ���ʵ�ʵ���y���ϱ���һ�¡�*/
//        range_image_widget.markPoint(keypoint_indices.points[i] % range_image.width, range_image.height - keypoint_indices.points[i] / range_image.width, fg_color, bg_color, radius, layer_id, opacity);
//    }
//    // -------------------------------------
//    // ---------��3D��ͼ����ʾ�ؼ���--------
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
//        range_image_widget.spinOnce();  // ����GUI�¼�
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

    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift; //����sift�ؼ��������
    pcl::PointCloud<pcl::PointWithScale> result;
    sift.setInputCloud(cloud_xyz);                              //�����������
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());    //����һ���յ�kd������tree
    sift.setSearchMethod(tree);                                 //��kd�����󴫵ݸ�sift������
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);  //ָ�������ؼ���ĳ߶ȷ�Χ
    sift.setMinimumContrast(min_contrast);                      //�������ƹؼ��������ֵ
    sift.compute(result);                                       //ִ��sift�ؼ����⣬��������result

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(result, *cloud_temp);                        //��������pcl::PointWithScale������ת��Ϊ������pcl::PointXYZ������

    //���ӻ�������ƺ͹ؼ���
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
