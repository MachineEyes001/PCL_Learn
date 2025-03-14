#include <pcl/range_image/range_image.h>    //���ͼ���ͷ�ļ�
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>& pointCloud = *pointCloudPtr;

    // ѭ���������Ƶ�����
    for (float y = -0.5f; y <= 0.5f; y += 0.01f) {
        for (float z = -0.5f; z <= 0.5f; z += 0.01f) {
            pcl::PointXYZ point;
            point.x = 2.0f - y;
            point.y = y;
            point.z = z;
            pointCloud.points.push_back(point); //ѭ����ӵ����ݵ����ƶ���
        }
    }
    pointCloud.width = (uint32_t)pointCloud.points.size();
    pointCloud.height = 1; //���õ��ƶ����ͷ��Ϣ
    pcl::PCDWriter writer;
    writer.write("depthtest.pcd", *pointCloudPtr, true);
    //����֮ǰ�õ��ĵ���ͼ��ͨ��1deg�ķֱ����������ͼ
    //angular_resolutionΪģ�����ȴ������ĽǶȷֱ��ʣ������ͼ����һ�����ض�Ӧ�ĽǶȴ�С
    float angularResolution = (float)(1.0f * (M_PI / 180.0f));  // ����1��
    //max_angle_widthΪģ�����ȴ�������ˮƽ�������Ƕ�
    float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));  // ����360��
    //max_angle_heightΪģ�⴫�����Ĵ�ֱ�����������Ƕ�,��תΪ����
    float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // ����180��
    //�������Ĳɼ�λ��
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    //���ͼ����ѭ����ϵͳ
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel = 0.00;    //noise_level��ȡ���ͼ�����ʱ�����ڵ�Բ�ѯ�����ֵ��Ӱ��ˮƽ
    float minRange = 0.0f;    //min_range������С�Ļ�ȡ���룬С����С��ȡ�����λ��Ϊ��������ä��
    int borderSize = 1;     //border_size������ͼ��ı�Ե�Ŀ��

    boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage& rangeImage = *range_image_ptr;
    rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    std::cout << rangeImage << "\n";

    // --------------------------------------------
    // ----------------��ʾ3D����------------------
    // --------------------------------------------
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(1, 1, 1);
    // ���ԭʼ����
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> org_image_color_handler(pointCloudPtr, 255, 100, 0);
    viewer.addPointCloud(pointCloudPtr, org_image_color_handler, "orginal image");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "orginal image");
    viewer.initCameraParameters();
    viewer.addCoordinateSystem(1.0);

    // --------------------------
    // -----��ʾ���ͼ-----
    // --------------------------
    //����ͼ��ķ�ʽ���ӻ����ͼ��ͼ�����ɫȡ�������ֵ
    pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
    range_image_widget.showRangeImage(rangeImage);      //ͼ����ӻ���ʽ��ʾ���ͼ��
    range_image_widget.setSize(800, 600);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return (0);
}
