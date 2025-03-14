#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointType;

int main()
{
    // ��ȡԭʼ��������
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    pcl::PCDReader reader;
    reader.read("G:/vsdata/PCLlearn/PCDdata/test.pcd", *cloud);

    // ����Բ�ĺͰ뾶
    float centerX = 0.0f;
    float centerY = 0.0f;
    float radius = 9.0f;
    float minZ = -10.0f;
    float maxZ = 10.0f;

    // ����һ���˲�������
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");  // �����˲��ֶ�Ϊ X ��
    pass.setFilterLimits(centerX - radius, centerX + radius);  // �����˲���Χ
    pcl::PointCloud<PointType>::Ptr filteredCloud(new pcl::PointCloud<PointType>);
    pass.filter(*filteredCloud);

    pass.setInputCloud(filteredCloud);
    pass.setFilterFieldName("y");  // �����˲��ֶ�Ϊ Y ��
    pass.setFilterLimits(centerY - radius, centerY + radius);  // �����˲���Χ
    pass.filter(*filteredCloud);

    pass.setInputCloud(filteredCloud);
    pass.setFilterFieldName("z");  // �����˲��ֶ�Ϊ Z ��
    pass.setFilterLimits(minZ, maxZ);  // �����˲���Χ
    pass.filter(*filteredCloud);


    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singelColor(filteredCloud, 255, 0, 0);
    viewer.addPointCloud(filteredCloud, singelColor, "filter", 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, "filter");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}