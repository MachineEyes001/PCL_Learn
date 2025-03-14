#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

int user_data;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
    // ���ñ���ɫΪ�ۺ�ɫ
    viewer.setBackgroundColor(1.0, 0.5, 1.0);
    // ���һ��Բ��Ϊo���뾶Ϊ0.1m������
    pcl::PointXYZ o;
    o.x = 0.2;
    o.y = 0;
    o.z = 0;
    viewer.addSphere(o, 0.1, "sphere", 0);
    std::cout << "i only run once" << std::endl;
}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer) {
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    // ÿ��ˢ��ʱ���Ƴ�text������µ�text
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0);

    //FIXME: possible race condition here:
    user_data++;
}

int main() 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("G:/vsdata/PCLlearn/PCDdata/bun0.pcd", *cloud);
    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    //�����һֱ����ֱ�����Ʊ���Ⱦ
    viewer.showCloud(cloud);

    // ֻ�����һ�� (�Ǳ���)
    viewer.runOnVisualizationThreadOnce(viewerOneOff);
    // ÿ�ο��ӻ������������һ�Σ�Ƶ�����ã� (�Ǳ���)
    viewer.runOnVisualizationThread(viewerPsycho);
    while (!viewer.wasStopped()) {
        user_data++;
    }
    return 0;
}