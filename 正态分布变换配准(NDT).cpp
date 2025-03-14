#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
//ʹ�á���̬�ֲ��任�㷨���͡������������ݵĹ���������Ӧ��ͷ�ļ�������������������������������滻, 
//����ʹ���������������(approximate voxel filter)�������Ϻ�
#include <pcl/registration/ndt.h> // ndt��׼ͷ�ļ�
#include <pcl/filters/approximate_voxel_grid.h>

using namespace std::chrono_literals;

int main(int argc, char** argv) 
{
    // �����״εķ���ɨ��������ΪĿ����� target_cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("G:/vsdata/PCLlearn/PCDdata/room_scan1.pcd", *target_cloud) == -1) {
        PCL_ERROR("Couldn't read file room_scan1.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << target_cloud->size() << " data points from room_scan1.pcd" << std::endl;

    // ���ش��µ��ӽǵõ��ķ���ڶ���ɨ��������Ϊ����Դ���� input_cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("G:/vsdata/PCLlearn/PCDdata/room_scan2.pcd", *input_cloud) == -1) {
        PCL_ERROR("Couldn't read file room_scan2.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << input_cloud->size() << " data points from room_scan2.pcd" << std::endl;
/*-------------------------------------------------------------------------------------------------------------------------
���ϴ������������ PCD �ļ�������ָ�룬��׼��������� ��Դ����input_cloud������Ŀ�����target_cloud������ϵ�任����Ĺ��㣬
�����input_cloud�任��target_cloud�ı任����
-------------------------------------------------------------------------------------------------------------------------*/
    
    // ����������Ƶ�ԭʼ��С��Լ10%���������׼�ٶ�
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud(input_cloud);
    approximate_voxel_filter.filter(*filtered_cloud);
    std::cout << "Filtered cloud contains " << filtered_cloud->size()
        << " data points from room_scan2.pcd" << std::endl;
/*-------------------------------------------------------------------------------------------------------------------------
���ϴ��뽫����������Ƶ�ԭʼ��С��Լ10%���������׼�ٶȡ��������κ��������ȹ����������ԣ�Ŀ�����target_cloud����Ҫ�����˲�
������ΪNDT�㷨��Ŀ����ƶ�Ӧ������Voxel�������ݽṹ����ʱ����ʹ�õ����㣬����ʹ�����صĵ㡣�������˽���������
-------------------------------------------------------------------------------------------------------------------------*/

    // ʹ��Ĭ��ֵ����NDT�����ڲ����ݽṹֱ���Ժ�Ż��ʼ����
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // �����������ݵĳ߶�����NDT��ز���
    // ������ֹ��������Сת����
    ndt.setTransformationEpsilon(0.01);
    // ����More Thuente����������󲽳�
    ndt.setStepSize(0.1);
    // ����NDT����ṹ�ķֱ��ʣ�VoxelGridCovariance��
    ndt.setResolution(1.0);
/*-------------------------------------------------------------------------------------------------------------------------
���ϴ�������һЩ�߶���صĲ���,��Ϊ NDT �㷨ʹ��һ�����ػ����ݽṹ��More-Thuente�������������Ҫ����һЩ��������Ӧ���ݼ���
���ϲ���������������ʹ�õķ���ߴ���������еغܺã��������������Ҫ��������һ�����ȱ���ɨ��֮���С���壬��Ҫ�Բ������к�
��̶ȵ���С���ڱ任�� Epsilon �����ֱ�ӳ��Ⱥͻ��ȣ������˱任ʸ��[ x, y, z,roll,pitch, yaw]����С��ɵĵ�������һ��������
��С������ٽ�ֵ���� ����ô��׼�㷨�ͽ���ֹ������StepSize���������� More-Thuente �������������󲽳�������������㷨ȷ����
���ֵ���µ���Ѳ��������������Ž�ʱ���㷨�����̵����������ڸ������󲽳������ڽ��ٵĵ��������±����ϴ�ľ��룬����ȴ�й�
�ȵ������ڲ�����Ҫ��ľֲ���Сֵ�������ķ��ա�
-------------------------------------------------------------------------------------------------------------------------*/

    // ������׼������������
    ndt.setMaximumIterations(35);
/*-------------------------------------------------------------------------------------------------------------------------
���MaximumIterations�����������Ż��������е�������������һ����˵���ڴﵽ�������ֵ֮ǰ�Ż�����ͻ��� epsilon �任��ֵ��
��ֹ����Ӵ����������������ܹ����ӳ���³����,��ֹ�����ڴ���ķ�������̫�á�
-------------------------------------------------------------------------------------------------------------------------*/

    // ���ù��˺������Դ���ƣ��ڶ���ɨ�����ݣ�
    ndt.setInputSource(filtered_cloud);
    // ����Ŀ����ƣ���һ��ɨ�����ݣ�����Ϊ������׼��Ŀ��
    ndt.setInputTarget(target_cloud);
/*-------------------------------------------------------------------------------------------------------------------------
������ǰѵ��Ƹ��� NDT ��׼����Ŀ����Ƶ�����ϵ�Ǳ�ƥ���������ƵĲο�����ϵ��ƥ����ɺ�������ƽ����任����Ŀ�����ͬ
һ����ϵ�£�������Ŀ����ƺ�NDT �㷨���ڲ����ݽṹ����ʼ����
-------------------------------------------------------------------------------------------------------------------------*/

    // ����ʹ�û����˲�෨�õ��Ĵ��Գ�ʼ�任����
    Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(1.79387, 0, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
/*-------------------------------------------------------------------------------------------------------------------------
���ϴ�����д�����һ��������׼�任����ĳ�ʼ���ƣ���Ȼ�㷨���в�����Ҫ������һ����ʼ�任���󣬵������������׵õ����õĽ����
�����ǵ��ο�����ϵ֮���нϴ����ʱ���������ǣ����ڻ�����Ӧ�ó��������������ɴ����ݼ���Ӧ�ó����У�ͨ��ʹ����̱�������
�ɳ�ʼת����
-------------------------------------------------------------------------------------------------------------------------*/

    // ��������ĸ���任����ʹ���������Ŀ����ƶ���
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*output_cloud, init_guess);
    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
        << " score: " << ndt.getFitnessScore() << std::endl;
/*-------------------------------------------------------------------------------------------------------------------------
�������׼��������ơ����ɵ�ת�����������ƴ洢����������С�Ȼ��������ʾ����Ľ���Լ�ŷ������ʺ϶ȵ÷�FitnessScore��
�÷�������Ϊ������Ƶ�Ŀ�����������ľ����ƽ����
-------------------------------------------------------------------------------------------------------------------------*/

    // ʹ���ҵ��ı任��������δ���˵�������ƽ��б任
    pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());

    // ����任����������
    pcl::io::savePCDFileASCII("room_scan2_transformed.pcd", *output_cloud);
/*-------------------------------------------------------------------------------------------------------------------------
�ڶ���֮���������output_cloud�������������˺��������Ƶ�ת���汾����Ϊ�������㷨�����˹��˺�ĵ��ƣ�������ԭʼ������ơ�
Ϊ�˻��ԭʼ���ƵĶ���汾����NDT�㷨����ȡ����ת������ת��ԭʼ������ơ����ڣ����Խ����Ʊ��浽�ļ�room_scan2_transformed.pcd
�У��Ա�����ʹ�á�
-------------------------------------------------------------------------------------------------------------------------*/

    // ��ʼ�����ƿ��ӻ�����
    pcl::visualization::PCLVisualizer::Ptr
        viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    // ��ɫ�����ӻ�Ŀ����ƣ���ɫ��
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        target_color(target_cloud, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        1, "target cloud");

    // ��ɫ�����ӻ�ת�����������ƣ���ɫ��
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        output_color(output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        1, "output cloud");

    // ���ӻ�
    viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();
    while (!viewer_final->wasStopped()) {
        viewer_final->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }

    return (0);
}