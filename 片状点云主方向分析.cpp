#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/pca.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("F:/Paper/test/12-29-38-cloud-stand-ECE-GR.pcd", *pointCloud) == -1) //* �����ļ�
    {
    	return false;
    }//����

    // ��ȡ���ĺͰ뾶
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;// �������Ծع����������������ƣ������м���
    feature_extractor.setInputCloud(pointCloud);
    feature_extractor.compute();
    Eigen::Vector3f mass_center;//����
    feature_extractor.getMassCenter(mass_center);
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    float radius = (max_point_OBB.x - min_point_OBB.x + max_point_OBB.y - min_point_OBB.y) / 4;//�뾶

    // ������ƽ�淨��
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);// ���߶���
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation; // ���ڷ��߹��ƵĶ���
    normalEstimation.setInputCloud(pointCloud);
    normalEstimation.setRadiusSearch(0.3); // ����ÿ���㣬ʹ�ð뾶ΪSearchRadius����������
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);// ͨ��kd��ʹ�������Ӹ�Ч
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);  // ���㷨��

    Eigen::Vector3f mainDirection(0.0, 0.0, 0.0);
    // �������з��߷�����ܺ�
    for (size_t i = 0; i < normals->size(); ++i)
    {
        mainDirection.x() += normals->points[i].normal_x;
        mainDirection.y() += normals->points[i].normal_y;
        mainDirection.z() += normals->points[i].normal_z;
    }
    // ��һ���õ�ƽ�����߷�����Ϊ������
    mainDirection.normalize();
    // ���������
    std::cout << "������" << mainDirection.x() << ", " << mainDirection.y() << ", " << mainDirection.z() << std::endl;
    
    // ��ӡ���
    std::cout << "�������꣺" << mass_center(0) << ", " << mass_center(1) << ", " << mass_center(2) << std::endl;
    std::cout << "�뾶��" << radius << std::endl;

    pcl::visualization::PCLVisualizer viewer;
    // ������ʾ����
    pcl::PointXYZRGB selected_point;
    selected_point.x = mass_center(0);
    selected_point.y = mass_center(1);
    selected_point.z = mass_center(2);
    selected_point.r = 255;
    selected_point.g = 0;
    selected_point.b = 0;
    viewer.addSphere(selected_point, 0.005, 255, 0, 0, "center_point");
    viewer.addPointCloud(pointCloud);
    viewer.addCoordinateSystem();
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    return 0;
}
