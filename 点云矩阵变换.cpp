//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/common/transforms.h>
//#include <pcl/ModelCoefficients.h>
//
////����Բ������������ռ�����Բ�������
////param[in]  pcl::ModelCoefficients::Ptr& coefficients_cylinder://Բ���������ϵ��0��1��2����Բ�������ϵ�ԭ�㣬3��4��5�����������ߵķ���������ϵ��6����Բ���İ뾶��
////param[in] valization��Ĭ�Ͽ��ӻ�
////param[out] cloud��PCD��ʽ�ĵ����ļ�
//void CreatCylinder(pcl::ModelCoefficients::Ptr& coefficients_cylinder, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, bool valization = true) {
//	//������
//	if (coefficients_cylinder->values.size() != 7) {
//		std::cerr << "������Ŀ���󡣡���" << std::endl;
//		system("pause");
//	}
//	if (coefficients_cylinder->values[6] <= 0) {
//		std::cerr << "Բ����뾶����С�ڵ���0" << std::endl;
//		system("pause");
//	}
//	
//	Eigen::RowVector3d axis(-coefficients_cylinder->values[3], -coefficients_cylinder->values[4], -coefficients_cylinder->values[5]);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud(new pcl::PointCloud<pcl::PointXYZ>);
//	if (pcl::io::loadPCDFile<pcl::PointXYZ>("F:/Paper/PCLvision/model/cheesecloudmodel.pcd", *modelCloud) == -1) //* �����ļ�
//	{
//		return;
//	}//����
//
//	//ƽ�� �����ʽͲɴ
//	//for (size_t i = 0; i < modelCloud->size(); i++) {
//
//	//	modelCloud->points[i].z = modelCloud->points[i].z + 0.083;
//	//}
//
//	//������ת Z��ת��axis
//	Eigen::RowVector3d  Z(0.0, 0.0, 0.1), T(coefficients_cylinder->values[0] + 0.083*(-coefficients_cylinder->values[3]), coefficients_cylinder->values[1] + 0.083 * (-coefficients_cylinder->values[4]), coefficients_cylinder->values[2] + 0.083 * (-coefficients_cylinder->values[5]));
//	Eigen::Matrix3d R;
//	Eigen::Matrix4d Rotate;
//	R = Eigen::Quaterniond::FromTwoVectors(Z, axis).toRotationMatrix();
//	std::cout << R << std::endl;
//	Rotate.setIdentity();
//	//��ת
//	Rotate.block<3, 3>(0, 0) = R;
//	Rotate.block<3, 1>(0, 3) = T;
//	std::cout << Rotate << std::endl;
//	pcl::transformPointCloud(*modelCloud, *cloud, Rotate);
//
//	if (valization) {
//		//--------------------------------------���ӻ�--------------------------
//		pcl::visualization::PCLVisualizer viewer;
//		//�����ĵ��ƺ�ֱ��addcylinder����������Բ������Ƭ���бȶ�
//		viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud1");
//		//viewer.addCylinder(*coefficients_cylinder, "cylinder");
//		viewer.addCoordinateSystem();
//		while (!viewer.wasStopped())
//		{
//			viewer.spinOnce(100);
//		}
//	}
//}
//
//int main()
//{
//	pcl::ModelCoefficients::Ptr cylinder(new pcl::ModelCoefficients);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	cylinder->values.resize(7);
//	//������ò���
//	cylinder->values[0] = 0.130561;//x
//	cylinder->values[1] = -0.0304771;//y
//	cylinder->values[2] = 0.900897;//z
//	cylinder->values[3] = -0.156569;//w
//	cylinder->values[4] = 0.505592;//p
//	cylinder->values[5] = -0.848448;//r
//	cylinder->values[6] = 0.0731894;//R
//	//CreatCylinder(cylinder, cloud);
//
//	for (double x : {-0.210 / 2, 0.210 / 2}) {
//		for (double y : {-0.04, 0.04}) {
//			for (double z : {-0.04, 0.04}) {
//				pcl::PointXYZ point;
//				point.x = x;
//				point.y = y;
//				point.z = z;
//				cloud->points.push_back(point);
//			}
//		}
//	}
//	cloud->width = (int)cloud->size();
//	cloud->height = 1;
//	//����д�����
//	pcl::io::savePCDFile("cloud.pcd", *cloud);
//	return 0;
//}

#include <iostream>
#include <vector>
// ����cmath����ʹ����ѧ����
#include <cmath>

std::vector<std::vector<double>>  toRotationMatrix(double w, double x, double y, double z)
{
    std::vector<std::vector<double>> matrix = std::vector<std::vector<double>>(3, std::vector<double>(3, 0));
    // ������ת�����Ԫ��
    matrix[0][0] = 1 - 2 * y * y - 2 * z * z;
    matrix[0][1] = 2 * x * y - 2 * z * w;
    matrix[0][2] = 2 * x * z + 2 * y * w;

    matrix[1][0] = 2 * x * y + 2 * z * w;
    matrix[1][1] = 1 - 2 * x * x - 2 * z * z;
    matrix[1][2] = 2 * y * z - 2 * x * w;

    matrix[2][0] = 2 * x * z - 2 * y * w;
    matrix[2][1] = 2 * y * z + 2 * x * w;
    matrix[2][2] = 1 - 2 * x * x - 2 * y * y;

    return matrix;
};

int main() {

    std::vector<std::vector<double>> R;
    R = toRotationMatrix(-0.013037768380911025,
        -0.6996869005475356,
        0.7141071933285246,
        0.017865448057235056);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            std::cout << R[i][j] << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
