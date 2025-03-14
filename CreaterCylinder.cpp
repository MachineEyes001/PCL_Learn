#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>

//����Բ������������ռ�����Բ�������
//param[in]  pcl::ModelCoefficients::Ptr& coefficients_cylinder://Բ���������ϵ��0��1��2����Բ�������ϵ�ԭ�㣬3��4��5�����������ߵķ���������ϵ��6����Բ���İ뾶��
//param[in] valization��Ĭ�Ͽ��ӻ�
//param[out] cloud��PCD��ʽ�ĵ����ļ�
void CreatCylinder(pcl::ModelCoefficients::Ptr& coefficients_cylinder, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, bool valization = true) {
	//������
	if (coefficients_cylinder->values.size() != 7) {
		std::cerr << "������Ŀ���󡣡���" << std::endl;
		system("pause");
	}
	if (coefficients_cylinder->values[6] <= 0) {
		std::cerr << "Բ����뾶����С�ڵ���0" << std::endl;
		system("pause");
	}
	//�ȹ�������ΪZ���Բ�������
	int Num = 100;
	float inter = 2.0 * M_PI / Num;
	Eigen::RowVectorXd vectorx(Num), vectory(Num), vectorz(Num);
	Eigen::RowVector3d axis(-coefficients_cylinder->values[3], -coefficients_cylinder->values[4], -coefficients_cylinder->values[5]);
	float length = axis.norm();
	vectorx.setLinSpaced(Num, 0, Num - 1);
	vectory = vectorx;
	float r0 = coefficients_cylinder->values[6];

	pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder(new pcl::PointCloud<pcl::PointXYZ>);

	for (float z(0); z <=0.164; z += 0.005)
	{
		for (auto i = 0; i < Num; ++i) {
			pcl::PointXYZ point;
			point.x = r0 * cos(vectorx[i] * inter);
			point.y = r0 * sin(vectory[i] * inter);
			point.z = z;
			cylinder->points.push_back(point);
		}
	}

	cylinder->width = (int)cylinder->size();
	cylinder->height = 1;
	cylinder->is_dense = false;
	pcl::io::savePCDFile("cylinderbeforeR.pcd", *cylinder);

	//������ת Z��ת��axis
	Eigen::RowVector3d  Z(0.0, 0.0, 0.1), T(coefficients_cylinder->values[0], coefficients_cylinder->values[1], coefficients_cylinder->values[2]);
	Eigen::Matrix3d R;
	Eigen::Matrix3d E = Eigen::MatrixXd::Identity(3, 3);
	Eigen::Matrix4d Rotate, Translation;
	R = Eigen::Quaterniond::FromTwoVectors(Z, axis).toRotationMatrix();
	Rotate.setIdentity();
	Translation.setIdentity();

	//��ת
	Rotate.block<3, 3>(0, 0) = R;
	Rotate.block<3, 1>(0, 3) = T;
	pcl::transformPointCloud(*cylinder, *cloud, Rotate);


	if (valization) {
		//--------------------------------------���ӻ�--------------------------
		pcl::visualization::PCLVisualizer viewer;
		//�����ĵ��ƺ�ֱ��addcylinder����������Բ������Ƭ���бȶ�
		viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud1");
		//viewer.addCylinder(*coefficients_cylinder, "cylinder");
		viewer.addCoordinateSystem();
		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
		}
	}
}

int main()
{
	pcl::ModelCoefficients::Ptr cylinder(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cylinder->values.resize(7);
	//������ò���
	cylinder->values[0] = 0.130561;//x
	cylinder->values[1] = -0.0304771;//y
	cylinder->values[2] = 0.900897;//z
	cylinder->values[3] = -0.156569;//w
	cylinder->values[4] = 0.505592;//p
	cylinder->values[5] = -0.848448;//r
	cylinder->values[6] = 0.0731894;//R
	CreatCylinder(cylinder, cloud);
	//����д�����
	pcl::io::savePCDFile("cylinder.pcd", *cloud);
	return 0;
}

