#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>

//根据圆柱面参数创建空间任意圆柱面点云
//param[in]  pcl::ModelCoefficients::Ptr& coefficients_cylinder://圆柱面参数：系数0、1、2代表圆柱轴线上的原点，3、4、5代表这条轴线的方向向量，系数6就是圆柱的半径。
//param[in] valization：默认可视化
//param[out] cloud：PCD格式的点云文件
void CreatCylinder(pcl::ModelCoefficients::Ptr& coefficients_cylinder, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, bool valization = true) {
	//检查参数
	if (coefficients_cylinder->values.size() != 7) {
		std::cerr << "参数数目错误。。。" << std::endl;
		system("pause");
	}
	if (coefficients_cylinder->values[6] <= 0) {
		std::cerr << "圆柱面半径不能小于等于0" << std::endl;
		system("pause");
	}
	//先构建轴线为Z轴的圆柱面点云
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

	//点云旋转 Z轴转到axis
	Eigen::RowVector3d  Z(0.0, 0.0, 0.1), T(coefficients_cylinder->values[0], coefficients_cylinder->values[1], coefficients_cylinder->values[2]);
	Eigen::Matrix3d R;
	Eigen::Matrix3d E = Eigen::MatrixXd::Identity(3, 3);
	Eigen::Matrix4d Rotate, Translation;
	R = Eigen::Quaterniond::FromTwoVectors(Z, axis).toRotationMatrix();
	Rotate.setIdentity();
	Translation.setIdentity();

	//旋转
	Rotate.block<3, 3>(0, 0) = R;
	Rotate.block<3, 1>(0, 3) = T;
	pcl::transformPointCloud(*cylinder, *cloud, Rotate);


	if (valization) {
		//--------------------------------------可视化--------------------------
		pcl::visualization::PCLVisualizer viewer;
		//创建的点云和直接addcylinder函数创建的圆柱面面片进行比对
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
	//随便设置参数
	cylinder->values[0] = 0.130561;//x
	cylinder->values[1] = -0.0304771;//y
	cylinder->values[2] = 0.900897;//z
	cylinder->values[3] = -0.156569;//w
	cylinder->values[4] = 0.505592;//p
	cylinder->values[5] = -0.848448;//r
	cylinder->values[6] = 0.0731894;//R
	CreatCylinder(cylinder, cloud);
	//点云写入磁盘
	pcl::io::savePCDFile("cylinder.pcd", *cloud);
	return 0;
}

