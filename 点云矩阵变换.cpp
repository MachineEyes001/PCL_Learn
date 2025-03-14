//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/common/transforms.h>
//#include <pcl/ModelCoefficients.h>
//
////根据圆柱面参数创建空间任意圆柱面点云
////param[in]  pcl::ModelCoefficients::Ptr& coefficients_cylinder://圆柱面参数：系数0、1、2代表圆柱轴线上的原点，3、4、5代表这条轴线的方向向量，系数6就是圆柱的半径。
////param[in] valization：默认可视化
////param[out] cloud：PCD格式的点云文件
//void CreatCylinder(pcl::ModelCoefficients::Ptr& coefficients_cylinder, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, bool valization = true) {
//	//检查参数
//	if (coefficients_cylinder->values.size() != 7) {
//		std::cerr << "参数数目错误。。。" << std::endl;
//		system("pause");
//	}
//	if (coefficients_cylinder->values[6] <= 0) {
//		std::cerr << "圆柱面半径不能小于等于0" << std::endl;
//		system("pause");
//	}
//	
//	Eigen::RowVector3d axis(-coefficients_cylinder->values[3], -coefficients_cylinder->values[4], -coefficients_cylinder->values[5]);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud(new pcl::PointCloud<pcl::PointXYZ>);
//	if (pcl::io::loadPCDFile<pcl::PointXYZ>("F:/Paper/PCLvision/model/cheesecloudmodel.pcd", *modelCloud) == -1) //* 加载文件
//	{
//		return;
//	}//调试
//
//	//平移 针对立式筒纱
//	//for (size_t i = 0; i < modelCloud->size(); i++) {
//
//	//	modelCloud->points[i].z = modelCloud->points[i].z + 0.083;
//	//}
//
//	//点云旋转 Z轴转到axis
//	Eigen::RowVector3d  Z(0.0, 0.0, 0.1), T(coefficients_cylinder->values[0] + 0.083*(-coefficients_cylinder->values[3]), coefficients_cylinder->values[1] + 0.083 * (-coefficients_cylinder->values[4]), coefficients_cylinder->values[2] + 0.083 * (-coefficients_cylinder->values[5]));
//	Eigen::Matrix3d R;
//	Eigen::Matrix4d Rotate;
//	R = Eigen::Quaterniond::FromTwoVectors(Z, axis).toRotationMatrix();
//	std::cout << R << std::endl;
//	Rotate.setIdentity();
//	//旋转
//	Rotate.block<3, 3>(0, 0) = R;
//	Rotate.block<3, 1>(0, 3) = T;
//	std::cout << Rotate << std::endl;
//	pcl::transformPointCloud(*modelCloud, *cloud, Rotate);
//
//	if (valization) {
//		//--------------------------------------可视化--------------------------
//		pcl::visualization::PCLVisualizer viewer;
//		//创建的点云和直接addcylinder函数创建的圆柱面面片进行比对
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
//	//随便设置参数
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
//	//点云写入磁盘
//	pcl::io::savePCDFile("cloud.pcd", *cloud);
//	return 0;
//}

#include <iostream>
#include <vector>
// 引入cmath库来使用数学函数
#include <cmath>

std::vector<std::vector<double>>  toRotationMatrix(double w, double x, double y, double z)
{
    std::vector<std::vector<double>> matrix = std::vector<std::vector<double>>(3, std::vector<double>(3, 0));
    // 计算旋转矩阵的元素
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
