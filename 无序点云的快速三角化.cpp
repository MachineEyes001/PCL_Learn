#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>      //̰��ͶӰ���ǻ��㷨
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

int main(int argc, char** argv)
{
	// ��һ��XYZ�����͵�PCD�ļ��򿪲��洢��������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile("G:/vsdata/PCLlearn/PCDdata/bun0.pcd", cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);

	// ��̬����
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;   //���߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);   //�洢���Ƶķ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);  //����kd��ָ��
	tree->setInputCloud(cloud);   //��cloud����tree����
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);   //���Ʒ��ߴ洢������
	//* ���߲�Ӧ�����㷨��+��������

	// ����XYZ�ֶκͷ����ֶ�*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);    //�����ֶ�
	//* cloud_with_normals = cloud + normals

	// ��������������
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);   //���ƹ���������

	// ��ʼ������
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   //�������ǻ�����
	pcl::PolygonMesh triangles;                //�洢�������ǻ�������ģ��

	// �������ӵ�֮��������루���߳���
	gp3.setSearchRadius(0.025);  //�������ӵ�֮��������룬���������������߳���

	// ���ø�����ֵ
	gp3.setMu(2.5);  //���ñ���������������ڵ����Զ����Ϊ2.5��Ϊ��ʹ�õ����ܶȵı仯
	gp3.setMaximumNearestNeighbors(100);    //������������������������
	gp3.setMaximumSurfaceAngle(M_PI / 4); // ����ĳ�㷨�߷���ƫ�������㷨�ߵ����Ƕ�45
	gp3.setMinimumAngle(M_PI / 18); // �������ǻ���õ����������ڽǵ���С�ĽǶ�Ϊ10
	gp3.setMaximumAngle(2 * M_PI / 3); // �������ǻ���õ����������ڽǵ����Ƕ�Ϊ120
	gp3.setNormalConsistency(false);  //���øò�����֤���߳���һ��

	// ��ȡ���
	gp3.setInputCloud(cloud_with_normals);     //�����������Ϊ�������
	gp3.setSearchMethod(tree2);   //����������ʽ
	gp3.reconstruct(triangles);  //�ؽ���ȡ���ǻ�

	// ���Ӷ�����Ϣ
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	// ���ӻ�
	// �������ӻ�����
	pcl::visualization::PCLVisualizer viewer("PolygonMesh Viewer");

	// ������񵽿��ӻ�����
	viewer.addPolygonMesh(triangles, "mesh");

	// ���ù۲��ͷ���
	viewer.setCameraPosition(0, 0, -2, 0, -1, 0);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}
	return (0);
}