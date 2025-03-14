#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv) {
    // ��Ҫ�����ж���
    pcl::PCDReader reader;                          // PCD�ļ���ȡ����
    pcl::PassThrough<PointT> pass;                  // ֱͨ�˲���
    pcl::NormalEstimation<PointT, pcl::Normal> ne;  // ���߹������
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;   // �ָ���
    pcl::PCDWriter writer;                                      // PCD�ļ�д������
    pcl::ExtractIndices<PointT> extract;                        // ����ȡ����
    pcl::ExtractIndices<pcl::Normal> extract_normals;           // ������ȡ����
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    // ����
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

    // ��ȡ��������
    reader.read("G:/vsdata/PCLlearn/PCDdata/table_scene_mug_stereo_textured.pcd", *cloud);
    std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

    // ����һ��ֱͨ��������ɾ����ٵ�NaNs
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1.5);
    pass.filter(*cloud_filtered);
    std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

    // ���Ƶ㷨��
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // Ϊƽ��ģ�ʹ����ָ�����������в���
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);
    // ���ƽ���ڵ��ϵ��
    seg.segment(*inliers_plane, *coefficients_plane);
    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // ������������ȡƽ������
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);

    // ��ƽ����Ʊ���
    pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_plane);
    std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points."
        << std::endl;
    writer.write("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

    // �Ƴ�ƽ��inliers, ��ȡ���ಿ��
    extract.setNegative(true);
    extract.filter(*cloud_filtered2);
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals2);

    // ����Բ����ָ�������
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);   // ���÷ָ�ģ��ΪԲ����
    seg.setMethodType(pcl::SAC_RANSAC);         // ���ò���RANSAC�㷨���в�������
    seg.setNormalDistanceWeight(0.1);           // ���ñ��淨��Ȩ��ϵ��
    seg.setMaxIterations(10000);                // ��������������10000
    seg.setDistanceThreshold(0.05);             // �����ڵ㵽ģ�͵������� 0.05m
    seg.setRadiusLimits(0, 0.1);                // ����Բ����İ뾶��Χ0 -> 0.1m
    seg.setInputCloud(cloud_filtered2);
    seg.setInputNormals(cloud_normals2);

    // ��ȡcylinder inliers
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // ��cylinder inliers����
    extract.setInputCloud(cloud_filtered2);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_cylinder);
    if (cloud_cylinder->points.empty())
        std::cerr << "Can't find the cylindrical component." << std::endl;
    else {
        std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size()
            << " data points." << std::endl;
        writer.write("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
    }
    return (0);
}