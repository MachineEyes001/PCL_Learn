#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/impl/search.hpp>

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> PCLHandler;

class FeatureCloud {
public:
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud() :search_method_xyz_(new SearchMethod),normal_radius_(0.02f),feature_radius_(0.02f) {}

    ~FeatureCloud() {}

    // ��������ĵ���
    void
        setInputCloud(PointCloud::Ptr xyz) {
        xyz_ = xyz;
        processInput();
    }

    // ���ز��������PCD�ļ��еĵ���
    void
        loadInputCloud(const std::string& pcd_file) {
        xyz_ = PointCloud::Ptr(new PointCloud);
        pcl::io::loadPCDFile(pcd_file, *xyz_);
        processInput();
    }

    // ��ȡָ����Ƶ�ָ��
    PointCloud::Ptr
        getPointCloud() const {
        return (xyz_);
    }

    // ��ȡָ����ά���淨�ߵĵ��Ƶ�ָ��
    SurfaceNormals::Ptr
        getSurfaceNormals() const {
        return (normals_);
    }

    // ��ȡָ�������������Ƶ�ָ��
    LocalFeatures::Ptr
        getLocalFeatures() const {
        return (features_);
    }

protected:
    // ������淨�ߺ;ֲ�����
    void
        processInput() {
        computeSurfaceNormals();
        computeLocalFeatures();
    }

    // ������淨��
    void
        computeSurfaceNormals() {
        // �������淨����
        normals_ = SurfaceNormals::Ptr(new SurfaceNormals);

        // ������淨����
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
        norm_est.setInputCloud(xyz_);
        norm_est.setSearchMethod(search_method_xyz_);
        norm_est.setRadiusSearch(normal_radius_);
        norm_est.compute(*normals_);
    }

    // ����ֲ�����������
    /**
     * ���ݱ��淨���� ���㱾����������
     */
    void
        computeLocalFeatures() {
        features_ = LocalFeatures::Ptr(new LocalFeatures);

        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
        fpfh_est.setInputCloud(xyz_);
        fpfh_est.setInputNormals(normals_);
        fpfh_est.setSearchMethod(search_method_xyz_);
        fpfh_est.setRadiusSearch(feature_radius_);
        fpfh_est.compute(*features_);
    }

private:
    // ��������
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_; // ���ٵ�����ֱ��ͼ Fast Point Feature Histogram
    SearchMethod::Ptr search_method_xyz_; // KDTree������������

    // ����
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment {
public:

    // ���ڴ洢�������Ľṹ
    struct Result {
        // ƥ�����
        float fitness_score;
        // ת������
        Eigen::Matrix4f final_transformation;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment() :
        min_sample_distance_(0.05f),
        max_correspondence_distance_(0.01f * 0.01f),
        nr_iterations_(500) {
        // ��ʼ���������һ���ԣ�SAC-IA���㷨�еĲ���
        sac_ia_.setMinSampleDistance(min_sample_distance_);
        sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
        sac_ia_.setMaximumIterations(nr_iterations_);
    }

    ~TemplateAlignment() {}

    // �������ĵ�������Ϊģ�彫��֮�����Ŀ��
    void setTargetCloud(FeatureCloud& target_cloud) {
        target_ = target_cloud;
        // ��������target����
        sac_ia_.setInputTarget(target_cloud.getPointCloud());
        // ��������target
        sac_ia_.setTargetFeatures(target_cloud.getLocalFeatures());
    }

    // ������������ӵ�ģ������б���
    void addTemplateCloud(FeatureCloud& template_cloud) {
        templates_.push_back(template_cloud);
    }

    // ��������ģ�������setTargetCloud����ָ����Ŀ�����
    // ����ĺ��Ĵ���
    void align(FeatureCloud& template_cloud, TemplateAlignment::Result& result) {
        // ��������Դ
        sac_ia_.setInputSource(template_cloud.getPointCloud());
        // ��������Դ
        sac_ia_.setSourceFeatures(template_cloud.getLocalFeatures());

        pcl::PointCloud<pcl::PointXYZ> registration_output;
        sac_ia_.align(registration_output);

        // ������Զ��Ӧ�������ƥ�����
        result.fitness_score = (float)sac_ia_.getFitnessScore(max_correspondence_distance_);
        // ��ȡ����ת������
        result.final_transformation = sac_ia_.getFinalTransformation();
    }

    // ��addTemplateCloud���õ�����ģ�������setTargetCloud����ָ����Ŀ�����
    void alignAll(std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> >& results) {
        results.resize(templates_.size());
        for (size_t i = 0; i < templates_.size(); ++i) {
            align(templates_[i], results[i]);
        }
    }

    // ������ģ�������Ŀ����ƶ��룬���ҵ�������Ѷ��������ģ����
    int findBestAlignment(TemplateAlignment::Result& result) {
        // ������ģ����Ŀ����ƶ���
        std::vector<Result, Eigen::aligned_allocator<Result> > results;
        alignAll(results);

        // �ҵ�����ʵ�ģ��
        float lowest_score = std::numeric_limits<float>::infinity();
        int best_template = 0;
        for (size_t i = 0; i < results.size(); ++i) {
            const Result& r = results[i];
            if (r.fitness_score < lowest_score) {
                lowest_score = r.fitness_score;
                best_template = (int)i;
            }
        }

        // ������ƥ��
        result = results[best_template];
        return (best_template);
    }

private:
    // ģ���Ƽ��佫Ҫ�����Ŀ����б�
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // �������һ���ԣ�SAC-IA���㷨ע�����̼������
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};

/**
 * �������ģ�弯�ϵ�һ��ʾ������
 * ͨ��ֱͨ�˲��򶨷�Χ���õ�����Ȥ����
 * ������Ȥ������н����������ģ��ƥ��Ч�ʣ�
 */
int main(int argc, char** argv) 
{
    // ���ض���ģ��
    std::vector<FeatureCloud> object_templates;
    object_templates.resize(0);

    // ����������
    FeatureCloud template_cloud;
    template_cloud.loadInputCloud("G:/vsdata/PCLlearn/PCDdata/bun0.pcd");
    object_templates.push_back(template_cloud);

    // ����Ŀ�����
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read("G:/vsdata/PCLlearn/PCDdata/bun0.pcd", *cloud);

    // �Ƴ���Ⱥ��
    const float depth_limit = 1.0;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, depth_limit);
    pass.filter(*cloud);

    // ����������, ���ټ�����
    // �������ش�С 5mm
    const float voxel_grid_size = 0.005f;
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setInputCloud(cloud);
    // ����Ҷ�ӽڵ�Ĵ�Сlx, ly, lz
    vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
    vox_grid.filter(*tempCloud);
    cloud = tempCloud;

    // �����˲�&��������ĵ���ͼ
    pcl::io::savePCDFileBinary("pass_through_voxel.pcd", *tempCloud);
    std::cout << "pass_through_voxel.pcd saved" << std::endl;

    // ���뵽Ŀ����������
    FeatureCloud target_cloud;
    target_cloud.setInputCloud(cloud);

    // ����TemplateAlignment����
    TemplateAlignment template_align;
    for (size_t i = 0; i < object_templates.size(); i++) {
        FeatureCloud& object_template = object_templates[i];
        // ���ģ�����
        template_align.addTemplateCloud(object_template);
    }
    // ����Ŀ�����
    template_align.setTargetCloud(target_cloud);
    std::cout << "findBestAlignment" << std::endl;
    // �������ģ����뷽ʽ
    // ���Ĵ���
    TemplateAlignment::Result best_alignment;
    int best_index = template_align.findBestAlignment(best_alignment);
    const FeatureCloud& best_template = object_templates[best_index];

    // ��ӡƥ��ȷ�����С��0.00002��ֵ�Ǻõģ�
    printf("Best fitness score: %f\n", best_alignment.fitness_score);
    printf("Best fitness best_index: %d\n", best_index);

    // ��ӡ��ת�����ƽ������
    Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
    Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);
    Eigen::Vector3f euler_angles = rotation.eulerAngles(2, 1, 0) * 180 / M_PI;
    printf("\n");
    printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
    printf("\n");
    cout << "yaw(z) pitch(y) roll(x) = " << euler_angles.transpose() << endl;
    printf("\n");
    printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

    // ��������ģ���Խ��п��ӻ�
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    // ��ģ���б���ĵ���ͼ������ת����任���ѱ任������浽transformed_cloud
    pcl::transformPointCloud(*best_template.getPointCloud(), transformed_cloud, best_alignment.final_transformation);
    //pcl::io::savePCDFileBinary("output.pcd", transformed_cloud);

    pcl::visualization::PCLVisualizer viewer("example");
    // ��������ϵϵͳ
    viewer.addCoordinateSystem(0.5, "cloud", 0);
    // ���ñ���ɫ
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

    // ��ת��ĵ���rotated --------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud(&transformed_cloud);
    PCLHandler transformed_cloud_handler(t_cloud, 255, 255, 255);
    viewer.addPointCloud(t_cloud, transformed_cloud_handler, "transformed_cloud");
    // ������Ⱦ���ԣ����С��
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");

    // Ŀ�����target --------------------------------
    PCLHandler target_cloud_handler(cloud, 255, 100, 100);
    viewer.addPointCloud(cloud, target_cloud_handler, "target_cloud");
    // ������Ⱦ���ԣ����С��
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");

    // ģ�����template --------------------------------
    PCLHandler template_cloud_handler(cloud, 100, 255, 255);
    viewer.addPointCloud(best_template.getPointCloud(), template_cloud_handler, "template_cloud");
    // ������Ⱦ���ԣ����С��
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "template_cloud");

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    return (0);
}