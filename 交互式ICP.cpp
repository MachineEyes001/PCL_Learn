#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

void print4x4Matrix(const Eigen::Matrix4d& matrix)
{
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}
/**
 * �˺����ǲ鿴���Ļص������鿴������λ�ڶ���ʱ��ֻҪ����������ͻ���ô˺���������������ո񡱣�������ֵ����Ϊtrue��
 */
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,void* nothing)
{
    if (event.getKeySym() == "space" && event.keyDown())
        next_iteration = true;
}

// ת�����Ƹ�ʽ
void convertToPCL(pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& pclPointCloud)
{
    // write PointXYZ data
    uint32_t size = cloud.size();
    pclPointCloud.resize(size);

    for (size_t i = 0; i < size; i++) {
        pclPointCloud[i].x = 0.001 * cloud[i].x; // mm to m
        pclPointCloud[i].y = 0.001 * cloud[i].y; // mm to m
        pclPointCloud[i].z = 0.001 * cloud[i].z; // mm to m
        //pclPointCloud[i].x = cloud[i].x; // mm to m
        //pclPointCloud[i].y = cloud[i].y; // mm to m
        //pclPointCloud[i].z = cloud[i].z; // mm to m
    }

    return;
}


int main(int argc,char* argv[])
{
    // ׼����ʹ�õĵ���
    PointCloudT::Ptr cloud_in(new PointCloudT);  // Original point cloud
    PointCloudT::Ptr cloud_tr(new PointCloudT);  // Transformed point cloud
    PointCloudT::Ptr cloud_icp(new PointCloudT);  // ICP output point cloud

    //// ������Ĳ��������ó�ʼICP�����Ĵ�����Ȼ���Լ���PLY�ļ�
    //if (argc < 2)
    //{
    //    printf("Usage :\n");
    //    printf("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
    //    PCL_ERROR("Provide one ply file.\n");
    //    return (-1);
    //}

    int iterations = 1;  // Ĭ�ϵ�ICP��������
    //if (argc > 2)
    //{
    //    // ����û�������������Ϊ��������
    //    iterations = atoi(argv[2]);
    //    if (iterations < 1)
    //    {
    //        PCL_ERROR("Number of initial iterations must be >= 1\n");
    //        return (-1);
    //    }
    //}

    pcl::console::TicToc time;
    time.tic();
    /*if (pcl::io::loadPLYFile(argv[1], *cloud_in) < 0)
    {
        PCL_ERROR("Error loading cloud %s.\n", argv[1]);
        return (-1);
    }
    std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in->size() << " points) in " << time.toc() << " ms\n" << std::endl;*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("F:/Paper/test/12-29-38-cloud-stand-ECE-GR.pcd", *cloud_in) < 0)
    {
        PCL_ERROR("Error loading cloud %s.\n", "F:/Paper/PCLvision/model/cheesemodel.pcd");
        return (-1);
    }
    std::cout << "\nLoaded file " << "F:/Paper/PCLvision/model/cheesemodel.pcd" << " (" << cloud_in->size() << " points) in " << time.toc() << " ms\n" << std::endl;

    //��texturedPointCloudPCL�����ݸ��Ƶ�currentPointCloud��;
    //convertToPCL(*PointCloud, *cloud_in);
    //pcl::PCDWriter writer;
    //writer.write("F:/Paper/test/liecheesePCL.pcd", *cloud_in, true);


    // ����ʹ�ø��Ծ���任���任ԭʼ���ơ�
    // cloud_in����ԭʼ���ơ�
    // cloud_tr��cloud_icp����ƽ��/��ת�ĵ��ơ�
    // cloud_tr�����ǽ�������ʾ�ı��ݣ��̵��ƣ���

    // ������ת�����ƽ������
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

    // һ����ת���� (see https://en.wikipedia.org/wiki/Rotation_matrix)
    double theta = M_PI / 8;  // �Ի���Ϊ��λ����ת�Ƕ�
    transformation_matrix(0, 0) = std::cos(theta);
    transformation_matrix(0, 1) = -sin(theta);
    transformation_matrix(1, 0) = sin(theta);
    transformation_matrix(1, 1) = std::cos(theta);

    // Z���ϵ�ƽ�ƣ�0.4�ף�
    transformation_matrix(2, 3) = 0.4;

    // ���ն�����ʾ�任����
    std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
    print4x4Matrix(transformation_matrix);

    // ִ��ת��
    //pcl::transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);
    if (pcl::io::loadPCDFile("G:/vsdata/PCLlearn/cylinder.pcd", *cloud_icp) < 0)
    {
        PCL_ERROR("Error loading cloud %s.\n", "F:/Paper/test/liecheese.pcd");
        return (-1);
    }
    //��texturedPointCloudPCL�����ݸ��Ƶ�currentPointCloud��;
    //convertToPCL(*PointCloud, *cloud_icp);
    *cloud_tr = *cloud_icp;  // ���ǽ�cloud_icp���ݵ�cloud_tr���Ա�����ʹ��

    // ����ICP����Ĵ�������������ICP�㷨�Ĳ�����
    // setMaximumIterations��iterations������Ҫִ�еĳ�ʼ����������Ĭ��ֵΪ1����
    // Ȼ�����ǽ�����ת��Ϊcloud_icp�� ��һ�ζ�������ǽ�����һ��ʹ�ø�ICP����ʱ�����û����¡��ո�ʱ����ICP��������������Ϊ1��

    // ICP�㷨
    time.tic();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(iterations);
    icp.setInputSource(cloud_icp);
    icp.setInputTarget(cloud_in);
    icp.align(*cloud_icp);
    icp.setMaximumIterations(1);  // �´ε���.align��������ʱ�����ǽ��˱�������Ϊ1
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

    // ���ICP�㷨�Ƿ������������˳������������true�����ǽ�ת������洢��4x4�����У�Ȼ���ӡ���Ծ���ת����
    if (icp.hasConverged())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix = icp.getFinalTransformation().cast<double>();
        print4x4Matrix(transformation_matrix);
    }
    else
    {
        PCL_ERROR("\nICP has not converged.\n");
        return (-1);
    }

    // ���ӻ�
    pcl::visualization::PCLVisualizer viewer("ICP demo");
    // ����������ֱ�ָ��Ĵ���
    int v1(0);
    int v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    // ��ɫ
    float bckgr_gray_level = 0.0;  // ��ɫ
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // ԭʼ����Ϊ��ɫ
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
        (int)255 * txt_gray_lvl);
    viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

    // �任��ĵ���Ϊ��ɫ
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
    viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

    // ICP�������Ϊ��ɫ
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_icp, 180, 20, 20);
    viewer.addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

    // ��ÿ���ӿ�������ı�����
    viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    std::stringstream ss;
    ss << iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str();
    viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

    // ���ñ�����ɫ
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // �������λ�úͷ���
    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize(1280, 1024);  // ���ӻ����ڵĳߴ�

    // ע����̻ص���
    viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);

    //���ӻ�
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();

        // ����û����¿ո�:
        if (next_iteration)
        {
            // ICP�㷨
            time.tic();
            // ����û����¼����ϵ��������������keyboardEventOccurred������ �˹��ܼ����Ƿ�Ϊ���ո񡱡�
            // ����ǣ���ȫ�ֲ���ֵnext_iteration����Ϊtrue���Ӷ�����鿴��ѭ������������һ���֣�����ICP�����Խ��ж��롣
            // ��ס�������Ѿ������˸ö�������/����ƣ�����֮ǰͨ��setMaximumIterations����������������Ϊ1��

            icp.align(*cloud_icp);
            std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;

            // ����ǰһ�������Ǽ��ICP�Ƿ���������������������˳�����
            if (icp.hasConverged())
            {
                // printf���� 033 [11A����; ���ն�����11���Ը�����ʾ�����һ��������һ��С���ɡ�
                // �����֮���������滻�ı������Ǳ�д���У� ʹ������߿ɶ��ԡ� �������ӵ��������Ը��¿��ӻ����е��ı�ֵ��
                printf("\033[11A");  // Go up 11 lines in terminal output.
                printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());

                // ����ζ�ţ�������Ѿ������10�ε�������˺������ؾ����Խ����ƴӵ���10ת��Ϊ11��
                std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;

                // ����getFinalTransformation���������ڵ�����������ɵĸ��Ծ���ת�����˴�Ϊ1�ε�������
                transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
                print4x4Matrix(transformation_matrix);  // ��ӡԭʼ��̬�͵�ǰ��̬֮���ת��

                ss.str("");
                ss << iterations;
                std::string iterations_cnt = "ICP iterations = " + ss.str();
                viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
                viewer.updatePointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
            }
            else
            {
                PCL_ERROR("\nICP has not converged.\n");
                return (-1);
            }
            //�ⲻ��������Ҫ�ġ�������ǽ����һ���������¾�����ˣ���ô������Ǵӿ�ʼ����ǰ������ת������
        }
        next_iteration = false;
    }
    return (0);
}