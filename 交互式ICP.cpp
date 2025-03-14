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
 * 此函数是查看器的回调。当查看器窗口位于顶部时，只要按任意键，就会调用此函数。如果碰到“空格”，将布尔值设置为true。
 */
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,void* nothing)
{
    if (event.getKeySym() == "space" && event.keyDown())
        next_iteration = true;
}

// 转换点云格式
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
    // 准备将使用的点云
    PointCloudT::Ptr cloud_in(new PointCloudT);  // Original point cloud
    PointCloudT::Ptr cloud_tr(new PointCloudT);  // Transformed point cloud
    PointCloudT::Ptr cloud_icp(new PointCloudT);  // ICP output point cloud

    //// 检查程序的参数，设置初始ICP迭代的次数，然后尝试加载PLY文件
    //if (argc < 2)
    //{
    //    printf("Usage :\n");
    //    printf("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
    //    PCL_ERROR("Provide one ply file.\n");
    //    return (-1);
    //}

    int iterations = 1;  // 默认的ICP迭代次数
    //if (argc > 2)
    //{
    //    // 如果用户将迭代次数作为参数传递
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

    //将texturedPointCloudPCL的数据复制到currentPointCloud中;
    //convertToPCL(*PointCloud, *cloud_in);
    //pcl::PCDWriter writer;
    //writer.write("F:/Paper/test/liecheesePCL.pcd", *cloud_in, true);


    // 我们使用刚性矩阵变换来变换原始点云。
    // cloud_in包含原始点云。
    // cloud_tr和cloud_icp包含平移/旋转的点云。
    // cloud_tr是我们将用于显示的备份（绿点云）。

    // 定义旋转矩阵和平移向量
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

    // 一个旋转矩阵 (see https://en.wikipedia.org/wiki/Rotation_matrix)
    double theta = M_PI / 8;  // 以弧度为单位的旋转角度
    transformation_matrix(0, 0) = std::cos(theta);
    transformation_matrix(0, 1) = -sin(theta);
    transformation_matrix(1, 0) = sin(theta);
    transformation_matrix(1, 1) = std::cos(theta);

    // Z轴上的平移（0.4米）
    transformation_matrix(2, 3) = 0.4;

    // 在终端中显示变换矩阵
    std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
    print4x4Matrix(transformation_matrix);

    // 执行转换
    //pcl::transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);
    if (pcl::io::loadPCDFile("G:/vsdata/PCLlearn/cylinder.pcd", *cloud_icp) < 0)
    {
        PCL_ERROR("Error loading cloud %s.\n", "F:/Paper/test/liecheese.pcd");
        return (-1);
    }
    //将texturedPointCloudPCL的数据复制到currentPointCloud中;
    //convertToPCL(*PointCloud, *cloud_icp);
    *cloud_tr = *cloud_icp;  // 我们将cloud_icp备份到cloud_tr中以备将来使用

    // 这是ICP对象的创建。我们设置ICP算法的参数。
    // setMaximumIterations（iterations）设置要执行的初始迭代次数（默认值为1）。
    // 然后，我们将点云转换为cloud_icp。 第一次对齐后，我们将在下一次使用该ICP对象时（当用户按下“空格”时）将ICP最大迭代次数设置为1。

    // ICP算法
    time.tic();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(iterations);
    icp.setInputSource(cloud_icp);
    icp.setInputTarget(cloud_in);
    icp.align(*cloud_icp);
    icp.setMaximumIterations(1);  // 下次调用.align（）函数时，我们将此变量设置为1
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

    // 检查ICP算法是否收敛；否则退出程序。如果返回true，我们将转换矩阵存储在4x4矩阵中，然后打印刚性矩阵转换。
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

    // 可视化
    pcl::visualization::PCLVisualizer viewer("ICP demo");
    // 创建两个垂直分隔的窗口
    int v1(0);
    int v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    // 颜色
    float bckgr_gray_level = 0.0;  // 黑色
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // 原始点云为白色
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
        (int)255 * txt_gray_lvl);
    viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

    // 变换后的点云为绿色
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
    viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

    // ICP对齐点云为红色
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_icp, 180, 20, 20);
    viewer.addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

    // 在每个视口中添加文本描述
    viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    std::stringstream ss;
    ss << iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str();
    viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

    // 设置背景颜色
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // 设置相机位置和方向
    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize(1280, 1024);  // 可视化窗口的尺寸

    // 注册键盘回调：
    viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);

    //可视化
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();

        // 如果用户按下空格:
        if (next_iteration)
        {
            // ICP算法
            time.tic();
            // 如果用户按下键盘上的任意键，则会调用keyboardEventOccurred函数。 此功能检查键是否为“空格”。
            // 如果是，则全局布尔值next_iteration设置为true，从而允许查看器循环输入代码的下一部分：调用ICP对象以进行对齐。
            // 记住，我们已经配置了该对象输入/输出云，并且之前通过setMaximumIterations将最大迭代次数设置为1。

            icp.align(*cloud_icp);
            std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;

            // 和以前一样，我们检查ICP是否收敛，如果不收敛，则退出程序。
            if (icp.hasConverged())
            {
                // printf（“ 033 [11A”）; 在终端增加11行以覆盖显示的最后一个矩阵是一个小技巧。
                // 简而言之，它允许替换文本而不是编写新行； 使输出更具可读性。 我们增加迭代次数以更新可视化器中的文本值。
                printf("\033[11A");  // Go up 11 lines in terminal output.
                printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());

                // 这意味着，如果您已经完成了10次迭代，则此函数返回矩阵以将点云从迭代10转换为11。
                std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;

                // 函数getFinalTransformation（）返回在迭代过程中完成的刚性矩阵转换（此处为1次迭代）。
                transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
                print4x4Matrix(transformation_matrix);  // 打印原始姿态和当前姿态之间的转换

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
            //这不是我们想要的。如果我们将最后一个矩阵与新矩阵相乘，那么结果就是从开始到当前迭代的转换矩阵。
        }
        next_iteration = false;
    }
    return (0);
}