/*
 * main.cpp
 *
 *  Created on: Jul 29, 2021
 *      Author: zack
 */
#include "g2o_solver.h"
#include "ceres_solver.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include "GaussNewton.h"
using namespace trunk;
bool next_iteration = false;
double ndt_stepsize = 0.1;
double ndt_resolution = 0.5;
double ndt_outratio = 0.2;
void keyboardEventCallback (const pcl::visualization::KeyboardEvent& event, void* nothing)
{
    if (event.getKeySym () == "space" && event.keyDown ())
        next_iteration = true;
}



int main (int argc, char* argv[])
{
    // The point clouds we will be using
    PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
    PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
    PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
    PointCloudT::Ptr cloud_ndt (new PointCloudT);  // ICP output point cloud
    PointCloudT::Ptr cloud_pre_ndt (new PointCloudT);  // ICP output point cloud
    PointCloudT::Ptr cloud_g2o (new PointCloudT);  // ICP output point cloud
    PointCloudT::Ptr cloud_gaussnewton(new PointCloudT);

    // Checking program arguments
    if (argc < 2)
    {
        printf ("Usage :\n");
        printf ("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
        PCL_ERROR ("Provide one ply file.\n");
        return (-1);
    }

    int iterations = 1;  // Default number of ICP iterations
    if (argc > 2)
    {
        // If the user passed the number of iteration as an argument
        iterations = atoi (argv[2]);
        if (iterations < 1)
        {
            PCL_ERROR ("Number of initial iterations must be >= 1\n");
            return (-1);
        }
    }

    pcl::console::TicToc time;
    time.tic();
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(argv[1], *cloud_in) < 0)
    {
        PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
        return (-1);
    }
    std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in->size () << " points) in " << time.toc() << " ms\n" << std::endl;

    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d gt_pose = Eigen::Matrix4d::Identity ();

    // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    double theta = M_PI_4 / 16;  // The angle of rotation in radians
    gt_pose (0, 0) = std::cos (theta);
    gt_pose (0, 1) = -sin (theta);
    gt_pose (1, 0) = sin (theta);
    gt_pose (1, 1) = std::cos (theta);

    // A translation on Z axis (0.4 meters)
    gt_pose (0, 3) = 0;
    // A translation on Z axis (0.4 meters)
    gt_pose (1, 3) = 0;
    // A translation on Z axis (0.4 meters)
    gt_pose (2, 3) = 0;

    // Display in terminal the transformation matrix
//    print4x4Matrix (gt_pose);

    // Executing the transformation
    pcl::transformPointCloud (*cloud_in, *cloud_tr, gt_pose);
    pcl::VoxelGrid<PointT> down_size_filter;
    down_size_filter.setLeafSize(0.05, 0.05, 0.05);
    down_size_filter.setInputCloud(cloud_tr);
    down_size_filter.filter(*cloud_tr);
    down_size_filter.setInputCloud(cloud_in);
    down_size_filter.filter(*cloud_in);

    *cloud_icp = *cloud_tr;  // We backup cloud_icp into cloud_tr for later use
    *cloud_ndt = *cloud_tr;  // We backup cloud_icp into cloud_tr for later use
    *cloud_gaussnewton = *cloud_tr;
    // The Iterative Closest Point algorithm


    OptimizedICPGN icp_gaussnewton;
    icp_gaussnewton.set_max_iterations(1);
    icp_gaussnewton.SetTargetCloud(cloud_in);


    pcl::IterativeClosestPoint<PointT, PointT> icp;
//    icp.setMaxCorrespondenceDistance(100);
    icp.setInputSource (cloud_icp);
    icp.setInputTarget (cloud_in);
//    icp.align (*cloud_icp);
    icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function

    pcl::NormalDistributionsTransform<PointT,PointT> ndt;
//    icp.setMaxCorrespondenceDistance(100);
    ndt.setStepSize(ndt_stepsize);
    ndt.setResolution(ndt_resolution);
    ndt.setMaximumIterations(0);
    ndt.setInputSource(cloud_ndt);
    ndt.setInputTarget(cloud_in);
//    ndt.setOulierRatio(ndt_outratio);
//    ndt.align(*cloud_ndt);

    // Visualization
    pcl::visualization::PCLVisualizer viewer ("ICP demo");
    // Create two vertically separated viewports
    int v1 (0);
    int v2 (1);
    int v3 (2);
    int v4 (3);
    viewer.createViewPort (0.0, 0.0, 0.5, 0.5, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 0.5, v2);
    viewer.createViewPort (0.0, 0.5, 0.5, 1.0, v3);
    viewer.createViewPort (0.5, 0.5, 1.0, 1.0, v4);

    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                               (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v3", v3);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v4", v4);

    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
    viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
    viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

    // NDT aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_ndt_color_h (cloud_ndt, 180, 20, 20);
    viewer.addPointCloud (cloud_ndt, cloud_ndt_color_h, "cloud_ndt_v3", v3);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_gaussnewton_color_h (cloud_gaussnewton, 180, 20, 20);
    viewer.addPointCloud (cloud_gaussnewton, cloud_gaussnewton_color_h, "cloud_gaussnewton_v4", v4);

    // Adding text descriptions in each viewport
    viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);
    viewer.addText ("White: Original point cloud\nRed: NDT aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "ndt_info_2", v3);
    viewer.addText ("White: Original point cloud\nRed: GaussNewton aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "gaussnewton_info_2", v4);
    std::stringstream ss;
    ss << iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str ();
    viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
    iterations_cnt = "NDT iterations = " + ss.str ();
    viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "ndt_iterations_cnt", v3);
    iterations_cnt = "Gaussnewton iterations = " + ss.str ();
    viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "gaussnewton_iterations_cnt", v4);

    // Set background color
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v3);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v4);

    // Set camera position and orientation
    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1920, 1080);  // Visualiser window size

    // Register keyboard callback :
    viewer.registerKeyboardCallback (&keyboardEventCallback, (void*) NULL);

    // Display the visualiser
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
//        std::cout<<"running"<<std::endl;

        // The user pressed "space" :
        if (next_iteration)
        {
        	std::cout<<"current iteration time:  "<<iterations<<std::endl;
            // The Iterative Closest Point algorithm
            time.tic ();
            icp.align (*cloud_icp);
            double icp_time = time.toc();
            time.tic ();
            ndt.align (*cloud_ndt);
            double ndt_time = time.toc();
            time.tic();
            Eigen::Matrix4f gn_result = Eigen::Matrix4f::Identity();
            icp_gaussnewton.Match(cloud_gaussnewton, Eigen::Matrix4f::Identity(), cloud_gaussnewton, gn_result);
            double gaussneton_time = time.toc();
            time.tic ();
            static std::string filename = "/home/zack/Data/result.txt";
            static std::ofstream filestream (filename.c_str());
            Eigen::Matrix4d delta_matrix;
            Eigen::Matrix4d final_transform = icp.getFinalTransformation().cast<double>();
            delta_matrix.block<3,3>(0,0) = (final_transform.block<3,3>(0,0)).inverse() * gt_pose.block<3,3>(0,0);
//            delta_matrix.block<3,3>(0,0).normalize();
            delta_matrix(0,3) = gt_pose(0,3) - final_transform(0,3);
            delta_matrix(1,3) = gt_pose(1,3) - final_transform(1,3);
            delta_matrix(2,3) = gt_pose(2,3) - final_transform(2,3);
            delta_matrix(3,3) = 0;
            std::cout<<"icp delta matrix:"<<'\n'<<delta_matrix<<std::endl;
            std::cout<<"icp final_transform matrix:"<<'\n'<<final_transform<<std::endl;
            std::cout<<"icp gt_pose matrix:"<<'\n'<<gt_pose<<std::endl;
            filestream<< iterations<<" "<<icp_time<<" "<<icp.getFitnessScore()<<" "<<std::acos(0.5 * (delta_matrix.coeff (0, 0) + delta_matrix.coeff (1, 1) + delta_matrix.coeff (2, 2) - 1))/M_PI * 180
            		<<" "<< delta_matrix.block<3,1>(0,3).norm();

            final_transform = ndt.getFinalTransformation().cast<double>();
            delta_matrix.block<3,3>(0,0) = (final_transform.block<3,3>(0,0)).inverse() * gt_pose.block<3,3>(0,0);
//            delta_matrix.block<3,3>(0,0).normalize();
            delta_matrix(0,3) = gt_pose(0,3) - final_transform(0,3);
            delta_matrix(1,3) = gt_pose(1,3) - final_transform(1,3);
            delta_matrix(2,3) = gt_pose(2,3) - final_transform(2,3);
            delta_matrix(3,3) = 0;
            std::cout<<"ndt delta matrix:"<<'\n'<<delta_matrix<<std::endl;
            std::cout<<"ndt final_transform matrix:"<<'\n'<<final_transform<<std::endl;
            std::cout<<"ndt gt_pose matrix:"<<'\n'<<gt_pose<<std::endl;

            filestream<<" "<<ndt_time<<" "<<ndt.hasConverged ()<<" "<<ndt.getFitnessScore()<<" "<<std::acos(0.5 * (delta_matrix.coeff (0, 0) + delta_matrix.coeff (1, 1) + delta_matrix.coeff (2, 2) - 1))/M_PI * 180
            		<<" "<< delta_matrix.block<3,1>(0,3).norm()<<" "<<ndt.getTransformationProbability()<<" "<<gaussneton_time<<" "<<cloud_in->size()<<" "<<cloud_gaussnewton->size()<<std::endl;


            if (icp.hasConverged ())
            {

                ss.str ("");
                ss << iterations;
                std::string iterations_cnt = "ICP iterations = " + ss.str ();
                viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
            }
            else
            {
                PCL_ERROR ("\nICP has not converged.\n");
                return (-1);
            }
            if (ndt.hasConverged ())
            {
                ss.str ("");
                ss << iterations;
                std::string iterations_cnt = "NDT iterations = " + ss.str ();
                viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "ndt_iterations_cnt");
                viewer.updatePointCloud (cloud_ndt, cloud_ndt_color_h, "cloud_ndt_v3");
            }
            else
            {
                PCL_ERROR ("\nNDT has not converged.\n");
                return (-1);
            }
            if (icp_gaussnewton.HasConverged ())
            {

                ss.str ("");
                ss << iterations;
                std::string iterations_cnt = "Gaussnewton iterations = " + ss.str ();
                viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "gaussnewton_iterations_cnt");
                viewer.updatePointCloud (cloud_gaussnewton, cloud_gaussnewton_color_h, "cloud_gaussnewton_v4");
            }
            else
            {
                PCL_ERROR ("\nGaussNewton has not converged.\n");
                return (-1);
            }
            iterations++;
            std::cout<<"chores time elapsed: "<<time.toc()<<std::endl;
        }
        next_iteration = false;
    }
    return (0);
}



