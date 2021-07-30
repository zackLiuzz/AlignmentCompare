/*
 * main.cpp
 *
 *  Created on: Jul 29, 2021
 *      Author: zack
 */
#include "common.h"
#include "g2o_solver.h"
#include "ceres_solver.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
using namespace trunk;
bool next_iteration = false;
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
    time.tic ();
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(argv[1], *cloud_in) < 0)
    {
        PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
        return (-1);
    }
    std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;

    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

    // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    double theta = M_PI_4 / 16;  // The angle of rotation in radians
    transformation_matrix (0, 0) = std::cos (theta);
    transformation_matrix (0, 1) = -sin (theta);
    transformation_matrix (1, 0) = sin (theta);
    transformation_matrix (1, 1) = std::cos (theta);

    // A translation on Z axis (0.4 meters)
    transformation_matrix (0, 3) = 0;
    // A translation on Z axis (0.4 meters)
    transformation_matrix (1, 3) = 0;
    // A translation on Z axis (0.4 meters)
    transformation_matrix (2, 3) = 0;

    // Display in terminal the transformation matrix
    std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
    print4x4Matrix (transformation_matrix);

    // Executing the transformation
    pcl::transformPointCloud (*cloud_in, *cloud_tr, transformation_matrix);
    *cloud_icp = *cloud_tr;  // We backup cloud_icp into cloud_tr for later use
    *cloud_pre_ndt = *cloud_tr;  // We backup cloud_icp into cloud_tr for later use
    // The Iterative Closest Point algorithm
    pcl::VoxelGrid<PointT> down_size_filter;
    down_size_filter.setLeafSize(0.15, 0.15, 0.15);
    down_size_filter.setInputCloud(cloud_pre_ndt);
    down_size_filter.filter(*cloud_ndt);
    time.tic ();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
//    icp.setMaxCorrespondenceDistance(100);
    icp.setInputSource (cloud_icp);
    icp.setInputTarget (cloud_in);
//    icp.align (*cloud_icp);
    icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

    pcl::NormalDistributionsTransform<PointT,PointT> ndt;
//    icp.setMaxCorrespondenceDistance(100);
    ndt.setStepSize(0.2);
    ndt.setResolution(0.5);
    ndt.setMaximumIterations(1);
    ndt.setInputSource(cloud_ndt);
    ndt.setInputTarget(cloud_in);
    ndt.setOulierRatio(0.2);
//    ndt.align(*cloud_ndt);


//    if (icp.hasConverged ())
//    {
//        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
//        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
//        transformation_matrix = icp.getFinalTransformation ().cast<double>();
//        print4x4Matrix (transformation_matrix);
//    }
//    else
//    {
//        PCL_ERROR ("\nICP has not converged.\n");
//        return (-1);
//    }
//    if (ndt.hasConverged ())
//    {
//        std::cout << "\nNDT has converged, score is " << ndt.getFitnessScore () << std::endl;
//        std::cout << "\nNDT transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
//        transformation_matrix = ndt.getFinalTransformation ().cast<double>();
//        print4x4Matrix (transformation_matrix);
//    }
//    else
//    {
//        PCL_ERROR ("\nNDT has not converged.\n");
//        return (-1);
//    }

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

    // Adding text descriptions in each viewport
    viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);
    viewer.addText ("White: Original point cloud\nRed: NDT aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "ndt_info_2", v3);
    std::stringstream ss;
    ss << iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str ();
    viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
    iterations_cnt = "NDT iterations = " + ss.str ();
    viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "ndt_iterations_cnt", v3);

    // Set background color
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v3);

    // Set camera position and orientation
    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1920, 1080);  // Visualiser window size

    // Register keyboard callback :
    viewer.registerKeyboardCallback (&keyboardEventCallback, (void*) NULL);

    // Display the visualiser
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();

        // The user pressed "space" :
        if (next_iteration)
        {
            // The Iterative Closest Point algorithm
            time.tic ();
            icp.align (*cloud_icp);
            std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;
            time.tic ();
            ndt.align (*cloud_ndt);
            std::cout << "Applied 1 NDT iteration in " << time.toc () << " ms" << std::endl;

            if (icp.hasConverged ())
            {
                printf ("\033[11A");  // Go up 11 lines in terminal output.
                printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
                std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
                transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
                print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

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
            std::cout<<"ndt.hasConverged (): "<<ndt.hasConverged ()<<std::endl;
            if (/*ndt.hasConverged ()*/1)
            {
                printf ("\033[11A");  // Go up 11 lines in terminal output.
                printf ("\nNDT has converged, score is %+.0e\n", ndt.getFitnessScore ());
                std::cout << "\nNDT transformation " << iterations << " : cloud_ndt -> cloud_in" << std::endl;
                transformation_matrix *= ndt.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
                print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

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
            iterations++;
        }
        next_iteration = false;
    }
    return (0);
}


