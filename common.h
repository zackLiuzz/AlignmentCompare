/*
 * common.h
 *
 *  Created on: Jul 29, 2021
 *      Author: zack
 */

#ifndef COMMON_H_
#define COMMON_H_
#include <iostream>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
namespace trunk{
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
//void print4x4Matrix (const Eigen::Matrix4d & matrix)
//{
////    printf ("Rotation matrix :\n");
////    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
////    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
////    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
////    printf ("Translation vector :\n");
////    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
//}



} //namespace trunk
#endif /* COMMON_H_ */
