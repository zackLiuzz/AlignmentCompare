/*
 * GaussNewton.cpp
 *
 *  Created on: Aug 4, 2021
 *      Author: zack
 */

#include "GaussNewton.h"
#include "Sophus/sophus/se3.hpp"
namespace trunk{

OptimizedICPGN::OptimizedICPGN()
        : kdtree_flann_ptr_(new pcl::KdTreeFLANN<PointT>) {
    max_iterations_ = 1;
    max_correspond_distance_ = std::numeric_limits<float>::max();
}

bool OptimizedICPGN::SetTargetCloud(const PointCloudT::Ptr &target_cloud_ptr) {
    target_cloud_ptr_ = target_cloud_ptr;
    kdtree_flann_ptr_->setInputCloud(target_cloud_ptr);//构建kdtree用于全局最近邻搜索
}

bool OptimizedICPGN::Match(const PointCloudT::Ptr &source_cloud_ptr, const Eigen::Matrix4f &predict_pose,
		PointCloudT::Ptr &transformed_source_cloud_ptr, Eigen::Matrix4f &result_pose) {
    source_cloud_ptr_ = source_cloud_ptr;

    PointCloudT::Ptr transformed_cloud(new PointCloudT);
    final_transformation_ = Eigen::Matrix4f::Identity();

//    Eigen::Matrix4f T = predict_pose;

    //Gauss-Newton's method solve ICP.
    for (unsigned int i = 0; i < max_iterations_; ++i) {
//        pcl::transformPointCloud(*source_cloud_ptr, *transformed_cloud, T);
        Eigen::Matrix<float, 6, 6> Hessian = Eigen::Matrix<float, 6, 6>::Zero();
        Eigen::Matrix<float, 6, 1> B = Eigen::Matrix<float, 6, 1>::Zero();
        pcl::console::TicToc time;
//        time.tic();

        for (unsigned int j = 0; j < source_cloud_ptr->size(); ++j) {
//        	time.tic();
            const PointT &origin_point = source_cloud_ptr->points[j];

            //删除距离为无穷点
            if (!pcl::isFinite(origin_point)) {
                continue;
            }

//            const PointT &transformed_point = transformed_cloud->at(j);
            std::vector<float> resultant_distances;
            std::vector<int> indices;
            //在目标点云中搜索距离当前点最近的一个点
            kdtree_flann_ptr_->nearestKSearch(origin_point, 1, indices, resultant_distances);

//            std::cout<<std::fixed<<std::setprecision(20)<<"search time: "<<time.toc()/1000<<std::endl;
//            time.tic();

            //舍弃那些最近点,但是距离大于最大对应点对距离
            if (resultant_distances.front() > max_correspond_distance_) {
                continue;
            }

            Eigen::Vector3f nearest_point = Eigen::Vector3f(target_cloud_ptr_->at(indices.front()).x,
                                                            target_cloud_ptr_->at(indices.front()).y,
                                                            target_cloud_ptr_->at(indices.front()).z);

//            Eigen::Vector3f point_eigen(transformed_point.x, transformed_point.y, transformed_point.z);
            Eigen::Vector3f origin_point_eigen(origin_point.x, origin_point.y, origin_point.z);
            Eigen::Vector3f error = origin_point_eigen - nearest_point;

            Eigen::Matrix<float, 3, 6> Jacobian = Eigen::Matrix<float, 3, 6>::Zero();
//            std::cout<<std::fixed<<std::setprecision(20)<<"before jacobian time: "<<time.toc()/1000<<std::endl;
//            time.tic();
            //构建雅克比矩阵
            Jacobian.leftCols(3) = Eigen::Matrix3f::Identity();
            Jacobian.rightCols(3) = - Sophus::SO3f::hat(origin_point_eigen);
//            std::cout<<std::fixed<<std::setprecision(20)<<"jacobian time: "<<time.toc()/1000<<std::endl;
//            time.tic();
            //构建海森矩阵
            Hessian += Jacobian.transpose() * Jacobian;
            B += -Jacobian.transpose() * error;
//            std::cout<<std::fixed<<std::setprecision(20)<<"one point add time: "<<time.toc()/1000<<std::endl;
        }
        if (Hessian.determinant() == 0) {
            continue;
        }
        time.tic();
        Eigen::Matrix<float, 6, 1> delta_x = Hessian.inverse() * B;
//        std::cout<<"solve time elapsed: "<<time.toc()<<std::endl;
        final_transformation_.block<3, 1>(0, 3) = final_transformation_.block<3, 1>(0, 3) + delta_x.head(3);
        final_transformation_.block<3, 3>(0, 0) *= Sophus::SO3f::exp(delta_x.tail(3)).matrix();

    }

//    final_transformation_ = T;
    result_pose = final_transformation_;
    pcl::transformPointCloud(*source_cloud_ptr, *transformed_source_cloud_ptr, result_pose);

    return true;
}

//该函数用于计算匹配之后的得分,其写法与pcl中的计算icp或者ndt的方式是一致,
//他们之间的得分可以进行比较
float OptimizedICPGN::GetFitnessScore() {
    float max_range = std::numeric_limits<float>::max();
    float fitness_score = 0.0f;

    PointCloudT::Ptr transformed_cloud_ptr(new PointCloudT);
    pcl::transformPointCloud(*source_cloud_ptr_, *transformed_cloud_ptr, final_transformation_);

    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);

    int nr = 0;

    for (unsigned int i = 0; i < transformed_cloud_ptr->size(); ++i) {
        kdtree_flann_ptr_->nearestKSearch(transformed_cloud_ptr->points[i], 1, nn_indices, nn_dists);

        if (nn_dists.front() <= max_range) {
            fitness_score += nn_dists.front();
            nr++;
        }
    }

    if (nr > 0)
        return fitness_score / static_cast<float>(nr);
    else
        return (std::numeric_limits<float>::max());
}

bool OptimizedICPGN::HasConverged() {
    ///TODO: add this function
    return true;
}
}
