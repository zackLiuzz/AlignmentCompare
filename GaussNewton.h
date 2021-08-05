/*
 * GaussNewton.h
 *
 *  Created on: Aug 2, 2021
 *      Author: zack
 */

#ifndef GAUSSNEWTON_H_
#define GAUSSNEWTON_H_
#include "common.h"
#include <eigen3/Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
namespace trunk{

//该类是对优化方法的ICP进行的实现,是从我的另一个项目中摘取的代码,
//它并不能直接使用,可能需要你做很小的改动,然后就能应用于你的项目,
//改动主要就是路径的设置,这里只是为了展示优化的ICP实现方法.
class OptimizedICPGN{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;//eigen自动内存对齐

    OptimizedICPGN();

    bool SetTargetCloud(const PointCloudT::Ptr &target_cloud_ptr);
    void set_max_iterations(unsigned int iteration){
    	max_iterations_ = iteration;

    }
    void set_max_correspond_distance(float max_correspond_distance){
    	max_correspond_distance_ = max_correspond_distance;

    }
    Eigen::Matrix4f get_final_transformation(){
    	return final_transformation_;
    }

    bool Match(const PointCloudT::Ptr &source_cloud_ptr,
               const Eigen::Matrix4f &predict_pose,
			   PointCloudT::Ptr &transformed_source_cloud_ptr,
               Eigen::Matrix4f &result_pose);

    float GetFitnessScore();

    bool HasConverged();

private:
    unsigned int max_iterations_;
    float max_correspond_distance_;

    PointCloudT::Ptr target_cloud_ptr_;
    PointCloudT::Ptr source_cloud_ptr_;
    Eigen::Matrix4f final_transformation_;

    pcl::KdTreeFLANN<PointT>::Ptr kdtree_flann_ptr_;//In order to search
};
} //namespace trunk

#endif /* GAUSSNEWTON_H_ */
