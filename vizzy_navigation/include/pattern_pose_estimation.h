/* 
 * Copyright 2025, João Avelino, Rui Figueiredo, João Penha Lopes and João Zenário.
 * This code was originaly developped by João Avelino and Rui Figueiredo for ROS1 in 2019.
 * It was later refactored for ROS2 in 2025 by João Penha Lopes and João Zenário.
 * All rights reserved.
 */

#ifndef PATTERN_POSE_ESTIMATION_HPP_
#define PATTERN_POSE_ESTIMATION_HPP_

#include <random>
#include <chrono>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/console/print.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/parse.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/registration/icp.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ppf_registration.h>
#include <pcl/filters/statistical_outlier_removal.h>

class PatternPoseEstimation
{
private:

    double rot_thresh;
    double tran_thresh;
    double fitting_score_thresh;
    double distance_threshold;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    pcl::PointCloud<pcl::Normal>::Ptr normals_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_output_subsampled;
    double discretization_step;

    // Other members not in the initializer list
    std::vector<pcl::PPFHashMapSearch::Ptr> hashmap_search_vector;
    double cluster_tran_thresh;
    double cluster_rot_thresh;
    
public:
    // --- Public Methods ---
    pcl::PointCloud<pcl::PointNormal>::Ptr getPointNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
    pcl::PointCloud<pcl::PointNormal>::Ptr getOutputCloud() const { return cloud_output_subsampled; }
    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> dense_cloud_models;
    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> cloud_models_with_normals;

    PatternPoseEstimation(  double rot_thresh_=30.0,
                double tran_thresh_=0.05,
                double fitting_score_thresh_=0.01,
                double discretization_step_=0.01,
                double distance_threshold_=1.0,
                std::string file_="file");

    int train (std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> cloud_models_with_normals_);
    Eigen::Affine3d detect(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);

    Eigen::Matrix4f refine(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_target);
    void loadModelFromMesh(std::string file_name="filename");
};

#endif