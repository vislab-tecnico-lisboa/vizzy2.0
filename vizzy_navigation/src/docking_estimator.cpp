/* 
 * Copyright 2025, Joao Avelino, João Penha Lopes and João Zenário.
 * This code was originaly developped by João Avelino for ROS1 in 2019.
 * It was later refactored for ROS2 in 2025 by João Penha Lopes and João Zenário.
 * All rights reserved.
 */

/*********************************************************************************************
*                         Docking Estimator C++ File for ROS2                                *
*                                          -                                                 *
* This file contains the implementation of every method defined in the                       *
* DockingEstimator class header file.                                                        *
* It includes the constructor, parameter loading, laser scan processing,                     *
* median filtering, and publishing of the estimated docking pose.                            *
*                                          -                                                 *
* This docking estimation procedure was redesigned to work with Nav2's Docking Server.       *
* More details can be found in the documentation:                                            *   
* https://github.com/open-navigation/opennav_docking/tree/humble                             *    
*********************************************************************************************/

// Include necessary headers for ROS2, and other dependencies.
#include "docking_estimator.h" 
#include <functional>           
#include "tf2_eigen/tf2_eigen.hpp"

// Constructor implementation for the DockingEstimator class.
// This constructor uses a member initializer list to initialize the base class Node,
// this happens before the main body of the constructor is executed,
// which makes the DockingEstimator object initialization more efficient.
// The first member of the initializer list is the Node class constructor, 
// inherited from rclcpp::Node, it initializes the node with the name 
// "docking_estimator_node" and passes the options provided.
// The second member initializes the filter buffer size to 10, 
// which is used for median filtering of the pose estimates.
DockingEstimator::DockingEstimator(const rclcpp::NodeOptions & options)

    /*
        Member Initializer List:
        ** - `Node("docking_estimator_node", options)`: Initializes the base class Node.
        ** - `filter_buffer_size_(10)`: Initializes the filter buffer size to 10.
    */
    : Node("docking_estimator_node", options),
      filter_buffer_size_(10)


{
    // Create a shared pointer for the TF2 buffer and listener.
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare and get necessary parameters.
    this->declareAndGetParameters();
    
    // Create ROS2 publishers.
    docking_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/dock_pose", 10);
    model_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/model_point_cloud", 10);

    // Create Subscriber.
    // Start with the front laser by default.
    this->useFrontLaser();

    // Log a message indicating that the DockingEstimator node has been initialized.
    RCLCPP_INFO(this->get_logger(), "Docking Estimator node has been initialized.");
}

// Declares and loads the necessary parameters from the ROS2 node and 
// initializes the pattern_pose_estimation_ member with a shared pointer to a new
// PatternPoseEstimation object with these parameters.
void DockingEstimator::declareAndGetParameters()
{
    // Declare parameters with default values.
    // Attention: The parameters may not be set with these default values,
    // they are just declared here. That is why we retrieve them later.
    this->declare_parameter<double>("tran_thresh", 0.05);
    this->declare_parameter<double>("rot_thresh", 30.0);
    this->declare_parameter<double>("fitting_score_thresh", 0.01);
    this->declare_parameter<double>("discretization_step", 0.01);
    this->declare_parameter<double>("pattern_distance_threshold", 1.5);
    this->declare_parameter<std::string>("model_file", "file");

    // Get the current active parameter values.
    double tran_thresh = this->get_parameter("tran_thresh").as_double();
    double rot_thresh = this->get_parameter("rot_thresh").as_double();
    double fitting_score_thresh = this->get_parameter("fitting_score_thresh").as_double();
    double discretization_step = this->get_parameter("discretization_step").as_double();
    double pattern_distance_threshold = this->get_parameter("pattern_distance_threshold").as_double();
    std::string model_file = this->get_parameter("model_file").as_string();

    // Log the parameters for debugging.
    RCLCPP_INFO(this->get_logger(), "tran_thresh: %f", tran_thresh);
    RCLCPP_INFO(this->get_logger(), "rot_thresh: %f", rot_thresh);
    RCLCPP_INFO(this->get_logger(), "fitting_score_thresh: %f", fitting_score_thresh);
    RCLCPP_INFO(this->get_logger(), "discretization_step: %f", discretization_step);
    RCLCPP_INFO(this->get_logger(), "pattern_distance_threshold: %f", pattern_distance_threshold);

    // Initialize the Point Pair Feature (PPF) registration model with the loaded parameters.
    // Provide the shared pointer to the private member `pattern_pose_estimation_`.
    pattern_pose_estimation_ = std::make_shared<PatternPoseEstimation>(
        rot_thresh, tran_thresh, fitting_score_thresh, discretization_step,
        model_file, pattern_distance_threshold);
}

// This method switches the laser subscription to the front laser topic.
// It resets the existing subscription and creates a new one for the specified topic.
void DockingEstimator::useFrontLaser(std::string laser_topic_front)
{
    // Resetting the smart pointer effectively shuts down the old subscription.
    point_cloud_sub_.reset();
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        laser_topic_front, 10, std::bind(&DockingEstimator::pointCloudCallback, this, std::placeholders::_1));
}

// This method switches the laser subscription to the rear laser topic.
// Similar to the front laser, it resets the existing subscription and creates a new one for the specified topic.
void DockingEstimator::useRearLaser(std::string laser_topic_rear)
{
    // Resetting the smart pointer effectively shuts down the old subscription.
    point_cloud_sub_.reset();
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        laser_topic_rear, 10, std::bind(&DockingEstimator::pointCloudCallback, this, std::placeholders::_1));
}

// Main callback for processing laser data.
void DockingEstimator::pointCloudCallback(const std::shared_ptr<sensor_msgs::msg::PointCloud2> scan)
{

    // If the estimator is not enabled, do nothing.
    if (!enabled_) return;

    // * The initial version of this method was called `laserCallback`, 
    // * but it has been renamed to `pointCloudCallback` to better reflect its functionality.
    // * Since the hokuyo LiDAR scanner also publishes PointCloud2 messages, 
    // * we now use directly the PointCloud2 message type instead of LaserScan, to 
    // * avoid unnecessary conversions and to handle the data more efficiently.

    // Convert the incoming ROS 2 PointCloud2 message to a PCL point cloud.
    pcl::fromROSMsg(*scan, *cloud_pcl_);
    cloud_normals_ = pattern_pose_estimation_->getPointNormal(cloud_pcl_);

    // Compute the transform using PPF.
    Eigen::Affine3d transformNN;
    try {
        transformNN = pattern_pose_estimation_->detect(cloud_normals_);
    } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Detection failed: %s", e.what());
        return;
    }

    // Extract pose from transform.
    Eigen::Matrix3d rotation = transformNN.rotation();
    Eigen::Vector3d translation = transformNN.translation();
    Eigen::Vector3d ea = rotation.eulerAngles(2, 1, 0); // ZYX yaw, pitch, roll

    if (-M_PI * 0.5 > ea[0]) { ea[0] += M_PI; } 
    else if (M_PI * 0.5 < ea[0]) { ea[0] -= M_PI; }

    // Apply Median Filter.
    x_filter_.push_back(translation[0]);
    y_filter_.push_back(translation[1]);
    z_filter_.push_back(translation[2]);
    yaw_filter_.push_back(ea[0]);

    if (yaw_filter_.size() >= filter_buffer_size_) {
        x_filter_.pop_front(); y_filter_.pop_front(); z_filter_.pop_front(); yaw_filter_.pop_front();
    }

    double x_filtered = findMedian(x_filter_);
    double y_filtered = findMedian(y_filter_);
    double z_filtered = findMedian(z_filter_);
    double yaw_filtered = findMedian(yaw_filter_);

    // Reconstruct filtered transform.
    Eigen::Matrix3d m = Eigen::AngleAxisd(yaw_filtered, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Affine3d transformNNfiltered = Eigen::Affine3d::Identity();
    transformNNfiltered.translate(Eigen::Vector3d(x_filtered, y_filtered, z_filtered));
    transformNNfiltered.rotate(m);
    
    // Convert Eigen transform to PoseStamped message and publish the dock pose.
    dock_pose_.pose = tf2::toMsg(transformNNfiltered);
    dock_pose_.header = scan->header;
    docking_pub_->publish(dock_pose_);

    // Publish model for visualization.
    pattern_pose_estimation_->getOutputCloud()->header.frame_id = scan->header.frame_id;

    // PCL timestamp conversion.
    pcl_conversions::toPCL(scan->header, pattern_pose_estimation_->getOutputCloud()->header);

    // Create a ROS 2 message object.
    sensor_msgs::msg::PointCloud2 output_cloud_msg;

    // Convert PCL data into the ROS 2 message.
    pcl::toROSMsg(*pattern_pose_estimation_->getOutputCloud(), output_cloud_msg);

    // Make sure the header is set correctly on the ROS message.
    output_cloud_msg.header = scan->header;

    // Publish the ROS 2 message with the model point cloud.
    model_pub_->publish(output_cloud_msg);
}

// Getter for the dock pose, returns the current estimated docking pose.
geometry_msgs::msg::PoseStamped DockingEstimator::getPatternPose()
{
    return dock_pose_;
}

// This method computes the median of a deque of doubles.
double DockingEstimator::findMedian(std::deque<double> a)
{
    if (a.empty()) return 0.0;
    std::sort(a.begin(), a.end());
    return a.at(a.size() / 2);
}