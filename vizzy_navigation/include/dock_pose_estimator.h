/* 
 * Copyright 2025, Joao Avelino, João Penha Lopes and João Zenário.
 * This code was originaly developped by João Avelino for ROS1 in 2019.
 * It was later refactored for ROS2 in 2025 by João Penha Lopes and João Zenário.
 * All rights reserved.
 */

/*********************************************************************************************
* Dock Pose Estimator Header File for ROS2                                                   *
* -                                                                                          *
* This file defines the DockPoseEstimator class, which is responsible for estimating the pose*
* of a docking pattern using laser scans and PCL (Point Cloud Library). It includes methods  *
* for subscribing to laser scan data, processing the data to estimate the docking pose, and  *
* publishing the estimated pose. The class also handles parameter loading and filtering of   *
* the estimated pose using a median filter.                                                  *
* -                                                                                          *
* This docking estimation procedure was redesigned to work with Nav2's Docking Server.       *
* More details can be found in the documentation:                                            * 
* https://github.com/open-navigation/opennav_docking/tree/humble                             *
*********************************************************************************************/

#ifndef DOCK_POSE_ESTIMATOR_ROS2_H_
#define DOCK_POSE_ESTIMATOR_ROS2_H_

#include <deque>
#include <memory>
#include <string>
#include <functional>   
#include <yaml-cpp/yaml.h>
#include <tf2_ros/buffer.h>        
#include <tf2_eigen/tf2_eigen.hpp>
#include "pattern_pose_estimation.h" 
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

/**
 * @brief The DockPoseEstimator class is responsible for estimating the pose of a docking pattern.
 * It subscribes to laser scan data, processes it to estimate the docking pose, and publishes the result
 * as a PoseStamped message. The class also handles parameter loading and filtering of the estimated pose.
 * This class inherits from rclcpp_lifecycle::LifecycleNode, as to be managed by the ROS2 Lifecycle Manager.
 */
class DockPoseEstimator : public rclcpp_lifecycle::LifecycleNode
{
public:
    /**
     * @brief Constructor for the DockPoseEstimator class.
     * The constructor is a special member function that is called automatically
     * when an object of the class is created.
     * The constructor will initialize the node.
     * The constructor takes a single argument of type rclcpp::NodeOptions,
     * which allows for additional options to be passed when creating the node.
     */
    explicit DockPoseEstimator(const rclcpp::NodeOptions & options);

    /**
     * @brief Destructor for the DockPoseEstimator class.
     * The destructor is also a special member function that is called automatically
     * when an object of the class is destroyed.
     * In this case, the destructor does not need to do anything.
     */
    ~DockPoseEstimator(){};

    /**
     * @brief Returns the estimated pose of the dock pattern.
     * The node's state is controlled by the ROS 2 Lifecycle Manager,
     * which makes these manual control functions obsolete and simplifies the class API.
     */
    geometry_msgs::msg::PoseStamped getPatternPose();

private:

    /**
     * @brief Loads parameters from the ROS 2 node.
     * This method is called during the configuration phase of the node's lifecycle.
     * It retrieves parameters such as the laser topic name, filter buffer size, and other settings
     * necessary for the operation of the DockPoseEstimator.
     */
    void declareAndGetParameters();

    /**
     * @brief Computes the median of a deque.
     * This method takes a deque of doubles as input and returns the median value.
     * The median is a measure of central tendency that is less sensitive to outliers than the mean.
     * @param a A deque of doubles from which to compute the median.
     * @return The median value of the input deque.
     */
    double findMedian(std::deque<double> a);


    /**
     * @brief Callback for processing laser scan data.
     * This method is called whenever a new laser scan message is received on the subscribed topic.
     * It processes the laser scan data to estimate the pose of the docking pattern and publishes
     * the estimated pose if the estimator is enabled and ready.
     * 
     * Based on the received laser scan, the method performs the following steps:
     * 1. Converts the laser scan to a point cloud.
     * 2. Transforms the point cloud to the "base_link" frame.
     * 3. Estimates the pose of the docking pattern using the PatternPoseEstimation class.
     * 4. Applies a median filter to the estimated pose to smooth out noise and outliers.
     * 5. Publishes the filtered pose as a PoseStamped message.
     *
     * Median Filter:
     *   The median filter is a non-linear digital filtering technique, often used to remove noise from signals.
     *   It works by moving a window of a specified size over the data and replacing each value with from the window with the median value of that window.
     *
     *   Take a slice of values such as [2, 3, 80, 6, 2, 3] and a window size of 3.
     *   The median filter will process the values as follows:
     *
     *   1. For the first three values [0, 2, 3], the median is 2. The boundary value is taken to be 0.
     *
     *   2. For the second window [2, 3, 80], the median is 3.
     * 
     *   3. For the third window [3, 80, 6], the median is 6.
     * 
     *   4. For the fourth window [80, 6, 2], the median is 6.
     * 
     *   5. For the fifth window [6, 2, 3], the median is 3.
     * 
     *   6. For the last window [2, 3, 0], the median is 2.
     *
     * Thus, the filtered output would be [2, 3, 6, 6, 3, 2], effectively smoothing out the 80 value which is an outlier in the original data.
     *
     * For more information on Median Filters, refer to: https://en.wikipedia.org/wiki/Median_filter
     * 
     * @param scan A shared pointer to the received LaserScan message.
     */
    void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> scan);

    /**
     * @brief on_configure: This callback is called when the node transitions from 'unconfigured' to 'inactive'.
     * This is where we load parameters and set up resources that are needed for the node to operate.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);

    /**
     * @brief on_activate: This callback is called when the node transitions from 'inactive' to 'active'.
     * This is where we create subscriptions and start processing data.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);

    /**
    * @brief on_deactivate: This callback is called when the node transitions from 'active' to 'inactive'.
    * This is where we destroy subscriptions and timers to stop processing data and save resources.
    */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

    /**
     * @brief on_cleanup: This callback is called when the node transitions from 'inactive' to 'finalized'.
     * This is where we release resources that were allocated during the node's lifecycle.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

    /**
     * @brief on_shutdown: This callback is called when the node is shutting down.
     * This is where we perform any necessary cleanup before the node is destroyed.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

    /**
     * @brief Subscription object to the laser topic. Messages are received through here.
     */
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

    /**
     * @brief Publishers for the estimated docking pose.
     */
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> docking_pub_;

    /**
     * @brief Publisher for the model point cloud.
     */
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> model_pub_;

    /**
     * @brief Publisher for the scene point cloud.
     */
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> scene_cloud_pub_;

    /**
     * @brief Buffer for storing TF2 transforms.
     */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    /**
     * @brief Listener for TF2 transforms.
     */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /**
     * @brief PoseStamped message that holds the last estimated pose of the dock pattern.
     */
    geometry_msgs::msg::PoseStamped dock_pose_;

    /**
     * @brief Deque for the median filter.
     */
    std::deque<double> yaw_filter_;

    /**
     * @brief Deque for the yaw filter.
     */
    std::deque<double> x_filter_;

    /**
     * @brief Deque for the y filter.
     */
    std::deque<double> y_filter_;

    /**
     * @brief Deque for the z filter.
     */
    std::deque<double> z_filter_;

    /**
     * @brief Flag indicating if the estimator is enabled (now controlled by on_activate/on_deactivate).
     */
    bool enabled_ = false;

    /**
     * @brief Flag indicating if the estimator is ready to provide estimates (now controlled by on_configure).
     */
    bool ready_ = false;

    /**
     * @brief Size of the filter buffer for pose estimation.
     */
    unsigned int filter_buffer_size_;

    /**
     * @brief Pointer to the PatternPoseEstimation object for pose estimation.
     */
    std::shared_ptr<PatternPoseEstimation> pattern_pose_estimation_;

    /**
     * @brief Point cloud for the laser scan.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl_;

    /**
     * @brief Point cloud with normals for the laser scan.
     */
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_;

    /**
     * @brief Topic name for the rear laser scan.
     */
    std::string rear_laser_topic_;

    /**
     * @brief Flag to determine if the dock point cloud should be published.
     */
    bool publish_dock_point_cloud_ = false;
};

#endif // DOCK_POSE_ESTIMATOR_ROS2_H_