/* 
 * Copyright 2025, Joao Avelino, João Penha Lopes and João Zenário.
 * This code was originaly developped by João Avelino for ROS1 in 2019.
 * It was later refactored for ROS2 in 2025 by João Penha Lopes and João Zenário.
 * All rights reserved.
 */

/*********************************************************************************************
*                       Dock Pose Estimator Header File for ROS2                             *
*                                          -                                                 *
* This file defines the DockPoseEstimator class, which is responsible for estimating the pose*
* of a docking pattern using laser scans and PCL (Point Cloud Library). It includes methods  *
* for subscribing to laser scan data, processing the data to estimate the docking pose, and  *
* publishing the estimated pose. The class also handles parameter loading and filtering of   *
* the estimated pose using a median filter.                                                  *
*                                          -                                                 *
* This docking estimation procedure was redesigned to work with Nav2's Docking Server.       *
* More details can be found in the documentation:                                            *   
* https://github.com/open-navigation/opennav_docking/tree/humble                             *                                                     *
*********************************************************************************************/

// Include guards to prevent multiple inclusions of this header file.
#ifndef DOCK_POSE_ESTIMATOR_ROS2_HPP_
#define DOCK_POSE_ESTIMATOR_ROS2_HPP_

// Include necessary headers for ROS2, PCL, and other dependencies.
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "laser_geometry/laser_geometry.hpp"
#include "pattern_pose_estimation.h" 
#include <deque>
#include <memory>
#include <string>

// We define the DockPoseEstimator class here.
// This class inherits from rclcpp::Node, which is the base class for all ROS 2 nodes, thus,    
// allowing it to utilize ROS 2 functionalities such as publishing and subscribing to topics.
class DockPoseEstimator : public rclcpp::Node
{

// The 'public' section is where we define the public interface of the class.
// This section contains the methods that can be called from outside the class,
// allowing users to interact with the DockPoseEstimator.
public:
    // First we define the constructor for the class.
    // The constructor is a special member function that is called automatically
    // when an object of the class is created.
    // The constructor will initialize the node.
    // The constructor takes a single argument of type rclcpp::NodeOptions,
    // which allows for additional options to be passed when creating the node.
    explicit DockPoseEstimator(const rclcpp::NodeOptions & options);

    // After, we define the destructor for the class.
    // The destructor is also a special member function that is called automatically
    // when an object of the class is destroyed.
    // In this case, the destructor does not need to do anything.
    ~DockPoseEstimator(){};

    /*
        * Public API Methods:
        ** - `getPatternPose()`: Returns the estimated pose of the dock pattern.
        ** - `isReady()`: Checks if the estimator is ready to provide estimates.
        ** - `enable()`: Enables the estimator to start processing laser scans.
        ** - `disable()`: Disables the estimator, stopping it from processing scans.
        ** - `useFrontLaser(std::string laser_topic)`: Subscribes to the front laser scan topic.
        ** - `useBackLaser(std::string laser_topic)`: Subscribes to the back laser scan topic.
    */
    geometry_msgs::msg::PoseStamped getPatternPose();
    bool isReady() { return ready_; };
    void enable() { enabled_ = true; };
    void disable() { enabled_ = false; };
    void useFrontLaser(std::string laser_topic = "/nav_hokuyo_laser/front/scan");
    void useRearLaser(std::string laser_topic = "/nav_hokuyo_laser/rear/scan");

// The 'private' section is where we define the private members of the class.
// These members are not accessible from outside the class, ensuring encapsulation.
private:

    /*
        * Private API Methods:
        ** - `declareAndGetParameters()`: Loads parameters from the ROS 2 node.
        ** - `findMedian(std::deque<double> a)`: Computes the median of a deque.
        ** - `laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> scan)`: Callback for processing laser scan data.
    */
    void declareAndGetParameters();
    double findMedian(std::deque<double> a);
    void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> scan);

    /*
        * Private Member Variables:
        ** - `laser_sub_`: Subscription object to the laser topic. Messages are received through here.
        ** - `docking_pub_`: Publisher object for the estimated docking pose. Messages are published through here.
        ** - `model_pub_`: Publisher object for the model point cloud. Messages are published through here.
        ** - `tf_buffer_`: Buffer for storing TF2 transforms.
        ** - `tf_listener_`: Listener for TF2 transforms.
        ** - `dock_pose_`: PoseStamped message that holds the last estimated pose of the dock pattern.
        ** - `yaw_filter_`, `x_filter_`, `y_filter_`, `z_filter_`: Four std::deque (double-ended queue) containers. 
        They act as sliding window buffers for a median filter, smoothing the raw pose estimates over time to produce a more stable output.

        Deques:
        A double-ended queue (deque) is a generalized version of a queue that does not need to follow FIFO (First In First Out) order.
        Deques allow insertion and deletion of elements from both ends.

        For more information on Deques, refer to: https://www.geeksforgeeks.org/cpp/cpp-implementation-double-ended-queue/

        Median Filter:
        The median filter is a non-linear digital filtering technique, often used to remove noise from signals.
        It works by moving a window of a specified size over the data and replacing each value with from the window with the median value of that window.

        Take a slice of values such as [2, 3, 80, 6, 2, 3] and a window size of 3.
        The median filter will process the values as follows:

        1. For the first three values [0, 2, 3], the median is 2. The boundary value is taked to be 0.
        2. For the second window [2, 3, 80], the median is 3.
        3. For the third window [3, 80, 6], the median is 6.
        4. For the fourth window [80, 6, 2], the median is 6.
        5. For the fifth window [6, 2, 3], the median is 3.
        6. For the last window [2, 3, 0], the median is 2.

        Thus, the filtered output would be [2, 3, 6, 6, 3, 2], effectively smoothing out the 80 value which is an outlier in the original data.

        For more information on Median Filters, refer to: https://en.wikipedia.org/wiki/Median_filter

        ** - `enabled_`: Flag indicating if the estimator is enabled.
        ** - `ready_`: Flag indicating if the estimator is ready to provide estimates.
        ** - `filter_buffer_size`: Size of the filter buffer for pose estimation.
        ** - `pattern_pose_estimation`: Pointer to the PatternPoseEstimation object for pose estimation.
        ** - `cloud_pcl`: Point cloud for the laser scan.
        ** - `cloud_normals`: Point cloud with normals for the laser scan.

        FYI: The trailing underscore in variable names is a common convention in C++ to indicate that the variable is a member of a class.
    */
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr docking_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr model_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scene_cloud_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::PoseStamped dock_pose_;
    std::deque<double> yaw_filter_;
    std::deque<double> x_filter_;
    std::deque<double> y_filter_;
    std::deque<double> z_filter_;
    bool enabled_ = false;
    bool ready_ = false;
    unsigned int filter_buffer_size_;
    std::shared_ptr<PatternPoseEstimation> pattern_pose_estimation_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl_;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_;
    int laser_sub_counter_ = 0;
    bool current_laser_is_front_ = true; // Flag to check if the current laser is the front one.
};

#endif // DOCK_POSE_ESTIMATOR_ROS2_HPP_