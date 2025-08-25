/* 
 * Copyright 2025, Joao Avelino, João Penha Lopes and João Zenário.
 * This code was originaly developped by João Avelino for ROS1 in 2019.
 * It was later refactored for ROS2 in 2025 by João Penha Lopes and João Zenário.
 * All rights reserved.
 */

/*********************************************************************************************
*                         Dock Pose Estimator C++ File for ROS2                              *
*                                          -                                                 *
* This file contains the implementation of every method defined in the                       *
* DockPoseEstimator class header file.                                                       *
* It includes the constructor, parameter loading, laser scan processing,                     *
* median filtering, and publishing of the estimated docking pose.                            *   
*                                          -                                                 *
* To simplify execution of the code and to integrate with the Nav2 ecoystem, the following   *
* approach was used to detect the dock pose:                                                 *
* 1. Whichever laser is used, the algorithm will try to detect the dock pose.                *
* 2. If the dock pattern is not detected, a counter will be incremented.                     *
* 3. If the counter reaches a certain threshold (e.g. 30, meaning 30 consecutive scans       *
* without detection), the algorithm will switch to the other laser. The counter will         *
* be reset to zero in this event.                                                            *
* 4. The counter will be reset to zero when a detection is made.                             *
* This way, the automated switching between the front and rear lasers remain completely      *
* autonomous and independent from the Nav2 Docking Server.                                   *
*                                          -                                                 *
* This docking estimation procedure was redesigned to work with Nav2's Docking Server.       *
* More details can be found in the documentation:                                            *   
* https://github.com/open-navigation/opennav_docking/tree/humble                             *    
*********************************************************************************************/

// Include necessary headers for ROS2, and other dependencies.
#include "dock_pose_estimator.h" 
#include <functional>           
#include "tf2_eigen/tf2_eigen.hpp"

// Constructor implementation for the DockPoseEstimator class.
// This constructor uses a member initializer list to initialize the base class Node,
// this happens before the main body of the constructor is executed,
// which makes the DockPoseEstimator object initialization more efficient.
// The first member of the initializer list is the Node class constructor, 
// inherited from rclcpp::Node, it initializes the node with the name 
// "dock_pose_estimator_node" and passes the options provided.
// The second member initializes the filter buffer size to 10, 
// which is used for median filtering of the pose estimates.
DockPoseEstimator::DockPoseEstimator(const rclcpp::NodeOptions & options)

    /*
        Member Initializer List:
        ** - `Node("dock_pose_estimator_node", options)`: Initializes the base class Node.
        ** - `filter_buffer_size_(10)`: Initializes the filter buffer size to 10.
    */
    : Node("dock_pose_estimator_node", options),
      filter_buffer_size_(10)


{
    // Create a shared pointer for the TF2 buffer and listener.
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create shared pointers for the Point Clouds.
    cloud_pcl_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud_normals_ = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();

    // Declare and get necessary parameters.
    this->declareAndGetParameters();
    
    // Create ROS2 publishers.
    // ! The dock pose topic name needs to be named "/detected_dock_pose" to match the Nav2 Docking Server 
    // ! SimpleChargingDock plugin expectations.
    docking_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/detected_dock_pose", 10);
    //model_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/model_point_cloud", 10);
    scene_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scene_point_cloud", 10);

    // Create Subscriber.
    // Start with the front laser by default.
    this->useRearLaser();

    current_laser_is_front_ = false; // Start with the front laser.
    laser_sub_counter_ = 0; // Initialize the counter for failed detections.

    // Create a timer that calls checkAndSwitchLaser() every second.
    laser_switch_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&DockPoseEstimator::checkAndSwitchLaser, this));

    // Log a message indicating that the DockPoseEstimator node has been initialized.
    //RCLCPP_INFO(this->get_logger(), "Dock Pose Estimator node has been initialized.");
}

// Declares and loads the necessary parameters from the ROS2 node and 
// initializes the pattern_pose_estimation_ member with a shared pointer to a new
// PatternPoseEstimation object with these parameters.
void DockPoseEstimator::declareAndGetParameters()
{
    // Declare parameters with default values.
    // Attention: The parameters may not be set with these default values,
    // they are just declared here. That is why we retrieve them later.
    this->declare_parameter<double>("tran_thresh", 0.015);
    this->declare_parameter<double>("rot_thresh", 30.0);
    this->declare_parameter<double>("fitting_score_thresh", 0.03);
    this->declare_parameter<double>("discretization_step", 0.01);
    this->declare_parameter<double>("distance_threshold", 2.0);
    this->declare_parameter<std::string>("model_file", "file");
    this->declare_parameter<std::string>("front_laser_topic", "/nav_hokuyo_laser/front/scan");
    this->declare_parameter<std::string>("rear_laser_topic", "/nav_hokuyo_laser/rear/scan");

    // Get the current active parameter values.
    double tran_thresh = this->get_parameter("tran_thresh").as_double();
    double rot_thresh = this->get_parameter("rot_thresh").as_double();
    double fitting_score_thresh = this->get_parameter("fitting_score_thresh").as_double();
    double discretization_step = this->get_parameter("discretization_step").as_double();
    double distance_threshold = this->get_parameter("distance_threshold").as_double();
    std::string model_file = this->get_parameter("model_file").as_string();
    front_laser_topic_ = this->get_parameter("front_laser_topic").as_string();
    rear_laser_topic_ = this->get_parameter("rear_laser_topic").as_string();

    // Log the parameters for debugging.
    //RCLCPP_INFO(this->get_logger(), "tran_thresh: %f", tran_thresh);
    //RCLCPP_INFO(this->get_logger(), "rot_thresh: %f", rot_thresh);
    //RCLCPP_INFO(this->get_logger(), "fitting_score_thresh: %f", fitting_score_thresh);
    //RCLCPP_INFO(this->get_logger(), "discretization_step: %f", discretization_step);
    //RCLCPP_INFO(this->get_logger(), "distance_threshold: %f", distance_threshold);

    // Initialize the Point Pair Feature (PPF) registration model with the loaded parameters.
    // Provide the shared pointer to the private member `pattern_pose_estimation_`.
    pattern_pose_estimation_ = std::make_shared<PatternPoseEstimation>(
        rot_thresh, tran_thresh, fitting_score_thresh, discretization_step, distance_threshold,
        model_file);
}

// This method switches the laser subscription to the front laser topic.
// It resets the existing subscription and creates a new one for the specified topic.
void DockPoseEstimator::useFrontLaser()
{
    // Resetting the smart pointer effectively shuts down the old subscription.
    laser_sub_.reset();
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        front_laser_topic_, 10, std::bind(&DockPoseEstimator::laserCallback, this, std::placeholders::_1));
}

// This method switches the laser subscription to the rear laser topic.
// Similar to the front laser, it resets the existing subscription and creates a new one for the specified topic.
void DockPoseEstimator::useRearLaser()
{
    // Resetting the smart pointer effectively shuts down the old subscription.
    laser_sub_.reset();
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        rear_laser_topic_, 10, std::bind(&DockPoseEstimator::laserCallback, this, std::placeholders::_1));
}

// This method checks the laser subscription counter and switches the laser if necessary.
void DockPoseEstimator::checkAndSwitchLaser()
{
    // This function runs separately from the laser callback.
    // This is important to avoid unsafeties with the subscription management.
    // Because the laser publishing frequency is around 15Hz, the algorithm will switch lasers
    // every 2 seconds if no dock is detected.
    // A higher counter value before switching helps to avoid unnecessary switching
    // when the dock is not detected for a few scans due to noise or other temporary factors.
    // However, a 2 second interval is still a good compromise to ensure responsiveness.
    if (laser_sub_counter_ > 30) {
        if (!current_laser_is_front_) {
            //RCLCPP_INFO(this->get_logger(), "Switching to front laser.");
            this->useFrontLaser();
            current_laser_is_front_ = true;
        } else {
            //RCLCPP_INFO(this->get_logger(), "Switching to rear laser.");
            this->useRearLaser();
            current_laser_is_front_ = false;
        }
        laser_sub_counter_ = 0; // Reset after switching.
    }
}

// Main callback for processing laser data.
void DockPoseEstimator::laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> scan)
{

    RCLCPP_INFO(this->get_logger(), "Processing laser scan data...");

    // If the estimator is not enabled, do nothing.
    if (!enabled_) return;

    if (!pattern_pose_estimation_) {
        RCLCPP_ERROR(this->get_logger(), "ERROR: pattern_pose_estimation_ is null!");
        return;
    }
    if (!cloud_pcl_) {
        RCLCPP_ERROR(this->get_logger(), "ERROR: cloud_pcl_ is null!");
        return;
    }

    // Convert incoming ROS2 LaserScan message to PCL PointCloud.
    laser_geometry::LaserProjection projector_;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    projector_.projectLaser(*scan, cloud_msg);
    pcl::fromROSMsg(cloud_msg, *cloud_pcl_);
    cloud_normals_ = pattern_pose_estimation_->getPointNormal(cloud_pcl_);
    scene_cloud_pub_->publish(cloud_msg);

    if (!cloud_normals_) {
        RCLCPP_INFO(this->get_logger(), "getPointNormal() returned a null pointer. No valid points found.");
        laser_sub_counter_++; // Increment the counter on failure.
        return;
    }

    //RCLCPP_INFO(this->get_logger(), "Point cloud with normals has been created.");

    // Compute the transform using PPF.
    Eigen::Affine3d transformNN;
    try {
        transformNN = pattern_pose_estimation_->detect(cloud_normals_);
        laser_sub_counter_ = 0; // Reset on success.
    } catch (const std::exception &e) {
        RCLCPP_INFO(this->get_logger(), "No dock found in the current scan.");
        laser_sub_counter_++; // Increment the counter on failure.
        return;
    }

    //RCLCPP_INFO(this->get_logger(), "Transform computed successfully.");

    // Extract pose from transform.
    Eigen::Matrix3d rotation = transformNN.rotation();
    Eigen::Vector3d translation = transformNN.translation();
    Eigen::Vector3d ea = rotation.eulerAngles(2, 1, 0); // ZYX yaw, pitch, roll

    //RCLCPP_INFO(this->get_logger(), "Extracted pose: [x: %f, y: %f, z: %f, yaw: %f]",
                //translation[0], translation[1], translation[2], ea[0]);

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

    //RCLCPP_INFO(this->get_logger(), "Filtered pose: [x: %f, y: %f, z: %f, yaw: %f]",
                //x_filtered, y_filtered, z_filtered, yaw_filtered);

    // Reconstruct filtered transform.
    Eigen::Matrix3d m = Eigen::AngleAxisd(yaw_filtered, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Affine3d transformNNfiltered = Eigen::Affine3d::Identity();
    transformNNfiltered.translate(Eigen::Vector3d(x_filtered, y_filtered, z_filtered));
    transformNNfiltered.rotate(m);
    
    // Convert Eigen transform to PoseStamped message and publish the dock pose.
    dock_pose_.pose = tf2::toMsg(transformNNfiltered);
    dock_pose_.header = scan->header;
    docking_pub_->publish(dock_pose_);

    RCLCPP_INFO(this->get_logger(), "Dock pose published: [x: %f, y: %f, z: %f, yaw: %f]", 
    x_filtered, y_filtered, z_filtered, yaw_filtered);

    // TODO: Apply corrective transform for correct visualization of the final model point cloud.
    // For now, it is commented out for resource sparing purposes.

    // Publish model for visualization.
    //pattern_pose_estimation_->getOutputCloud()->header.frame_id = scan->header.frame_id;

    // PCL timestamp conversion.
    //pcl_conversions::toPCL(scan->header, pattern_pose_estimation_->getOutputCloud()->header);

    //pcl::PointCloud<pcl::PointNormal>::Ptr rotated_output_cloud(new pcl::PointCloud<pcl::PointNormal>);
    //Eigen::Affine3f visualization_transform = Eigen::Affine3f::Identity();
    //float theta_viz = M_PI / 2.0; // 90 degrees around the Y-axis
    //visualization_transform.rotate(Eigen::AngleAxisf(theta_viz, Eigen::Vector3f::UnitY()));
    
    //pcl::transformPointCloudWithNormals(*pattern_pose_estimation_->getOutputCloud(), *rotated_output_cloud, visualization_transform);

    // Create a ROS 2 message object.
    //sensor_msgs::msg::PointCloud2 output_cloud_msg;

    // Convert PCL data into the ROS 2 message.
    //pcl::toROSMsg(*rotated_output_cloud, output_cloud_msg);

    // Make sure the header is set correctly on the ROS message.
    //output_cloud_msg.header = scan->header;

    // Publish the ROS 2 message with the model point cloud.
    //model_pub_->publish(output_cloud_msg);

    //RCLCPP_INFO(this->get_logger(), "Laser data processed and dock pose published.");
}

// Getter for the dock pose, returns the current estimated docking pose.
geometry_msgs::msg::PoseStamped DockPoseEstimator::getPatternPose()
{
    return dock_pose_;
}

// This method computes the median of a deque of doubles.
double DockPoseEstimator::findMedian(std::deque<double> a)
{
    if (a.empty()) return 0.0;
    std::sort(a.begin(), a.end());
    return a.at(a.size() / 2);
}