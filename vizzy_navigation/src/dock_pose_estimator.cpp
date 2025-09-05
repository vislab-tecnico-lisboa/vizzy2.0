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
* 1. The dock_pose_estimator_node is implemented as a Lifecycle node, which allows for better*
* management of its state and resources externally.                                          *
* 2. When, in due processing, the robot reaches the staging pose during a docking procedure, *
* the dock_pose_estimator_node is activated by the lifecycle manager.                        *
* 3. The node then subscribes to the rear laser scan topic and starts processing the data to *
* estimate and publish the dock pose.                                                        *
*                                          -                                                 *
* This docking estimation procedure was redesigned to work with Nav2's Docking Server.       *
* More details can be found in the documentation:                                            *   
* https://github.com/open-navigation/opennav_docking/tree/humble                             *    
*********************************************************************************************/

#include "dock_pose_estimator.h" 

DockPoseEstimator::DockPoseEstimator(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("dock_pose_estimator_node", options),
      filter_buffer_size_(10)
{
    RCLCPP_INFO(this->get_logger(), "Dock Pose Estimator node has been created and is in an 'unconfigured' state.");
}

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
    this->declare_parameter<std::string>("rear_laser_topic", "/nav_hokuyo_laser/rear/scan");
    this->declare_parameter<bool>("publish_dock_point_cloud", false);

    // Get the current active parameter values.
    double tran_thresh = this->get_parameter("tran_thresh").as_double();
    double rot_thresh = this->get_parameter("rot_thresh").as_double();
    double fitting_score_thresh = this->get_parameter("fitting_score_thresh").as_double();
    double discretization_step = this->get_parameter("discretization_step").as_double();
    double distance_threshold = this->get_parameter("distance_threshold").as_double();
    std::string model_file = this->get_parameter("model_file").as_string();
    rear_laser_topic_ = this->get_parameter("rear_laser_topic").as_string();
    bool publish_dock_point_cloud_ = this->get_parameter("publish_dock_point_cloud").as_bool();

    // Log the parameters for debugging.
    RCLCPP_INFO(this->get_logger(), "--- Dock Pose Estimator Parameters ---");
    RCLCPP_INFO(this->get_logger(), "tran_thresh: %f", tran_thresh);
    RCLCPP_INFO(this->get_logger(), "rot_thresh: %f", rot_thresh);
    RCLCPP_INFO(this->get_logger(), "fitting_score_thresh: %f", fitting_score_thresh);
    RCLCPP_INFO(this->get_logger(), "discretization_step: %f", discretization_step);
    RCLCPP_INFO(this->get_logger(), "distance_threshold: %f", distance_threshold);
    RCLCPP_INFO(this->get_logger(), "publish_dock_point_cloud: %s", publish_dock_point_cloud_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "--- End of Parameters ---");

    // Initialize the Point Pair Feature (PPF) registration model with the loaded parameters.
    // Provide the shared pointer to the private member `pattern_pose_estimation_`.
    pattern_pose_estimation_ = std::make_shared<PatternPoseEstimation>(
        rot_thresh, tran_thresh, fitting_score_thresh, discretization_step, distance_threshold,
        model_file);
}

// --- LIFECYCLE CALLBACKS ---

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DockPoseEstimator::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Configuring Dock Pose Estimator...");

    // Initialize TF2 buffer and listener.
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize Point Clouds.
    cloud_pcl_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud_normals_ = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();

    // Declare and get necessary parameters.
    this->declareAndGetParameters();
    
    // Create the publishers. They are inactive until the node is activated.
    docking_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/detected_dock_pose", 10);
    // scene_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scene_point_cloud", 10);
    // model_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/model_point_cloud", 10);

    ready_ = true;
    RCLCPP_INFO(this->get_logger(), "Configuration successful. Node is now 'inactive'.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DockPoseEstimator::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Activating Dock Pose Estimator...");

    // Activate the publishers to allow them to publish messages.
    docking_pub_->on_activate();
    scene_cloud_pub_->on_activate();

    // Create the subscription ONLY to the rear laser. This starts the flow of data.
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        rear_laser_topic_, 10, std::bind(&DockPoseEstimator::laserCallback, this, std::placeholders::_1));
    
    enabled_ = true;

    RCLCPP_INFO(this->get_logger(), "Activation successful. Subscribed to '%s'. Node is now 'active'.", rear_laser_topic_.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DockPoseEstimator::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Deactivating Dock Pose Estimator...");
    
    enabled_ = false;

    // Destroy the subscription to stop receiving laser scans.
    laser_sub_.reset();

    // Deactivate publishers to prevent them from publishing.
    docking_pub_->on_deactivate();
    scene_cloud_pub_->on_deactivate();

    RCLCPP_INFO(this->get_logger(), "Deactivation successful. Node is now 'inactive'.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DockPoseEstimator::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Cleaning up Dock Pose Estimator...");

    // Release all resources.
    tf_listener_.reset();
    tf_buffer_.reset();
    docking_pub_.reset();
    scene_cloud_pub_.reset();
    laser_sub_.reset();
    pattern_pose_estimation_.reset();

    ready_ = false;
    RCLCPP_INFO(this->get_logger(), "Cleanup successful. Node is now 'unconfigured'.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DockPoseEstimator::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Dock Pose Estimator.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

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
        return;
    }

    //RCLCPP_INFO(this->get_logger(), "Point cloud with normals has been created.");

    // Compute the transform using PPF.
    Eigen::Affine3d transformNN;
    try {
        transformNN = pattern_pose_estimation_->detect(cloud_normals_);
    } catch (const std::exception &e) {
        RCLCPP_INFO(this->get_logger(), "No dock found in the current scan.");
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

    // If there is no need to publish the dock point cloud, we can return now.
    if (!publish_dock_point_cloud_) return;

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

    RCLCPP_INFO(this->get_logger(), "Laser data processed and dock pose published.");
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