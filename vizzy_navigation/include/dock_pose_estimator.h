/* * Copyright 2025, Joao Avelino, João Penha Lopes and João Zenário.
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
* https://github.com/ros-navigation/navigation2/tree/humble_main/nav2_docking                *
*********************************************************************************************/

#ifndef DOCK_POSE_ESTIMATOR_ROS2_H_
#define DOCK_POSE_ESTIMATOR_ROS2_H_

#include <deque>
#include <cmath> 
#include <string>
#include <memory>
#include <thread>
#include <limits>
#include <yaml-cpp/yaml.h>
#include <tf2_ros/buffer.h>
#include <pclomp/ndt_omp.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>   
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <pcl/filters/statistical_outlier_removal.h> 

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
     * @brief Callback for processing laser scan data.
     * This method is called whenever a new laser scan message is received on the subscribed topic.
     * It processes the laser scan data to estimate the docking pose and publishes
     * the estimated pose if the estimator is enabled and ready.
     * * Based on the received laser scan, the method performs the following steps:
     * 1. Converts the laser scan to a point cloud.
     * 2. Transforms the point cloud to the odom frame (initial guess calculation).
     * 3. Filters the point cloud to focus on the region of interest.
     * 4. Estimates the docking pose using the NDT algorithm.
     * 6. Publishes the estimated docking pose.
     * * @param scan A shared pointer to the received LaserScan message.
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
     * @brief Estimates the docking pose using NDT (Normal Distributions Transform).
     * This method takes two point clouds as input: the target cloud (scene) and the source cloud (model).
     * It uses the NDT algorithm to align the source cloud to the target cloud and estimates the transformation.
     * The estimated transformation is then used to create a PointCloud2 message representing the aligned model.
     * It also takes a LaserScan message as input to create an odometry-adjusted initial guess.
     * This method was designed to be agnostic of the rest of the DockPoseEstimator class, allowing for easier testing and algorithm swapping.
     * @param target_cloud The target point cloud (scene) to which the model will be aligned.
     * @param source_cloud The source point cloud (model) that will be aligned to the target.
     * @param scan_msg The LaserScan message containing the laser scan data.
     * @return A PointCloud2 message representing the aligned model after applying the estimated transformation.
     */
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::Matrix4f> estimateDockPose(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, std::shared_ptr<sensor_msgs::msg::LaserScan> scan_msg);

    /**
     * @brief Subscription object to the laser topic. Messages are received through here.
     */
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

    /**
     * @brief Publishers for the estimated docking pose.
     */
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> docking_pub_;

    /**
     * @brief Publisher for the scene point cloud.
     */
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> scene_cloud_pub_;

    /**
     * @brief Publisher for the model point cloud.
     */
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> model_cloud_pub_;

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
     * @brief Point cloud for the laser scan.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl_;

    /**
     * @brief Topic name for the rear laser scan.
     */
    std::string rear_laser_topic_;

    /**
     * @brief Point Cloud to hold the dock model (loaded from a SFD file).
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud_; 
    
    /**
     * @brief K-D Tree for nearest neighbor search.
     */
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_;

    /**
     * @brief Keep the last dock pose position published in the odometry frame.
     */
    geometry_msgs::msg::PoseStamped last_dock_pose_;

    /**
     * @brief Boolean to signal if the last dock pose position published in the odometry frame was saved.
     */
    bool last_dock_pose_saved_ = false;

    /**
     * @brief The alpha coefficient for the Exponential Moving Average (EMA) filter.
     * This parameter determines the weight given to the most recent measurement versus the historical average.
     * It must be in the range (0, 1].
     * - A higher value (closer to 1.0) makes the filter more responsive to new data but less smooth (more susceptible to noise).
     * - A lower value (closer to 0.0) makes the filter smoother/laggy but more robust to outliers.
     * A value of 0.5 implies that the new measurement contributes 50% to the current estimate, while the previous history contributes 50%.
     */
    double ema_alpha_ = 0.5;

    /**
     * @brief The smoothed X-coordinate of the dock pose in the laser frame.
     * This value is updated in every iteration using the EMA formula:
     * filtered_x = alpha * raw_x + (1 - alpha) * previous_filtered_x.
     * It is initialized with the first valid raw measurement.
     */
    double filtered_x_ = 0.0;

    /**
     * @brief The smoothed Y-coordinate of the dock pose in the laser frame.
     * This value represents the lateral offset of the dock relative to the laser.
     * Like the X-coordinate, it is updated using the linear EMA formula.
     */
    double filtered_y_ = 0.0;

    /**
     * @brief The smoothed Z-coordinate of the dock pose in the laser frame.
     * Although the docking is primarily 2D, we maintain a Z-estimate (usually near zero)
     * to prevent sudden vertical jumps if the NDT registration drifts slightly in the Z-axis.
     */
    double filtered_z_ = 0.0;

    /**
     * @brief The smoothed cosine component of the dock's yaw angle.
     * Since angles are cyclic (wrapping at PI/-PI), we cannot filter the raw yaw angle directly (averaging 179 and -179 yields 0, which is incorrect).
     * Instead, we decompose the angle into its vector components (sine and cosine), filter those components individually,
     * and then reconstruct the angle using atan2.
     */
    double filtered_cos_yaw_ = 0.0;

    /**
     * @brief The smoothed sine component of the dock's yaw angle.
     * Used in conjunction with filtered_cos_yaw_ to perform circular statistics smoothing on the orientation.
     * This component is updated using the standard EMA formula and then re-normalized to ensure the resulting vector remains on the unit circle.
     */
    double filtered_sin_yaw_ = 0.0;

    /**
     * @brief Flag indicating whether the EMA filter has been initialized.
     * The EMA filter is recursive and requires a previous state to function.
     * - If false: The filter accepts the current raw measurement as the initial state (seeding).
     * - If true: The filter applies the smoothing formula using the previous state.
     * This flag is reset to false whenever the node is activated or tracking is lost, ensuring the filter does not drag old/stale history into a new docking procedure.
     */
    bool ema_initialized_ = false;

    /**
     * @brief Counter stores the number of consecutive scan processing failures.
     * This variable is incremented when the NDT registration fails to converge or the fitness score
     * exceeds the acceptable threshold. It is used to trigger a reset of the motion tracking state,
     * forcing a re-initialization of the guess strategy, reverting to centroid calculation
     * if the estimator loses track of the dock for a prolonged period.
     */
    int consecutive_failures_;

    /**
     * @brief Parameter for the Region of Interest (ROI) filtering: Minimum X-coordinate.
     * Defines the near clipping plane in the laser frame. Points with x < roi_min_x_ are discarded.
     */
    double roi_min_x_;

    /**
     * @brief Parameter for the Region of Interest (ROI) filtering: Maximum X-coordinate.
     * Defines the far clipping plane in the laser frame. Points with x > roi_max_x_ are discarded.
     */
    double roi_max_x_;

    /**
     * @brief Parameter for the Region of Interest (ROI) filtering: Minimum Y-coordinate.
     * Defines the right lateral boundary in the laser frame. Points with y < roi_min_y_ are discarded.
     */
    double roi_min_y_;

    /**
     * @brief Parameter for the Region of Interest (ROI) filtering: Maximum Y-coordinate.
     * Defines the left lateral boundary in the laser frame. Points with y > roi_max_y_ are discarded.
     */
    double roi_max_y_;

    /**
     * @brief Parameter for NDT configuration: Grid Resolution.
     * Specifies the side length of the voxel grid used to model the environmental probability distribution.
     * A smaller resolution provides higher accuracy for detailed shapes but increases computational cost.
     */
    double ndt_resolution_;

    /**
     * @brief Parameter for NDT configuration: Step Size.
     * Defines the maximum step length allowed during the Newton-Raphson line search optimization.
     * This parameter influences the convergence speed and the likelihood of overshooting the local minimum.
     */
    double ndt_step_size_;

    /**
     * @brief Parameter to test the centroid-based initial guess strategy.
     * When set to true, the estimator will always use the centroid of the filtered point cloud
     * as the initial guess for NDT registration, ignoring any previous pose tracking.
     * This is useful for testing the robustness of the centroid initialization method.
     */
    bool force_centroid_guess_ = false;

    /**
     * @brief Calculated width of the dock model in meters.
     * This value is computed during parameter loading by analyzing the loaded model point cloud.
     * It is used to dynamically adjust the multi-start optimization offsets in the NDT alignment process,
     * ensuring that the offsets are geometrically relevant to the actual size of the dock.
     */
    float calculated_model_width_;

    /**
     * @brief Flag to enable or disable the use of Statistical Outlier Removal and Downsampling.
     * When set to true, the point cloud processing pipeline will include a Statistical Outlier Removal filter
     * followed by a Voxel Grid downsampling step. This improves noise hygiene and reduces the number of points
     * for faster NDT registration, at the cost of some detail loss.
     * When set to false, only ROI filtering will be applied to the point cloud.
     */
    bool use_statistical_outlier_removal_and_downsampling_ = true;
};

#endif // DOCK_POSE_ESTIMATOR_ROS2_H_