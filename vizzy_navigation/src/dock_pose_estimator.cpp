/* * Copyright 2025, Joao Avelino, João Penha Lopes and João Zenário.
 * This code was originaly developped by João Avelino for ROS1 in 2019.
 * It was later refactored for ROS2 in 2025 by João Penha Lopes and João Zenário.
 * All rights reserved.
 */

/*********************************************************************************************
* Dock Pose Estimator C++ File for ROS2                                                      *
* -                                                                                          *
* This file contains the implementation of every method defined in the                       *
* DockPoseEstimator class header file.                                                       *
* It includes the constructor, parameter loading, laser scan processing,                     *
* median filtering, and publishing of the estimated docking pose.                            * 
* -                                                                                          *
* To simplify execution of the code and to integrate with the Nav2 ecoystem, the following   *
* approach was used to detect the dock pose:                                                 *
* 1. The dock_pose_estimator_node is implemented as a Lifecycle node, which allows for better*
* management of its state and resources externally.                                          *
* 2. When, in due processing, the robot reaches the staging pose during a docking procedure, *
* the dock_pose_estimator_node is activated by the lifecycle manager.                        *
* 3. The node then subscribes to the rear laser scan topic and starts processing the data to *
* estimate and publish the dock pose.                                                        *
* -                                                                                          *
* This docking estimation procedure was redesigned to work with Nav2's Docking Server.       *
* More details can be found in the documentation:                                            * 
* https://github.com/ros-navigation/navigation2/tree/humble_main/nav2_docking                *
* *******************************************************************************************/

#include "dock_pose_estimator.h" 
#include <pcl/common/centroid.h> // Added for robust initial guessing.

DockPoseEstimator::DockPoseEstimator(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("dock_pose_estimator_node", options),
      filter_buffer_size_(10),
      consecutive_failures_(0),       // Initialize failure counter to zero.
      last_dock_pose_saved_(false),   // Explicitly init boolean flags.
      ema_initialized_(false),
      force_centroid_guess_(false)    // Initialize the debug flag to false by default.
{
    RCLCPP_INFO(this->get_logger(), "Dock Pose Estimator node has been created and is in an 'unconfigured' state.");
}

void DockPoseEstimator::declareAndGetParameters()
{
    // Declare parameters with default values.
    this->declare_parameter<std::string>("model_file", "file");
    this->declare_parameter<std::string>("rear_laser_topic", "/nav_hokuyo_laser/rear/scan");

    // ROI Parameters: Defining the Region of Interest as parameters allows tuning.
    // without recompilation, preventing blind spots if the robot stages further away.
    this->declare_parameter<double>("roi_min_x", 0.1);
    this->declare_parameter<double>("roi_max_x", 2.0);
    this->declare_parameter<double>("roi_min_y", -0.5);
    this->declare_parameter<double>("roi_max_y", 0.5);

    // NDT Parameters: Exposing resolution and step size allows us to balance.
    // speed vs. accuracy. For docking, a finer resolution (5cm) is often required.
    this->declare_parameter<double>("ndt_resolution", 0.05);
    this->declare_parameter<double>("ndt_step_size", 0.1);
    this->declare_parameter<bool>("force_centroid_guess", false);
    this->declare_parameter<bool>("use_statistical_outlier_removal_and_downsampling", true);

    // Get the current active parameter values.
    std::string model_file = this->get_parameter("model_file").as_string();
    rear_laser_topic_ = this->get_parameter("rear_laser_topic").as_string();
    
    // Retrieve the numerical tuning parameters.
    roi_min_x_ = this->get_parameter("roi_min_x").as_double();
    roi_max_x_ = this->get_parameter("roi_max_x").as_double();
    roi_min_y_ = this->get_parameter("roi_min_y").as_double();
    roi_max_y_ = this->get_parameter("roi_max_y").as_double();
    ndt_resolution_ = this->get_parameter("ndt_resolution").as_double();
    ndt_step_size_ = this->get_parameter("ndt_step_size").as_double();
    force_centroid_guess_ = this->get_parameter("force_centroid_guess").as_bool();
    use_statistical_outlier_removal_and_downsampling_ = this->get_parameter("use_statistical_outlier_removal_and_downsampling").as_bool();

    RCLCPP_INFO(this->get_logger(), "--- Dock Pose Estimator Parameters ---");
    RCLCPP_INFO(this->get_logger(), "model_file: %s", model_file.c_str());
    RCLCPP_INFO(this->get_logger(), "ROI X: [%.2f, %.2f] Y: [%.2f, %.2f]", roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_);
    RCLCPP_INFO(this->get_logger(), "NDT: Res=%.3f Step=%.3f", ndt_resolution_, ndt_step_size_);
    RCLCPP_INFO(this->get_logger(), "--- End of Parameters ---");

    // Load the dock model from the STL file.
    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFileSTL(model_file, mesh) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not read mesh file: %s", model_file.c_str());
        return;
    }
    pcl::fromPCLPointCloud2(mesh.cloud, *model_cloud_);

    RCLCPP_INFO(this->get_logger(), "Successfully loaded model with %zu points.", model_cloud_->size());

    // ------------------------ MODEL SLICING BLOCK -----------------------------------------------

    // Create a 2D slice of the model for matching with the laser scan.
    // Since the model is 3D but vertically symmetric, there is no information regarding pitch and there will never be any roll value.
    // Due to this, we create a 2D slice of the model to match against the 2D laser scan without losing information.

    const float slice_thick = 0.05f;   // 5 cm thick band.
    if (model_cloud_->empty()) {
    RCLCPP_ERROR(this->get_logger(), "Model cloud empty before 2D slicing.");
    return;
    }

    // Find z min/max.
    float zmin = std::numeric_limits<float>::max();
    float zmax = std::numeric_limits<float>::lowest();
    for (const auto& p : model_cloud_->points) {
    zmin = std::min(zmin, p.z);
    zmax = std::max(zmax, p.z);
    }

    // Slide a window to find the densest band.
    const float step = slice_thick * 0.25f;  // 75% overlap.
    float best_z_center = 0.0f;
    size_t best_count = 0;

    for (float zc = zmin; zc <= zmax; zc += step) {
    const float z_lo = zc - slice_thick * 0.5f;
    const float z_hi = zc + slice_thick * 0.5f;
    size_t count = 0;
    for (const auto& p : model_cloud_->points) {
        if (p.z >= z_lo && p.z <= z_hi) ++count;
    }
    if (count > best_count) {
        best_count = count;
        best_z_center = zc;
    }
    }

    if (best_count == 0) {
    RCLCPP_ERROR(this->get_logger(), "Could not find a non-empty z-band for 2D slicing.");
    return;
    }

    // Collect points in the best band and project to z=0.
    pcl::PointCloud<pcl::PointXYZ>::Ptr model2d(new pcl::PointCloud<pcl::PointXYZ>());
    model2d->reserve(best_count);
    const float z_lo = best_z_center - slice_thick * 0.5f;
    const float z_hi = best_z_center + slice_thick * 0.5f;
    for (const auto& p : model_cloud_->points) {
    if (p.z >= z_lo && p.z <= z_hi) {
        model2d->push_back(pcl::PointXYZ{p.x, p.y, 0.0f});
    }
    }

    // Downsample for faster processing.
    pcl::VoxelGrid<pcl::PointXYZ> v;
    v.setInputCloud(model2d);
    v.setLeafSize(0.02f, 0.02f, 0.02f);

    model_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    v.filter(*model_cloud_);

    // Calculate model width for dynamic multi-start offsetting.
    // Instead of hardcoding a value, we measure the bounding box of the model.
    // This allows the multi-hypothesis strategy to scale automatically to different dock sizes.
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*model_cloud_, min_pt, max_pt);
    calculated_model_width_ = max_pt.y - min_pt.y;
    RCLCPP_INFO(this->get_logger(),
                "Calculated dock model width: %.3f meters.",
                calculated_model_width_);

    // ----------------------------------------------------------------------------------------------------

    RCLCPP_INFO(this->get_logger(),
                "Auto 2D dock template: z∈[%.3f, %.3f], %zu points.",
                z_lo, z_hi, model_cloud_->size());
}

// LIFECYCLE CALLBACKS

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DockPoseEstimator::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Configuring Dock Pose Estimator...");

    // Initialize TF2 buffer and listener.
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize Point Clouds.
    cloud_pcl_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    model_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // Declare and get necessary parameters.
    this->declareAndGetParameters();
    
    // Create the publishers. They are inactive until the node is activated.
    docking_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/detected_dock_pose", 10);
    scene_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_scene_point_cloud", 10);
    model_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/dock_model_point_cloud", 10);

    // Initialize the KD-Tree for nearest neighbor search.
    kdtree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

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
    model_cloud_pub_->on_activate();

    // Create the subscription ONLY to the rear laser. This starts the flow of data.
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        rear_laser_topic_, 10, std::bind(&DockPoseEstimator::laserCallback, this, std::placeholders::_1));
    
    enabled_ = true;

    // Reset filtering state on activation to prevent stale data from influencing the initial output.
    consecutive_failures_ = 0;
    last_dock_pose_saved_ = false;
    ema_initialized_ = false;

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
    model_cloud_pub_->on_deactivate();

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
    model_cloud_pub_.reset();
    laser_sub_.reset();

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
    if (!enabled_) return;

    if (!model_cloud_ || model_cloud_->empty()) {
        RCLCPP_ERROR(get_logger(), "ERROR: Dock model is not loaded!");
        return;
    }

    RCLCPP_INFO(get_logger(), "---------------------");
    RCLCPP_INFO(get_logger(), "PROCESSING LASER SCAN");
    RCLCPP_INFO(get_logger(), "---------------------");

    // LaserScan -> PCL (laser frame).
    laser_geometry::LaserProjection projector_;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    projector_.projectLaser(*scan, cloud_msg);
    pcl::fromROSMsg(cloud_msg, *cloud_pcl_);
    if (cloud_pcl_->empty()) {
        RCLCPP_INFO(get_logger(), "Received an empty point cloud from laser scan.");
        return;
    }

    // Flatten the Z-axis immediately.
    // Lidar scans can have small Z variations due to sensor tilt or noise.
    // Since we are doing 2D docking, these variations only hurt NDT convergence.
    // We enforce 2D by zeroing Z, ensuring a pure SE(2) problem.
    for (auto& p : cloud_pcl_->points) {
        p.z = 0.0f;
    }

    // ------------------------ ROI FILTERING BLOCK -----------------------------------------------

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xy(new pcl::PointCloud<pcl::PointXYZ>());
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_pcl_);
        pass.setFilterFieldName("x");

        // Using the parameterized ROI values instead of hardcoded numbers to support different staging distances.
        pass.setFilterLimits(roi_min_x_, roi_max_x_);
        pass.filter(*cloud_filtered_x);
    }
    if (cloud_filtered_x->empty()) {
        RCLCPP_INFO(get_logger(), "Point cloud empty after X-axis ROI filtering.");
        return;
    }
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_filtered_x);
        pass.setFilterFieldName("y");

        // Using the parameterized ROI values here as well.
        pass.setFilterLimits(roi_min_y_, roi_max_y_);
        pass.filter(*cloud_filtered_xy);
    }

    // Check if the ROI contains enough points to form a valid geometric shape.
    // If there are too few points (e.g., < 20 out of the 87), NDT will likely produce garbage results.
    if (cloud_filtered_xy->size() < 20) {
        RCLCPP_INFO(get_logger(), "Point cloud has too few points (%zu) after Y-axis ROI filtering. Skipping.", cloud_filtered_xy->size());
        return;
    }

    // Statistical Outlier Removal (Hygiene).
    // Remove "ghost points" or mixed-pixel noise that often occur at edges of objects.
    // This dramatically stabilizes NDT by providing a clean input.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_denoised(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_filtered_xy);
    sor.setMeanK(10);             // Analyze 10 neighbors.
    sor.setStddevMulThresh(1.0);  // Points > 1.0 sigma away are noise.
    sor.filter(*cloud_denoised);

    if (cloud_denoised->size() < 10) {
        RCLCPP_INFO(get_logger(), "Too few points after outlier removal. Skipping.");
        return;
    }

    // Voxel Grid Downsampling on Target Cloud.
    // If the robot is close, we might have thousands of points, slowing down NDT.
    // We downsample the scan to a consistent density (e.g. 0.5cm) to guarantee real-time performance.
    // TODO: I am trying to choose a leaf size that does not reduce the number of points above a certain distance to the dock.
    // TODO: This way, only dense scans get downsampled, while sparse scans (far away) remain intact.
    // TODO: Still need to test this in real scenarios.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(cloud_denoised);
    vox.setLeafSize(0.005f, 0.005f, 0.005f); // 0.5cm leaf size.
    vox.filter(*cloud_downsampled);

    // Publish filtered scene cloud (visualize the cleaned data).
    {
        sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(*cloud_downsampled, filtered_cloud_msg);
        filtered_cloud_msg.header = scan->header;
        scene_cloud_pub_->publish(filtered_cloud_msg);
    }

    RCLCPP_INFO(get_logger(), "scene points: %zu (raw) -> %zu (denoised) -> %zu (downsampled)",
        cloud_filtered_xy->size(),
        cloud_denoised->size(),
        cloud_downsampled->size());

    // Calculate the percentage of points removed during denoising for logging.
    float denoising_removal_ratio = 100.0f * (1.0f - static_cast<float>(cloud_denoised->size()) / static_cast<float>(cloud_filtered_xy->size()));
    RCLCPP_INFO(get_logger(), "Denoising removed %.2f%% of points.", denoising_removal_ratio);

    // Calculate the percentage of points removed during downsampling for logging.
    float downsampling_removal_ratio = 100.0f * (1.0f - static_cast<float>(cloud_downsampled->size()) / static_cast<float>(cloud_denoised->size()));
    RCLCPP_INFO(get_logger(), "Downsampling removed %.2f%% of points.", downsampling_removal_ratio);

    RCLCPP_INFO(get_logger(), "scene points: %zu first=[%.3f %.3f %.3f] header.frame=%s",
        cloud_downsampled->points.size(),
        cloud_downsampled->points[0].x,
        cloud_downsampled->points[0].y,
        cloud_downsampled->points[0].z,
        scan->header.frame_id.c_str());

    // ----------------------------------------------------------------------------------------------------

    // ------------------------ ESTIMATION FUNCTION CALL  -------------------------------------------------

    // Start the timer to measure the performance of the NDT registration.
    auto start_time = std::chrono::high_resolution_clock::now();

    // Log the use_statistical_outlier_removal_and_downsampling_ flag status.
    RCLCPP_INFO(get_logger(), "use_statistical_outlier_removal_and_downsampling_ flag is set to: %s",
                use_statistical_outlier_removal_and_downsampling_ ? "TRUE" : "FALSE");

    // Pass the fully optimized (denoised + downsampled) cloud to the estimator or the raw one based on the flag.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_use = cloud_downsampled;
    if (!use_statistical_outlier_removal_and_downsampling_) {
        cloud_to_use = cloud_filtered_xy;
        // Log that we are skipping the extra processing.
        RCLCPP_INFO(get_logger(), " !!! Skipping Statistical Outlier Removal and Downsampling as per configuration. !!! ");
    }
    auto [aligned_model, T_model_in_laser] = estimateDockPose(cloud_to_use, model_cloud_, scan);

    // Stop the timer and calculate the duration in milliseconds.
    auto end_time = std::chrono::high_resolution_clock::now();
    double execution_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    // Log the execution time for performance monitoring.
    RCLCPP_INFO(get_logger(), "Dock Pose Estimation took: %.3f ms", execution_time_ms);

    // ----------------------------------------------------------------------------------------------------

    // ------------------------ FAILURE HANDLING & TRACKING RESET -----------------------------------------

    // Validate the estimator output. If the estimator returns a null pointer, it means
    // convergence failed or the fitness score was too high.
    if (!aligned_model) {
        consecutive_failures_++;
        RCLCPP_WARN(get_logger(), "!!! Dock pose estimation FAILED. Failure count: %d !!!", consecutive_failures_);

        // If we lose tracking for too many consecutive frames, the previous 'last_dock_pose_' 
        // is likely too stale to be a valid initial guess. We reset the tracking state 
        // to force a fresh global initialization (using the centroid) on the next successful scan.
        const int MAX_FAILURES_BEFORE_RESET = 10;
        if (consecutive_failures_ > MAX_FAILURES_BEFORE_RESET) {
            RCLCPP_WARN(get_logger(), "Lost tracking for >%d frames. Resetting initial guess logic.", MAX_FAILURES_BEFORE_RESET);
            last_dock_pose_saved_ = false; 
            consecutive_failures_ = 0;

            // We also reset EMA to avoid dragging old filter values into the new detection.
            ema_initialized_ = false;
        }
        return;
    }

    // If estimation was successful, we reset the failure counter.
    consecutive_failures_ = 0;

    if (aligned_model->points.size() < 2) {
        RCLCPP_WARN(get_logger(), "Dock pose estimation returned insufficient points (%zu). Skipping this scan.", aligned_model->points.size());
        return;
    }

    // ------------------------ EMA ESTIMATION FILTERING BLOCK  -------------------------------------------

    // Raw pose from NDT result.
    double raw_x = static_cast<double>(T_model_in_laser(0,3));
    double raw_y = static_cast<double>(T_model_in_laser(1,3));
    double raw_z = 0.0;
    double raw_yaw = std::atan2(static_cast<double>(T_model_in_laser(1,0)),
                                static_cast<double>(T_model_in_laser(0,0)));

    // EMA initialization on first valid estimate.
    if (!ema_initialized_) {
        filtered_x_ = raw_x;
        filtered_y_ = raw_y;
        filtered_z_ = raw_z;
        filtered_cos_yaw_ = std::cos(raw_yaw);
        filtered_sin_yaw_ = std::sin(raw_yaw);
        ema_initialized_ = true;
    } else {
        // Larger alpha -> more weight to new sample -> more responsive.
        const double a = ema_alpha_;

        // Linear EMA for position.
        filtered_x_ = a * raw_x + (1.0 - a) * filtered_x_;
        filtered_y_ = a * raw_y + (1.0 - a) * filtered_y_;
        filtered_z_ = a * raw_z + (1.0 - a) * filtered_z_;

        // Circular EMA for yaw.
        filtered_cos_yaw_ = a * std::cos(raw_yaw) + (1.0 - a) * filtered_cos_yaw_;
        filtered_sin_yaw_ = a * std::sin(raw_yaw) + (1.0 - a) * filtered_sin_yaw_;

        // Renormalize occasionally to avoid drift.
        const double norm = std::hypot(filtered_cos_yaw_, filtered_sin_yaw_);
        if (norm > 1e-9) {
            filtered_cos_yaw_ /= norm;
            filtered_sin_yaw_ /= norm;
        }
    }

    // Compose the filtered pose to publish.
    double fyaw = std::atan2(filtered_sin_yaw_, filtered_cos_yaw_);
    geometry_msgs::msg::PoseStamped inst_laser;
    inst_laser.header = scan->header;  // laser frame.
    inst_laser.pose.position.x = static_cast<float>(filtered_x_);
    inst_laser.pose.position.y = static_cast<float>(filtered_y_);
    inst_laser.pose.position.z = static_cast<float>(filtered_z_);

    // Move the dock pose slightly to the right for correction purposes regarding the model used.
    float lateral_offset = -0.02f; // 2cm to the right.
    inst_laser.pose.position.y = static_cast<float>(filtered_y_) + lateral_offset;

    tf2::Quaternion q; q.setRPY(0.0, 0.0, fyaw);
    inst_laser.pose.orientation = tf2::toMsg(q);

    // Log difference between raw and filtered (position in meters, yaw in degrees).
    double yaw_diff = std::atan2(std::sin(fyaw - raw_yaw), std::cos(fyaw - raw_yaw));
    RCLCPP_INFO(get_logger(), "Filter delta: Δx=%.4f m Δy=%.4f m Δyaw=%.2f°",
                 filtered_x_ - raw_x, filtered_y_ - raw_y, yaw_diff * 180.0 / M_PI);

    // ------------------------------- PUBLISHING BLOCK -----------------------------------------------

    // Transform to odom at scan time.
    geometry_msgs::msg::PoseStamped inst_odom;
    if (!tf_buffer_->canTransform("odometry", inst_laser.header.frame_id, inst_laser.header.stamp,
                                tf2::durationFromSec(0.2))) {
        RCLCPP_WARN(get_logger(), "TF not available at scan time; skipping this measurement.");
        return;
    }
    try {
        inst_odom = tf_buffer_->transform(inst_laser, "odometry", tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "Could not transform to 'odometry': %s", ex.what());
        return;
    }

    // Save last pose for next initial guess.
    last_dock_pose_ = inst_odom;       // store in odom frame.
    last_dock_pose_saved_ = true;

    // Publish in ODOM.
    docking_pub_->publish(inst_odom);

    // ----------------------------------------------------------------------------------------------------

    // ------------------------ MODEL VISUALIZATION BLOCK ------------------------------------------

    // Model cloud viz at estimated pose (laser frame).
    if (model_cloud_pub_->is_activated()) {
        sensor_msgs::msg::PointCloud2 model_viz_msg;
        pcl::toROSMsg(*aligned_model, model_viz_msg);
        model_viz_msg.header = scan->header;
        model_cloud_pub_->publish(model_viz_msg);
    }

    // ---------------------------------------------------------------------------------------------
}

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::Matrix4f> DockPoseEstimator::estimateDockPose(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, std::shared_ptr<sensor_msgs::msg::LaserScan> scan)
{
    // Start timer for average execution time measurement.
    auto start_debug = std::chrono::high_resolution_clock::now();

    // NDT parameters.
    const float resolution = static_cast<float>(ndt_resolution_); 
    const int max_iter = 35;
    const double epsilon = 0.0005;
    const double step = ndt_step_size_;

    Eigen::Matrix4f base_guess = Eigen::Matrix4f::Identity();

    // Log the force_centroid_guess_ parameter state.
    RCLCPP_INFO(get_logger(), "force_centroid_guess_ parameter is set to: %s", force_centroid_guess_ ? "TRUE" : "FALSE");

    // We only attempt to use the previous pose for tracking if two conditions are met:
    // 1. A valid previous pose exists (last_dock_pose_saved_ is true).
    // 2. The 'force_centroid_guess_' debugging parameter is FALSE. 
    // If 'force_centroid_guess_' is true, we intentionally skip this block to test the fallback logic.
    if (last_dock_pose_saved_ && !force_centroid_guess_) {
        // Transform last odom pose into current laser frame (scan->header.frame_id) at scan time.
        // This will allow the initial guess to account for robot movement since last estimate.
        try {
            // Get predicted last dock pose in laser frame.
            auto pred_in_laser = tf_buffer_->transform(last_dock_pose_, scan->header.frame_id,
                                                    tf2::durationFromSec(0.1));

            RCLCPP_INFO(get_logger(), "pred_in_laser header: frame=%s sec=%u nsec=%u",
                         pred_in_laser.header.frame_id.c_str(),
                         pred_in_laser.header.stamp.sec,
                         pred_in_laser.header.stamp.nanosec);
            RCLCPP_INFO(get_logger(), "pred_in_laser pose: x=%.3f y=%.3f z=%.3f",
                         pred_in_laser.pose.position.x,
                         pred_in_laser.pose.position.y,
                         pred_in_laser.pose.position.z);

            // Build initial guess from predicted pose.
            double rr, rp, ry;
            tf2::Quaternion q(pred_in_laser.pose.orientation.x,
                            pred_in_laser.pose.orientation.y,
                            pred_in_laser.pose.orientation.z,
                            pred_in_laser.pose.orientation.w);
            tf2::Matrix3x3(q).getRPY(rr, rp, ry);
            base_guess = Eigen::Matrix4f::Identity();
            base_guess(0,3) = static_cast<float>(pred_in_laser.pose.position.x);
            base_guess(1,3) = static_cast<float>(pred_in_laser.pose.position.y);
            Eigen::Matrix3f R;
            R << std::cos(ry), -std::sin(ry), 0,
                std::sin(ry),  std::cos(ry), 0,
                            0,              0, 1;
            base_guess.block<3,3>(0,0) = R;

            RCLCPP_INFO(get_logger(), "Using TF-based initial guess: x=%.2f y=%.2f yaw=%.1f°",
                        base_guess(0,3), base_guess(1,3), ry * 180.0f/M_PI);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "TF transform for guess failed: %s — using fallback guess.", ex.what());
            // If the TF lookup fails, we invalidate the saved pose so the code falls through 
            // to the centroid-based guess logic below.
            last_dock_pose_saved_ = false;
        }
    } 
    
    // We calculate the centroid-based guess if:
    // 1. No previous tracking data exists (!last_dock_pose_saved_).
    // OR
    // 2. The 'force_centroid_guess_' parameter is set to TRUE, specifically overriding history.
    if (!last_dock_pose_saved_ || force_centroid_guess_) {
        // Instead of hardcoding an initial guess, we calculate the centroid of the 
        // currently filtered cloud. This provides a much more robust starting point 
        // regardless of whether the dock is at 0.5m or 2.0m.
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*target_cloud, centroid);
        
        base_guess(0,3) = centroid[0]; 
        base_guess(1,3) = centroid[1];
        
        // We keep rotation as identity (0 degrees) since we lack orientation info without tracking.
        RCLCPP_INFO(get_logger(), "Initializing guess at cloud centroid (Tracking: %s): x=%.2f y=%.2f", 
                    last_dock_pose_saved_ ? "FORCED OFF" : "UNAVAILABLE",
                    base_guess(0,3), base_guess(1,3));
    }

    // Use OMP NDT for speed (multi-threaded).
    const int threads = std::max<int>(1, static_cast<int>(std::thread::hardware_concurrency()) - 1);
    
    // Build NDT object and set parameters.
    pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setNumThreads(threads);
    ndt.setTransformationEpsilon(epsilon);
    ndt.setStepSize(step);
    ndt.setResolution(resolution);
    ndt.setMaximumIterations(max_iter);
    ndt.setInputTarget(target_cloud);
    ndt.setInputSource(source_cloud);

    // Multi-Start Optimization Strategy:
    // We try the base guess, plus offsets to the left and right. 
    // This handles cases where the initial guess is stuck in a local minimum (one V matched to the other).
    // The offset retrieven from calculated_model_width_ is dynamic based on the model size.
    float lateral_offset = calculated_model_width_ * 0.5f; // Half the model width.
    RCLCPP_INFO(get_logger(), "Using lateral offset of %.3f meters for multi-start NDT.", lateral_offset);
    // By checking multiple hypotheses, we avoid the "WW" mismatch problem.
    std::vector<float> y_offsets = {0.0f, lateral_offset, -lateral_offset}; 
    
    double best_fitness = std::numeric_limits<double>::max();
    Eigen::Matrix4f best_transform = Eigen::Matrix4f::Identity();
    bool any_converged = false;

    for (float offset : y_offsets) {
        // Create a new guess based on the calculated base guess.
        Eigen::Matrix4f candidate_guess = base_guess;
        
        // Apply lateral offset in the candidate's local Y frame (scan frame).
        candidate_guess(1, 3) += offset;

        // Perform alignment.
        pcl::PointCloud<pcl::PointXYZ> output_cloud_unused;
        ndt.align(output_cloud_unused, candidate_guess);

        // Evaluate the result.
        if (ndt.hasConverged()) {
            double score = ndt.getFitnessScore();
            // NDT fitness score is Euclidean distance (lower is better).
            if (score < best_fitness) {
                best_fitness = score;
                best_transform = ndt.getFinalTransformation();
                any_converged = true;
            }
        }
    }

    // Check if any of the hypotheses converged.
    if (!any_converged) {
        RCLCPP_INFO(get_logger(), "NDT did not converge on any hypothesis.");
        return std::pair(nullptr, Eigen::Matrix4f::Identity());
    }

    // Get final transformation matrix from the best hypothesis.
    Eigen::Matrix4f T_model_in_laser = best_transform;

    // Transform source model with final transform.
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_model(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*source_cloud, *transformed_model, T_model_in_laser);

    // Extract 2D yaw and translation for logging.
    Eigen::Matrix3f R = T_model_in_laser.block<3,3>(0,0);
    float yaw_laser = std::atan2(R(1,0), R(0,0));
    Eigen::Vector3f t = T_model_in_laser.block<3,1>(0,3);

    RCLCPP_INFO(get_logger(), "NDT Best Fitness=%.6f  t=[%.3f %.3f] yaw=%.1f°  (threads=%d)",
                best_fitness, t.x(), t.y(), yaw_laser * 180.0f/M_PI, threads);

    // Threshold the fitness score to reject bad alignments.
    const double fitness_threshold = 0.001;
    if (best_fitness > fitness_threshold) {
        RCLCPP_WARN(get_logger(), "NDT fitness score %.6f exceeds threshold %.2f; rejecting estimate.", best_fitness, fitness_threshold);
        return std::pair(nullptr, Eigen::Matrix4f::Identity());
    }

    // Sanity check on Z-axis: Since we are performing 2D docking, a valid transformation
    // should not have a significant Z component.
    if (std::abs(T_model_in_laser(2,3)) > 0.1) {
        RCLCPP_WARN(get_logger(), "Estimated pose has abnormal Z-offset (%.2fm). Rejecting.", T_model_in_laser(2,3));
        return std::pair(nullptr, Eigen::Matrix4f::Identity());
    }

    Eigen::Matrix3f Rdbg = T_model_in_laser.block<3,3>(0,0);
    double raw_yaw_dbg = std::atan2(Rdbg(1,0), Rdbg(0,0));
    RCLCPP_INFO(get_logger(), "NDT raw transform: tx=%.3f ty=%.3f yaw=%.3fdeg fitness=%.6f",
                T_model_in_laser(0,3), T_model_in_laser(1,3), raw_yaw_dbg * 180.0/M_PI, best_fitness);

    // Average Time and Fitness logging (every 10 scans)
    // We compute this right before returning a SUCCESSFUL estimation.
    auto end_debug = std::chrono::high_resolution_clock::now();
    double dur_ms = std::chrono::duration<double, std::milli>(end_debug - start_debug).count();
    
    // Static variables to maintain state between function calls without modifying header
    static int stats_count = 0;
    static double stats_time_sum = 0.0;
    static double stats_fitness_sum = 0.0;
    
    stats_count++;
    stats_time_sum += dur_ms;
    stats_fitness_sum += best_fitness;
    
    if (stats_count >= 10) {
        RCLCPP_INFO(get_logger(), "--- [10-SCAN AVG] Time: %.3f ms | Fitness: %.6f ---", 
                    stats_time_sum / 10.0, stats_fitness_sum / 10.0);
        stats_count = 0;
        stats_time_sum = 0.0;
        stats_fitness_sum = 0.0;
    }

    return std::pair(transformed_model, T_model_in_laser);
}