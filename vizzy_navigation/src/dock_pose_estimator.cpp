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
* https://github.com/ros-navigation/navigation2/tree/humble_main/nav2_docking                *    
*********************************************************************************************/

#include "dock_pose_estimator.h" 

// Helper function to convert PCL point clouds to the Eigen::Matrix format TEASER++ needs.
Eigen::Matrix<double, 3, Eigen::Dynamic> convertPCLToEigen(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    Eigen::Matrix<double, 3, Eigen::Dynamic> matrix(3, cloud->points.size());
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        matrix(0, i) = cloud->points[i].x;
        matrix(1, i) = cloud->points[i].y;
        matrix(2, i) = cloud->points[i].z;
    }
    return matrix;
}

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
    this->declare_parameter<std::string>("model_file", "file");
    this->declare_parameter<std::string>("rear_laser_topic", "/nav_hokuyo_laser/rear/scan");

    // Get the current active parameter values.
    std::string model_file = this->get_parameter("model_file").as_string();
    rear_laser_topic_ = this->get_parameter("rear_laser_topic").as_string();
    
    RCLCPP_INFO(this->get_logger(), "--- Dock Pose Estimator Parameters ---");
    RCLCPP_INFO(this->get_logger(), "model_file: %s", model_file.c_str());
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

    RCLCPP_INFO(this->get_logger(), "Simplifying the model point cloud...");

    // Downsample the model using a VoxelGrid filter.
    pcl::PointCloud<pcl::PointXYZ>::Ptr simplified_model(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(model_cloud_);

    // Set the voxel size (in this case 2cm). 
    // This value controls the density of points in the simplified model.
    sor.setLeafSize(0.02f, 0.02f, 0.02f); 
    sor.filter(*simplified_model);

    // Update the model cloud to the simplified version.
    *model_cloud_ = *simplified_model;

    RCLCPP_INFO(this->get_logger(), "Applying centering transformation to the model...");

    // Define the centering transform using the negative of the barycenter values from MeshLab.
    // (The SDF model is not centered at the origin).
    Eigen::Affine3d centering_transform = Eigen::Affine3d::Identity();
    double center_x = 0.079505;
    double center_y = 0.005601;
    double center_z = 0.136650;
    centering_transform.translation() << -center_x, -center_y, -center_z;
    centering_transform.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));

    // Apply the transformation to the model point cloud.
    pcl::transformPointCloud(*model_cloud_, *model_cloud_, centering_transform);

    RCLCPP_INFO(this->get_logger(), "Successfully loaded and centered model with %zu points.", model_cloud_->size());

    // Configure the teaser++ solver parameters.
    teaser_params_.noise_bound = 0.05;
    teaser_params_.cbar2 = 1;
    teaser_params_.estimate_scaling = false;
    teaser_params_.rotation_max_iterations = 100;
    teaser_params_.rotation_gnc_factor = 1.4;
    teaser_params_.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    teaser_params_.inlier_selection_mode = teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_EXACT;
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
    model_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // Declare and get necessary parameters.
    this->declareAndGetParameters();
    
    // Create the publishers. They are inactive until the node is activated.
    docking_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/detected_dock_pose", 10);
    //scene_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scene_point_cloud", 10);

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
    //scene_cloud_pub_->on_activate();

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
    //scene_cloud_pub_->on_deactivate();

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
    //scene_cloud_pub_.reset();
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
        RCLCPP_ERROR(this->get_logger(), "ERROR: Dock model is not loaded!");
        return;
    }

    // Convert incoming LaserScan to PCL PointCloud.
    laser_geometry::LaserProjection projector_;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    projector_.projectLaser(*scan, cloud_msg);
    pcl::fromROSMsg(cloud_msg, *cloud_pcl_);
    //scene_cloud_pub_->publish(cloud_msg);

    if (cloud_pcl_->empty()) {
        RCLCPP_INFO(this->get_logger(), "Received an empty point cloud from laser scan.");
        return;
    }

    Eigen::Affine3d transformNN;
    try {
        // Convert PCL clouds to Eigen matrices.
        Eigen::Matrix<double, 3, Eigen::Dynamic> eigen_model = convertPCLToEigen(model_cloud_);
        Eigen::Matrix<double, 3, Eigen::Dynamic> eigen_scene = convertPCLToEigen(cloud_pcl_);

        // Find correspondences using a simple nearest-neighbor search.
        std::vector<std::pair<int, int>> correspondences;
        // Just update the input cloud (saves resources compared to creating one every iteration).
        kdtree_->setInputCloud(cloud_pcl_);

        for (int i = 0; i < eigen_model.cols(); ++i) {
            std::vector<int> a(1);
            std::vector<float> d(1);
            pcl::PointXYZ search_point;
            search_point.x = eigen_model(0, i);
            search_point.y = eigen_model(1, i);
            search_point.z = eigen_model(2, i);

            if (kdtree_->nearestKSearch(search_point, 1, a, d) > 0) {
                correspondences.push_back({i, a[0]}); // Match model point i to scene point a[0].
            }
        }
        
        if (correspondences.empty()) {
             throw std::runtime_error("Could not find any correspondences.");
        }

        // Create new matrices with only the matched points.
        // teaser++ expects two 3xN matrices where the columns are correctly ordered.
        Eigen::Matrix<double, 3, Eigen::Dynamic> matched_model(3, correspondences.size());
        Eigen::Matrix<double, 3, Eigen::Dynamic> matched_scene(3, correspondences.size());

        for (size_t i = 0; i < correspondences.size(); ++i) {
            int model_idx = correspondences[i].first;
            int scene_idx = correspondences[i].second;
            matched_model.col(i) = eigen_model.col(model_idx);
            matched_scene.col(i) = eigen_scene.col(scene_idx);
        }

        // Solve with teaser++.
        teaser::RobustRegistrationSolver solver(teaser_params_);
        solver.solve(matched_model, matched_scene);
        auto solution = solver.getSolution();
        
        // Reconstruct the final transform.
        transformNN = Eigen::Affine3d::Identity();
        transformNN.rotate(solution.rotation);
        transformNN.translate(solution.translation);

    } catch (const std::exception &e) {
        RCLCPP_INFO(this->get_logger(), "teaser++ failed: %s", e.what());
        return;
    }

    // Extract pose from transform.
    Eigen::Matrix3d rotation = transformNN.rotation();
    Eigen::Vector3d translation = transformNN.translation();
    Eigen::Vector3d ea = rotation.eulerAngles(2, 1, 0);

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

    double distance_to_dock = std::sqrt(x_filtered * x_filtered + y_filtered * y_filtered);
    RCLCPP_INFO(this->get_logger(), "Distance to dock: %f meters", distance_to_dock);

    // Reconstruct filtered transform.
    Eigen::Matrix3d m = Eigen::AngleAxisd(yaw_filtered, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Affine3d transformNNfiltered = Eigen::Affine3d::Identity();
    transformNNfiltered.translate(Eigen::Vector3d(x_filtered, y_filtered, z_filtered));
    transformNNfiltered.rotate(m);

    // Transform the pose to the odometry frame.
    // (nav2_docking expects the dock pose in the "odometry" frame).
    
    geometry_msgs::msg::PoseStamped pose_in_laser_frame;
    pose_in_laser_frame.header = scan->header;
    pose_in_laser_frame.pose = tf2::toMsg(transformNNfiltered);

    geometry_msgs::msg::PoseStamped pose_in_odom_frame;
    try {
      pose_in_odom_frame = tf_buffer_->transform(pose_in_laser_frame, "odometry");
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform dock pose: %s", ex.what());
      return;
    }

    // Correct the y value just a tiny bit (the dock model is not perfect).
    pose_in_odom_frame.pose.position.y -= 0.11;

    // Publish the estimated dock pose.
    docking_pub_->publish(pose_in_odom_frame);

    RCLCPP_INFO(this->get_logger(), "Dock pose published in odometry frame.");
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