#include <pattern_pose_estimation.h>
#include <random>
#include <chrono>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;
using namespace std;

PatternPoseEstimation::PatternPoseEstimation(double rot_thresh_, double tran_thresh_, double fitting_score_thresh_, double discretization_step_, double distance_threshold_, std::string file_) :
	rot_thresh( (rot_thresh_ / 180.0) * double (M_PI)), // Convert degrees to radians.
	tran_thresh(tran_thresh_),
	fitting_score_thresh(fitting_score_thresh_),
	distance_threshold(distance_threshold_),
	normals (new pcl::PointCloud<pcl::Normal>()),
	normals_ (new pcl::PointCloud<pcl::Normal>()),
	point_cloud_ (new pcl::PointCloud<pcl::PointXYZ>()),
	tree (new pcl::search::KdTree<pcl::PointXYZ> ()),
	cloud_output_subsampled(new PointCloud<PointNormal>()),
	discretization_step(discretization_step_)
{
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
	cluster_tran_thresh=tran_thresh;
	cluster_rot_thresh=rot_thresh;
	loadModelFromMesh(file_);
	train(dense_cloud_models);
}

pcl::PointCloud<pcl::PointNormal>::Ptr PatternPoseEstimation::getPointNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
    // Log the starting number of points from the raw laser scan.
    std::cout << "Starting with " << point_cloud->size() << " points from the raw laser scan." << std::endl;

    // Clear previous data.
    normals->clear();
    normals_->clear();
    point_cloud_->clear();

    // Filter and downsample the point cloud to reduce noise and computation.
    // A large leaf size is used for Z to treat it as a 2D plane.
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(point_cloud);
    sor.setLeafSize(discretization_step, discretization_step, 1000.0f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    sor.filter(*filtered_cloud);

    // Log the number of points after voxel grid filtering.
    std::cout << "After VoxelGrid filter, " << filtered_cloud->size() << " points remain." << std::endl;

    // Estimate 2D Normals.
    pcl::PointCloud<pcl::Normal>::Ptr estimated_normals(new pcl::PointCloud<pcl::Normal>());
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    kdtree->setInputCloud(filtered_cloud);

    // Iterate through each point to find neighbors and estimate the normal.
    // The normal is perpendicular to the tangent of the curve.
    for (size_t i = 0; i < filtered_cloud->size(); ++i)
    {
        pcl::PointXYZ searchPoint = filtered_cloud->points[i];
        std::vector<int> pointIdxNKNSearch(5); // Use 5 neighbors for robust estimation.
        std::vector<float> pointNKNSquaredDistance(5);

        if (kdtree->nearestKSearch(searchPoint, 5, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            Eigen::Vector2d tangent_vector(0.0, 0.0);
            for (size_t j = 1; j < pointIdxNKNSearch.size(); ++j)
            {
                const pcl::PointXYZ& neighbor = filtered_cloud->points[pointIdxNKNSearch[j]];
                tangent_vector.x() += neighbor.x - searchPoint.x;
                tangent_vector.y() += neighbor.y - searchPoint.y;
            }

            Eigen::Vector2d normal_vector(-tangent_vector.y(), tangent_vector.x());
            if (normal_vector.norm() > 1e-6)
            {
                normal_vector.normalize();

                pcl::Normal normal_point;
                normal_point.normal_x = normal_vector.x();
                normal_point.normal_y = normal_vector.y();
                normal_point.normal_z = 0.0;
                estimated_normals->push_back(normal_point);
                point_cloud_->push_back(searchPoint);
            }
        }
    }

    // Log the number of points after normal estimation.
    std::cout << "After normal estimation, " << point_cloud_->size() << " valid points were found." << std::endl;

    // Filter points based on distance from origin.
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points_by_dist(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr filtered_normals_by_dist(new pcl::PointCloud<pcl::Normal>());
    for (size_t i = 0; i < point_cloud_->size(); ++i)
    {
        double dist = std::sqrt(std::pow(point_cloud_->points[i].x, 2) + std::pow(point_cloud_->points[i].y, 2));
        if (dist <= distance_threshold)
        {
            filtered_points_by_dist->push_back(point_cloud_->points[i]);
            filtered_normals_by_dist->push_back(estimated_normals->points[i]);
        }
    }

    // Log the number of points after distance filtering.
    std::cout << "After distance filter, " << filtered_points_by_dist->size() << " points remain." << std::endl;
    
    // Check if we have a valid point cloud and normals.
    if (filtered_points_by_dist->empty() || filtered_normals_by_dist->empty())
    {
        std::cerr << "No valid points found after filtering." << std::endl;
        return nullptr; // Return a null pointer if no valid points are found.
    }

    // Concatenate points and normals into a single PointNormal cloud.
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*filtered_points_by_dist, *filtered_normals_by_dist, *cloud_normals);

    // Log the final number of points.
    std::cout << "Final output point cloud has " << cloud_normals->size() << " points." << std::endl;

    return cloud_normals;
}

void PatternPoseEstimation::loadModelFromMesh(std::string file_name)
{
    std::cerr << "[DEBUG] Now entering loadModelFromMesh() function." << std::endl;

    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFileSTL(file_name, mesh) == -1)
    {
        std::cerr << "ERROR: Could not read mesh file: " << file_name << std::endl;
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr verts(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(mesh.cloud, *verts);

    // Get a point cloud with normals from the mesh.
    pcl::PointCloud<pcl::PointNormal>::Ptr model_cloud_with_normals = getPointNormal(verts);

    if (!model_cloud_with_normals) {
        std::cerr << "Error: Model point cloud with normals could not be created." << std::endl;
        return;
    }

    // Apply a rotation to the model to align its coordinate system with the laser scan.
    // Rotate 90 degrees around the Y-axis.
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    float theta = M_PI / 2.0; // 90 degrees in radians.
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));
    
    // Apply the transformation to both points and normals.
    pcl::transformPointCloudWithNormals(*model_cloud_with_normals, *model_cloud_with_normals, transform);
    
    dense_cloud_models.push_back(model_cloud_with_normals);
    train(dense_cloud_models);
    std::cerr << "[DEBUG] Exiting loadModelFromMesh() function." << std::endl;
}

int PatternPoseEstimation::train(std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> cloud_models_with_normals_)
{
	std::cerr << "[DEBUG] Now entering train() function." << std::endl;
	for (size_t model_i = 0; model_i < cloud_models_with_normals_.size (); ++model_i)
	{
		cloud_models_with_normals.push_back (cloud_models_with_normals_[model_i]);

		PointCloud<PPFSignature>::Ptr cloud_model_ppf (new PointCloud<PPFSignature> ());
		PPFEstimation<PointNormal, PointNormal, PPFSignature> ppf_estimator;

		ppf_estimator.setInputCloud (cloud_models_with_normals_[model_i]);
		ppf_estimator.setInputNormals (cloud_models_with_normals_[model_i]);
		ppf_estimator.compute (*cloud_model_ppf);

		PPFHashMapSearch::Ptr hashmap_search (new PPFHashMapSearch (rot_thresh, tran_thresh));
		hashmap_search->setInputFeatureCloud (cloud_model_ppf);
		hashmap_search_vector.push_back (hashmap_search);
	}
	std::cerr << "[DEBUG] Exiting train() function." << std::endl;
	return 0;
}	

Eigen::Affine3d PatternPoseEstimation::detect(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{

	// Block for debugging purposes.
	std::cerr << "[DEBUG] Now entering detect() function." << std::endl;
    if (cloud_models_with_normals.empty()) {
        std::cerr << "[ERROR] FATAL: No models were trained! The model vector is empty." << std::endl;
        throw std::runtime_error("No models available for detection.");
    } else {
        std::cerr << "[INFO] Number of trained models: " << cloud_models_with_normals.size() << std::endl;
        if (cloud_models_with_normals[0]) {
            std::cerr << "[INFO] Model 0 has " << cloud_models_with_normals[0]->points.size() << " points." << std::endl;
        } else {
            std::cerr << "[ERROR] FATAL: Model 0 is a null pointer!" << std::endl;
            throw std::runtime_error("Trained model is null.");
        }
    }

	std::vector<Eigen::Affine3d> transforms_;

	try
	{		
		for (size_t model_i = 0; model_i < hashmap_search_vector.size (); ++model_i)
		{
			if (!cloud_models_with_normals[model_i] || cloud_models_with_normals[model_i]->empty())
            {
                std::cerr << "[ERROR] Model cloud at index " << model_i << " is null or empty! Check if the model file was loaded correctly." << std::endl;
                throw std::runtime_error("Invalid model cloud provided to detect()");
            }

			std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
			PPFRegistration<PointNormal, PointNormal> ppf_registration;
			ppf_registration.setSceneReferencePointSamplingRate (1);
			ppf_registration.setPositionClusteringThreshold(cluster_tran_thresh);
			ppf_registration.setRotationClusteringThreshold(cluster_rot_thresh);
			ppf_registration.setSearchMethod(hashmap_search_vector[model_i]);
			ppf_registration.setInputSource(cloud_models_with_normals[model_i]);
			ppf_registration.setInputTarget(cloud_with_normals);
			
			Eigen::Matrix4f mat = ppf_registration.getFinalTransformation ();
			ppf_registration.align(*cloud_output_subsampled);

			if (cloud_output_subsampled->empty())
			{
				std::cerr << "[WARN] PPF alignment failed to produce a result for this model. Skipping." << std::endl;
				continue;
			}
			
			mat = ppf_registration.getFinalTransformation ();
			std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

			begin = std::chrono::steady_clock::now();
			Eigen::Matrix4f refined_transform=refine(cloud_output_subsampled,cloud_with_normals);
			end = std::chrono::steady_clock::now();
			mat=refined_transform*mat;

			transforms_.push_back(Eigen::Affine3d (mat.cast<double>()));
		}
	}
	catch(std::exception &e)
	{
		throw std::exception(e);
	}

	if (transforms_.empty())
    {
        throw std::runtime_error("Detection failed: No valid transforms were found.");
    }

	return transforms_[0];
}

Eigen::Matrix4f PatternPoseEstimation::refine(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_target)
{
	pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_target);
	pcl::PointCloud<pcl::PointNormal> cloud_output;
	icp.align(cloud_output);
	*cloud_output_subsampled=cloud_output;

	if(icp.getFitnessScore()>fitting_score_thresh)
	{
		//std::cout << "icp did not converge" << std::endl;
		throw std::runtime_error("icp did not converge");
	}
	else
	{
		//std::cout << "icp converged: "<< icp.getFitnessScore() << std::endl;		
	}
	
	return icp.getFinalTransformation();
}