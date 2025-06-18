/*
 * LICENSE: https://github.com/utexas-bwi/segbot/blob/devel/LICENSE
 */

#ifndef VIZZY_FOOTPRINT_FILTER_HPP_
#define VIZZY_FOOTPRINT_FILTER_HPP_

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "filters/filter_base.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>

namespace vizzy_sensors
{
  class VizzyFootprintFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
  {
    public:
      std::vector<geometry_msgs::msg::Point32> footprint_polygon_;
      std::string footprint_frame_;

      // TF2 and Clock members.
      rclcpp::Clock::SharedPtr clock_;
      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

      bool configure() override {
        // Create our own clock instance.
        clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

        // Initialize TF2 buffer and listener, passing the new clock.
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Get footprint_frame parameter.
        if (!getParam("footprint_frame", footprint_frame_) || footprint_frame_.empty()) {
            RCLCPP_WARN(this->logging_interface_->get_logger(), "Parameter 'footprint_frame' not provided. Footprint will be assumed to be in the sensor's frame.");
        }

        // Get and parse the footprint parameter list.
        this->params_interface_->declare_parameter("footprint", rclcpp::ParameterValue(std::vector<double>()));
        rclcpp::Parameter footprint_param = this->params_interface_->get_parameter("footprint");
        std::vector<double> footprint_vector = footprint_param.as_double_array();

        if (footprint_vector.size() < 6 || footprint_vector.size() % 2 != 0) {
            RCLCPP_FATAL(this->logging_interface_->get_logger(), "Footprint parameter must be an even list of at least 6 values (3 x,y pairs).");
            return false;
        }

        for (size_t i = 0; i < footprint_vector.size(); i += 2) {
            geometry_msgs::msg::Point32 p;
            p.x = footprint_vector[i];
            p.y = footprint_vector[i+1];
            p.z = 0.0;
            footprint_polygon_.push_back(p);
        }

        RCLCPP_INFO(this->logging_interface_->get_logger(), "VizzyFootprintFilter configured successfully.");
        return true;
      }

      virtual ~VizzyFootprintFilter() {}

      bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan) override {
        filtered_scan = input_scan;
        std::vector<geometry_msgs::msg::Point32> transformed_footprint = footprint_polygon_;
        
        if (!footprint_frame_.empty()) {
            try {
                geometry_msgs::msg::TransformStamped transform;
                transform = tf_buffer_->lookupTransform(input_scan.header.frame_id, footprint_frame_, tf2::TimePointZero);
                for (auto& point : transformed_footprint) {
                    tf2::doTransform(point, point, transform);
                }
            } catch (const tf2::TransformException & ex) {
                RCLCPP_ERROR(
                    this->logging_interface_->get_logger(),
                    "Could not transform footprint from %s to %s. Filter will not be applied. Error: %s",
                    footprint_frame_.c_str(), input_scan.header.frame_id.c_str(), ex.what());
                return true;
            }
        }

        for(unsigned int i = 0; i < input_scan.ranges.size(); ++i){
            float range = input_scan.ranges[i];
            if (range >= input_scan.range_min && range <= input_scan.range_max) {
                double angle = input_scan.angle_min + i * input_scan.angle_increment;
                float test_x = range * cos(angle);
                float test_y = range * sin(angle);

                int nvert = transformed_footprint.size();
                bool in_footprint = false;
                for (int j = 0, k = nvert - 1; j < nvert; k = j++) {
                    if (((transformed_footprint[j].y > test_y) != (transformed_footprint[k].y > test_y)) &&
                        (test_x < (transformed_footprint[k].x - transformed_footprint[j].x) * (test_y - transformed_footprint[j].y) / (transformed_footprint[k].y - transformed_footprint[j].y) + transformed_footprint[j].x))
                    {
                        in_footprint = !in_footprint;
                    }
                }

                if (in_footprint) {
                    filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }
        return true;
      }
  };
}
#endif 