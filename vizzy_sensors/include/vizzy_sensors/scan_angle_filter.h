/*
 * LICENSE: https://github.com/utexas-bwi/segbot/blob/devel/LICENSE
 */

#ifndef LASER_SCAN_ANGLE_FILTER_H
#define LASER_SCAN_ANGLE_FILTER_H

#include <filters/filter_base.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp" 

namespace vizzy_sensors
{
  class ScanAngleFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
  {
    public:
      bool configure() override
      {
        // We Access parameter methods via the 'parameters_interface_'
        // The node that loads this filter plugin is responsible for declaring parameters.
        
        // Get the value of min_angle, with a default of -0.4 if not set.
        this->get_parameter("min_angle", min_angle_);
        
        // Get the value of max_angle, with a default of 0.4 if not set.
        this->get_parameter("max_angle", max_angle_);
        
        // Access the logger via the 'logging_interface_'
        RCLCPP_INFO(this->get_logger(), "ScanAngleFilter configured with min_angle: %f, max_angle: %f", min_angle_, max_angle_);

        return true;
      }

      virtual ~ScanAngleFilter(){}

      bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan) override
      {
        // Copy the entire input message first to preserve all headers and metadata.
        filtered_scan = input_scan;

        // Iterate through the ranges and apply the filter logic.
        for(unsigned int i = 0; i < input_scan.ranges.size(); ++i)
        {
          const double angle = input_scan.angle_min + (double)i * input_scan.angle_increment;

          if (angle < min_angle_ || angle > max_angle_)
          {
            filtered_scan.ranges[i] = std::numeric_limits<float>::infinity(); // Set out-of-range values to infinity
          }
        }
        return true;
      }

    private:
      // Declare member variables to hold the parameters
      double min_angle_ = -0.4; // Initialize with defaults
      double max_angle_ = 0.4;  // Initialize with defaults
  };
}
#endif