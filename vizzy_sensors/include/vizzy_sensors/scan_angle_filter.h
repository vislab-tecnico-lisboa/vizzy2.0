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
        // Declare parameters with default values. This makes them available to be set from a YAML file.
        this->declare_parameter("min_angle", -0.4);
        this->declare_parameter("max_angle", 0.4);

        // Get the values and store them in member variables.
        this->get_parameter("min_angle", min_angle_);
        this->get_parameter("max_angle", max_angle_);
        
        RCLCPP_INFO(this->get_logger(), "ScanAngleFilter configured with min_angle: %f, max_angle: %f", min_angle_, max_angle_);

        return true;
      }

      virtual ~ScanAngleFilter(){}

      bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan) override
      {
        filtered_scan = input_scan;
        
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
      double min_angle_;
      double max_angle_;
  };
}
#endif