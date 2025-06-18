/*
 * LICENSE: https://github.com/utexas-bwi/segbot/blob/devel/LICENSE
 */

#ifndef LASER_SCAN_ANGLE_FILTER_H
#define LASER_SCAN_ANGLE_FILTER_H

#include <filters/filter_base.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp" // For logging macros

namespace vizzy_sensors
{
  class ScanAngleFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
  {
    public:
      bool configure() override
      {
        // Declare parameters with default values using the node interface provided by FilterBase.
        // This makes them configurable from a YAML file.
        min_angle_ = this->declare_parameter("min_angle", -0.4);
        max_angle_ = this->declare_parameter("max_angle", 0.4);

        // Use the logger provided by FilterBase to print an info message.
        RCLCPP_INFO(
          this->get_logger(),
          "ScanAngleFilter configured with min_angle: %f, max_angle: %f",
          min_angle_,
          max_angle_);

        return true;
      }

      virtual ~ScanAngleFilter(){}

      // The update method is called for each message.
      bool update(
        const sensor_msgs::msg::LaserScan & input_scan,
        sensor_msgs::msg::LaserScan & filtered_scan) override
      {
        // Copy the entire input message first to preserve all headers and metadata.
        filtered_scan = input_scan;

        // Iterate through the ranges and apply the filter logic using the member variables.
        for(unsigned int i = 0; i < input_scan.ranges.size(); ++i)
        {
          const double angle = input_scan.angle_min + (static_cast<double>(i) * input_scan.angle_increment);

          if (angle < min_angle_ || angle > max_angle_)
          {
            // Set out-of-range values to infinity.
            filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
          }
        }
        return true;
      }

    private:
      // Declare member variables to hold the parameter values.
      double min_angle_;
      double max_angle_;
  };
}
#endif