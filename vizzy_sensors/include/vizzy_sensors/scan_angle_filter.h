/*
 * LICENSE: https://github.com/utexas-bwi/segbot/blob/devel/LICENSE
 */

#ifndef LASER_SCAN_ANGLE_FILTER_HPP_
#define LASER_SCAN_ANGLE_FILTER_HPP_

#include <limits>
#include <string>

#include "filters/filter_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace vizzy_sensors
{
class ScanAngleFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
public:
  bool configure() override
  {
    // The FilterBase class provides getParam for parameter access.
    if (!getParam("min_angle", min_angle_)) {
      // Use the logging_interface_ member variable to get the logger.
      RCLCPP_ERROR(logging_interface_->get_logger(), "ScanAngleFilter: Could not find parameter 'min_angle'. Using default value.");
    }
    if (!getParam("max_angle", max_angle_)) {
      // Use the logging_interface_ member variable to get the logger.
      RCLCPP_ERROR(logging_interface_->get_logger(), "ScanAngleFilter: Could not find parameter 'max_angle'. Using default value.");
    }

    // Use the logging_interface_ member variable to get the logger.
    RCLCPP_INFO(
      logging_interface_->get_logger(),
      "ScanAngleFilter configured with min_angle: %.2f and max_angle: %.2f",
      min_angle_,
      max_angle_);

    return true;
  }

  virtual ~ScanAngleFilter() {}

  bool update(
    const sensor_msgs::msg::LaserScan & input_scan,
    sensor_msgs::msg::LaserScan & filtered_scan) override
  {
    filtered_scan = input_scan;

    for (size_t i = 0; i < input_scan.ranges.size(); ++i) {
      const double angle = input_scan.angle_min + (static_cast<double>(i) * input_scan.angle_increment);

      if (angle < min_angle_ || angle > max_angle_) {
        filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
      }
    }
    return true;
  }

private:
  double min_angle_ = -0.4;
  double max_angle_ = 0.4;
};
} 
#endif  