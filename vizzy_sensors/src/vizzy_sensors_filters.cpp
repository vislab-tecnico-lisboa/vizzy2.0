/*
 * LICENSE: https://github.com/utexas-bwi/segbot/blob/devel/LICENSE
 */

#include "vizzy_sensors/nan_to_inf_filter.h"
#include "vizzy_sensors/scan_angle_filter.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "filters/filter_base.hpp"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vizzy_sensors::VizzyFootprintFilter, filters::FilterBase<sensor_msgs::msg::LaserScan>)
PLUGINLIB_EXPORT_CLASS(vizzy_sensors::NanToInfFilter, filters::FilterBase<sensor_msgs::msg::LaserScan>)
PLUGINLIB_EXPORT_CLASS(vizzy_sensors::ScanAngleFilter, filters::FilterBase<sensor_msgs::msg::LaserScan>)