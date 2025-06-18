/*
 * LICENSE: https://github.com/utexas-bwi/segbot/blob/devel/LICENSE
 */

#ifndef LASER_SCAN_NAN_TO_INF_FILTER_H
#define LASER_SCAN_NAN_TO_INF_FILTER_H

#include <filters/filter_base.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <limits> 
#include <cmath>  

namespace vizzy_sensors
{
  class NanToInfFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
  {
    public:

      bool configure() override {
        return true;
      }

      virtual ~NanToInfFilter(){}

      bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan) override {

        filtered_scan = input_scan;

        for(unsigned int i = 0; i < input_scan.ranges.size(); ++i){
          if (std::isnan(input_scan.ranges[i]) || input_scan.ranges[i] < input_scan.range_min)
          {
            filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
          }
        }

        return true;
      }
  };
}
#endif