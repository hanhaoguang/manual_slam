/*
 * @Author: hhg
 * @Date: 2022-09-06 20:18:19
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-06 20:22:49
 * @FilePath: /slam_ws/src/manual_slam/include/subscriber/gnss_subscriber.hpp
 * @Description: 
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */

#pragma once
#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include "sensor_data/gnss_data.hpp"

namespace lidar_localization {
class GNSSSubscriber {
  public:
    GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    GNSSSubscriber() = default;
    void ParseData(std::deque<GNSSData>& deque_gnss_data);

  private:
    void msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<GNSSData> new_gnss_data_;
};
}
