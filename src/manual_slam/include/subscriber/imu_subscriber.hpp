/*
 * @Author: hhg
 * @Date: 2022-09-06 20:19:13
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-06 20:22:25
 * @FilePath: /slam_ws/src/manual_slam/include/subscriber/imu_subscriber.hpp
 * @Description: 和cloud一样，都是，订阅回调，维持队列，提供parsedata操作
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */

#pragma once
#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "sensor_data/imu_data.hpp"

namespace lidar_localization {
class IMUSubscriber {
  public:
    IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    IMUSubscriber() = default;
    void ParseData(std::deque<IMUData>& deque_imu_data);

  private:
    void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<IMUData> new_imu_data_; 
};
}