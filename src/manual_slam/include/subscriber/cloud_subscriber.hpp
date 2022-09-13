/*
 * @Author: hhg
 * @Date: 2022-09-06 20:02:34
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-06 20:05:06
 * @FilePath: /slam_ws/src/manual_slam/include/subscriber/cloud_subscriber.hpp
 * @Description: 
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */


#pragma once
#include <deque>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_data/cloud_data.hpp"

namespace lidar_localization {
class CloudSubscriber {
  public:
    CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    CloudSubscriber() = default;
    void ParseData(std::deque<CloudData>& deque_cloud_data);

  private:
    void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    // 每一个订阅对象维护一个双向队列
    std::deque<CloudData> new_cloud_data_;
};
}
