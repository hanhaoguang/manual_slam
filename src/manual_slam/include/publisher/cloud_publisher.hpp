/*
 * @Author: hhg
 * @Date: 2022-09-06 16:28:15
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-06 17:22:37
 * @FilePath: /slam_ws/src/manual_slam/include/publisher/cloud_publisher.hpp
 * @Description: 点云发布类
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */
#pragma once 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_data/cloud_data.hpp"
namespace lidar_localization {
class CloudPublisher {
  public:
    CloudPublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   size_t buff_size,
                   std::string frame_id);
    // 这里为什么是default？？
    CloudPublisher() = default;
    void Publish(CloudData::CLOUD_PTR cloud_ptr_input);
  
  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
} 