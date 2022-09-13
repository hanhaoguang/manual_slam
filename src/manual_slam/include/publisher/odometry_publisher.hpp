/*
 * @Author: hhg
 * @Date: 2022-09-06 19:50:51
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-11 17:02:58
 * @FilePath: /slam_ws/src/manual_slam/include/publisher/odometry_publisher.hpp
 * @Description: 
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */

#pragma once
#include <string>

// #include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "global_def.hpp"

namespace lidar_localization {
class OdometryPublisher {
  public:
    OdometryPublisher(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      std::string base_frame_id,
                      std::string child_frame_id,
                      int buff_size);
    OdometryPublisher() = default;

    void Publish(const Eigen::Matrix4f& transform_matrix);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    nav_msgs::Odometry odometry_;
};
}
