/*
 * @Author: hhg
 * @Date: 2022-09-06 19:53:56
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-06 19:55:22
 * @FilePath: /slam_ws/src/manual_slam/src/publisher/odometry_publisher.cpp
 * @Description: 里程计publisher实现，传入SE（3）
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */

#include "publisher/odometry_publisher.hpp"

namespace lidar_localization {
OdometryPublisher::OdometryPublisher(ros::NodeHandle& nh, 
                                     std::string topic_name, 
                                     std::string base_frame_id,
                                     std::string child_frame_id,
                                     int buff_size)
    :nh_(nh) {

    publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
    odometry_.header.frame_id = base_frame_id;
    odometry_.child_frame_id = child_frame_id;
}

void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix) {
    odometry_.header.stamp = ros::Time::now();

    //set the position
    odometry_.pose.pose.position.x = transform_matrix(0,3);
    odometry_.pose.pose.position.y = transform_matrix(1,3);
    odometry_.pose.pose.position.z = transform_matrix(2,3);

    Eigen::Quaternionf q;
    q = transform_matrix.block<3,3>(0,0);
    odometry_.pose.pose.orientation.x = q.x();
    odometry_.pose.pose.orientation.y = q.y();
    odometry_.pose.pose.orientation.z = q.z();
    odometry_.pose.pose.orientation.w = q.w();

    publisher_.publish(odometry_);
}
}