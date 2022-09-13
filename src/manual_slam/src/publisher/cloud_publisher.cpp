/*
 * @Author: hhg
 * @Date: 2022-09-06 16:46:31
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-06 17:23:14
 * @FilePath: /slam_ws/src/manual_slam/src/publisher/cloud_publisher.cpp
 * @Description: 点云发布实现
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */

#include "publisher/cloud_publisher.hpp"

namespace lidar_localization {
CloudPublisher::CloudPublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               size_t buff_size,
                               std::string frame_id)
    :nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}
void CloudPublisher::Publish(CloudData::CLOUD_PTR  cloud_ptr_input) {
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);
    cloud_ptr_output->header.stamp = ros::Time::now();
    cloud_ptr_output->header.frame_id = frame_id_;
    publisher_.publish(*cloud_ptr_output);
}

} // namespace data_output