/*
 * @Author: hhg
 * @Date: 2022-09-06 20:05:46
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-11 14:42:08
 * @FilePath: /slam_ws/src/manual_slam/src/subscriber/cloud_subscriber.cpp
 * @Description: 点云订阅实现，包括回调函数
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */


#include "subscriber/cloud_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization {
CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
}

void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    CloudData cloud_data;
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));

    new_cloud_data_.push_back(cloud_data);
    
}
// 手动将回调队列加入点云buff，点云buff队列由外部参数传入
void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff) {
    if (new_cloud_data_.size() > 0) {
        cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();
    }
}
} // namespace data_input