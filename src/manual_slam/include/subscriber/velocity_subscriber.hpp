/*
 * @Author: hhg
 * @Date: 2022-09-13 16:46:19
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-13 16:50:19
 * @FilePath: /slam_ws/src/manual_slam/include/subscriber/velocity_subscriber.hpp
 * @Description: 
 * 
 * Copyright (c) 2022 by hhg hhg123567@163.com, All Rights Reserved. 
 */


#include "global_def.hpp"

#include <deque>
#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"
#include "sensor_data/velocity_data.hpp"

namespace lidar_localization {
class VelocitySubscriber {
  public:
    VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    VelocitySubscriber() = default;
    void ParseData(std::deque<VelocityData>& deque_velocity_data);

  private:
    void msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<VelocityData> new_velocity_data_; 
};
}