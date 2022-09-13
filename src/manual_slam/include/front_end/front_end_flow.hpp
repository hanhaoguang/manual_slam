/*
 * @Author: hhg
 * @Date: 2022-09-10 14:47:21
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-11 14:26:19
 * @FilePath: /slam_ws/src/manual_slam/include/front_end/front_end_flow.hpp
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */

#pragma once
#include <ros/ros.h>
#include "global_def.hpp"

#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "subscriber/gnss_subscriber.hpp"
// #include "tf_listener/tf_listener.hpp"
#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "front_end/front_end.hpp"

namespace lidar_localization {
class FrontEndFlow {
  public:
    FrontEndFlow(ros::NodeHandle& nh);

    bool Run();
    bool SaveMap();
    bool PublishGlobalMap();

  private:
    bool ReadData();
    // bool InitCalibration();
    bool InitWithConfig();
    bool InitGNSS();
    bool InitFrontEnd();
    bool HasData();
    bool ValidData();
    bool UpdateGNSSOdometry();
    bool UpdateLaserOdometry();
    bool PublishData();

  private:
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    // std::shared_ptr<TFListener> lidar_to_imu_ptr_;
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
    std::shared_ptr<FrontEnd> front_end_ptr_;

    std::deque<CloudData> cloud_data_buff_;
    std::deque<IMUData> imu_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;
    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();
    CloudData current_cloud_data_;
    IMUData current_imu_data_;
    GNSSData current_gnss_data_;

    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR global_map_ptr_;
    CloudData::CLOUD_PTR current_scan_ptr_;
    Eigen::Matrix4f gnss_odometry_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();

    std::string lidar_topic_;
    std::string imu_topic_;
    std::string gnss_topic_;
    YAML::Node config_node_;
    std::string config_file_path_ = WORK_SPACE_PATH + "/config/front_end_flow/config.yaml";
    
};
}

