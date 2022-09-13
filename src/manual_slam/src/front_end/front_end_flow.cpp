/*
 * @Author: hhg
 * @Date: 2022-09-10 14:50:42
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-11 15:35:18
 * @FilePath: /slam_ws/src/manual_slam/src/front_end/front_end_flow.cpp
 * @Description: 控制订阅和发布,更新gnss里程计和lidar里程计
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */

#include "front_end/front_end_flow.hpp"
#include "glog/logging.h"

namespace lidar_localization {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {

    local_map_ptr_.reset(new CloudData::CLOUD());
    global_map_ptr_.reset(new CloudData::CLOUD());
    current_scan_ptr_.reset(new CloudData::CLOUD());
    // lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "velo_link", "imu_link");

    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "local_map", 100, "/map");
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "global_map", 100, "/map");
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "laser_odom", "map", "lidar", 100);
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "gnss", "map", "lidar", 100);
    front_end_ptr_ = std::make_shared<FrontEnd>();
    usleep(5000);
    InitWithConfig();  

    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, lidar_topic_, 100000);
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, imu_topic_, 1000000);
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, gnss_topic_, 1000000);
}

bool FrontEndFlow::InitWithConfig() {
    config_node_ = YAML::LoadFile(config_file_path_);
    lidar_topic_ = config_node_["lidar_topic"].as<std::string>();
    LOG(INFO) << "点云话题" << lidar_topic_;
    imu_topic_ = config_node_["imu_topic"].as<std::string>();
    LOG(INFO) << "imu话题" << imu_topic_;
    gnss_topic_ = config_node_["gnss_topic"].as<std::string>();
    LOG(INFO) << "gnss话题" << gnss_topic_;
    std::vector<float> lidar_to_imu_v = config_node_["lidar_to_imu"].as<std::vector<float>>();

    lidar_to_imu_ = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(lidar_to_imu_v.data(), 4, 4); 
    LOG(INFO) << "外参";
    LOG(INFO) << lidar_to_imu_;
    return true;
}

bool FrontEndFlow::Run() {
    LOG(INFO) << "erererererer";

    ReadData();
    LOG(INFO) << "0000000000";

    // if (!InitCalibration()) 
    //     return false;

    if (!InitGNSS())
        return false;
    LOG(INFO) << "111111111111111";

    while(HasData()) {
        if (!ValidData())
            continue;
    LOG(INFO) << "222222222222222";

        UpdateGNSSOdometry();
        InitFrontEnd();
    LOG(INFO) << "333333333333333333";

        if (UpdateLaserOdometry())
        {
            PublishData();
        }

    
    }

    return true;
}

bool FrontEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    imu_sub_ptr_->ParseData(imu_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);

    return true;
}

// bool FrontEndFlow::InitCalibration() {
//     static bool calibration_received = false;
//     if (!calibration_received) {
//         if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
//             calibration_received = true;
//         }
//     }

//     return calibration_received;
// }

bool FrontEndFlow::InitGNSS() {
    static bool gnss_inited = false;
    if (!gnss_inited && gnss_data_buff_.size() > 0) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool FrontEndFlow::InitFrontEnd() {
    static bool front_end_pose_inited = false;
    if (!front_end_pose_inited) {
        front_end_pose_inited = true;
        front_end_ptr_->SetInitPose(gnss_odometry_);
        laser_odometry_ = gnss_odometry_;
    }
    return front_end_pose_inited;
}

bool FrontEndFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;
    if (gnss_data_buff_.size() == 0)
        return false;
    
    return true;
}

bool FrontEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();

    double d_time = current_cloud_data_.time - current_imu_data_.time;
    if (d_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (d_time > 0.05) {
        imu_data_buff_.pop_front();
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

bool FrontEndFlow::UpdateGNSSOdometry() {
    gnss_odometry_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();
    gnss_odometry_(0,3) = current_gnss_data_.local_E;
    gnss_odometry_(1,3) = current_gnss_data_.local_N;
    gnss_odometry_(2,3) = current_gnss_data_.local_U;
    gnss_odometry_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    // lidar2dd = imu2dd*lidar2imu
    // Tdl = Tdi * Til
    // 证明：Til * Pl = Pi; Tdi * Pi = Pd; Tdl * Pl = Pd 左等于右证毕
    gnss_odometry_ *= lidar_to_imu_;

    return true;
}

bool FrontEndFlow::UpdateLaserOdometry() {


    laser_odometry_ = Eigen::Matrix4f::Identity();
    if (front_end_ptr_->Update(current_cloud_data_, laser_odometry_))
    {
        return true;

    }
    else 
        return false;
}

bool FrontEndFlow::PublishData() {
    gnss_pub_ptr_->Publish(gnss_odometry_);
    laser_odom_pub_ptr_->Publish(laser_odometry_);

    front_end_ptr_->GetCurrentScan(current_scan_ptr_);
    cloud_pub_ptr_->Publish(current_scan_ptr_);

    if (front_end_ptr_->GetNewLocalMap(local_map_ptr_))
        local_map_pub_ptr_->Publish(local_map_ptr_);

    return true;
}

bool FrontEndFlow::SaveMap() {
    return front_end_ptr_->SaveMap();
}

bool FrontEndFlow::PublishGlobalMap() {
    if (front_end_ptr_->GetNewGlobalMap(global_map_ptr_)) { 
        global_map_pub_ptr_->Publish(global_map_ptr_);
        global_map_ptr_.reset(new CloudData::CLOUD());
    }
    return true;
}
}