/*
 * @Author: hhg
 * @Date: 2022-09-06 21:01:35
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-11 17:02:36
 * @FilePath: /slam_ws/src/manual_slam/include/front_end/front_end.hpp
 * @Description: 里程计包括的功能：配置参数，scan2map配准，初始化姿态，更新关键帧，保存地图
 *               主要逻辑功能是那个update函数，传入点云传出位姿
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */

#pragma once
#include <deque>

// #include <eigen3/Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>
#include "global_def.hpp"
#include "sensor_data/cloud_data.hpp"
#include "models/registration/ndt_registration.hpp"
#include "models/cloud_filter/voxel_filter.hpp"

namespace lidar_localization {
class FrontEnd {
  public:
  // 定义关键帧结构体
    struct Frame { 
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        CloudData cloud_data;
    };

  public:
    FrontEnd();

    bool InitWithConfig();
    bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);
    bool SetInitPose(const Eigen::Matrix4f& init_pose);

    bool SaveMap();
    bool GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr);
    bool GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr);
    bool GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr);

  private:
    bool InitParam(const YAML::Node& config_node);
    bool InitDataPath(const YAML::Node& config_node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
    bool UpdateWithNewFrame(const Frame& new_key_frame);

  private:
    std::string data_path_ = "";

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> display_filter_ptr_;
    std::shared_ptr<RegistrationInterface> registration_ptr_; 

    std::deque<Frame> local_map_frames_;
    std::deque<Frame> global_map_frames_;

    bool has_new_local_map_ = false;
    bool has_new_global_map_ = false;
    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR global_map_ptr_;
    CloudData::CLOUD_PTR result_cloud_ptr_;
    CloudData::CLOUD_PTR filtered_cloud_ptr_;
    CloudData::CLOUD_PTR filtered_local_map_ptr_;
    
    Frame current_frame_;

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

    float key_frame_distance_ = 2.0;
    int local_frame_num_ = 20;
};
}

