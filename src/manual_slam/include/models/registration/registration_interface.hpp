/*
 * @Author: hhg
 * @Date: 2022-09-06 20:54:18
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-11 17:03:02
 * @FilePath: /slam_ws/src/manual_slam/include/models/registration/registration_interface.hpp
 * @Description: 
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */
/*
 * @Author: hhg
 * @Date: 2022-09-06 20:54:18
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-06 20:54:39
 * @FilePath: /slam_ws/src/manual_slam/include/models/registration/registration_interface.hpp
 * @Description: 配准接口基类，包括匹配和输入目标点云，分开是因为不一定每次匹配前都要更新目标点云
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */

#pragma once
#include <yaml-cpp/yaml.h>
// #include <eigen3/Eigen/Dense>
#include "sensor_data/cloud_data.hpp"
#include "global_def.hpp"

namespace lidar_localization {
class RegistrationInterface {
  public:
    virtual ~RegistrationInterface() = default;

    virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) = 0;
    virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                          const Eigen::Matrix4f& predict_pose, 
                          CloudData::CLOUD_PTR& result_cloud_ptr,
                          Eigen::Matrix4f& result_pose) = 0;
};
} 
