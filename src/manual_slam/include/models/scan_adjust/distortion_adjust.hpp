/*
 * @Author: hhg
 * @Date: 2022-09-13 16:44:09
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-13 16:49:04
 * @FilePath: /slam_ws/src/manual_slam/include/models/scan_adjust/distortion_adjust.hpp
 * @Description: 
 * 
 * Copyright (c) 2022 by hhg hhg123567@163.com, All Rights Reserved. 
 */


#pragma once 
#include "glog/logging.h"
#include "global_def.hpp"

#include "models/scan_adjust/distortion_adjust.hpp"
#include "sensor_data/velocity_data.hpp"
#include "sensor_data/cloud_data.hpp"

namespace lidar_localization {
class DistortionAdjust {
  public:
    void SetMotionInfo(float scan_period, VelocityData velocity_data);
    bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr);

  private:
    inline Eigen::Matrix3f UpdateMatrix(float real_time);

  private:
    float scan_period_;
    Eigen::Vector3f velocity_;
    Eigen::Vector3f angular_rate_;
};
} // namespace lidar_slam
