/*
 * @Author: hhg
 * @Date: 2022-09-06 20:45:18
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-06 20:51:08
 * @FilePath: /slam_ws/src/manual_slam/include/models/cloud_filter/cloud_filter_interface.hpp
 * @Description: 点云滤波接口基类
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */

#pragma once 
#include <yaml-cpp/yaml.h>
#include "sensor_data/cloud_data.hpp"

namespace lidar_localization {
class CloudFilterInterface {
  public:
    virtual ~CloudFilterInterface() = default;

    virtual bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) = 0;
};
}

