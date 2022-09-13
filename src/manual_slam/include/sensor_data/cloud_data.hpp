/*
 * @Author: hhg
 * @Date: 2022-09-06 16:41:21
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-06 17:23:33
 * @FilePath: /slam_ws/src/manual_slam/include/sensor_data/cloud_data.hpp
 * @Description: 点云数据封装一个类，打上时间字段
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */
#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


namespace lidar_localization {
class CloudData {
  public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

  public:
    CloudData()
      :cloud_ptr(new CLOUD()) {
    }

  public:
    double time = 0.0;
    CLOUD_PTR cloud_ptr;
};
}