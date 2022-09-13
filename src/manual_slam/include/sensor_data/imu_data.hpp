/*
 * @Author: hhg
 * @Date: 2022-09-06 17:27:35
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-11 17:03:09
 * @FilePath: /slam_ws/src/manual_slam/include/sensor_data/imu_data.hpp
 * @Description: imu自定数据格式，带一个四元数转旋转矩阵的方法
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */
#pragma once
// #include <eigen3/Eigen/Dense>
#include "global_def.hpp"

namespace lidar_localization {
class IMUData {
  public:
    struct LinearAcceleration {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    struct AngularVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };
    
    struct Orientation {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      double w = 0.0;
    };

    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Orientation orientation;
  
  public:
    // 把四元数转换成旋转矩阵送出去
    Eigen::Matrix3f GetOrientationMatrix() {
      Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
      Eigen::Matrix3f matrix = q.matrix().cast<float>();

      return matrix;
    }
};
}