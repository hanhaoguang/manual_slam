/*
 * @Author: hhg
 * @Date: 2022-09-10 16:55:38
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-11 17:03:00
 * @FilePath: /slam_ws/src/manual_slam/include/param_server/param_server.hpp
 * @Description: 参数服务器管理类
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */
#pragma once 
#include <string>
#include "global_def.hpp"
// #include <eigen3/Eigen/Dense>
using namespace std;
namespace lidar_localization{
class ParamServer{
public:
    string lidar_topic_; // points_raw 原始点云数据
    string imu_topic_;        // imu_raw 对应park数据集，imu_correct对应outdoor数据集，都是原始imu数据，不同的坐标系表示
    string gps_topic_;        // odometry/gps，gps里程计
    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();// 标定参数
    // front_end
    string data_path_front_end;
    string registration_method_front_end;
    float key_frame_distance_front_end;
    int local_frame_num_front_end;
    string local_map_filter_front_end;
    string display_filter_front_end;
    string frame_filter_front_end;
    // ndt
    float ndt_res;
    float ndt_step_size;
    float ndt_trans_eps;
    int ndt_max_iter ;
    // filter
    float voxel_filter_local_map_leaf_size;
    float voxel_filter_frame_leaf_size;
    float voxel_filter_display_leaf_size;

    ParamServer() {
        
    }
};
}