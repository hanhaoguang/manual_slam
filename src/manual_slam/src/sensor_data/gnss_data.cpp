/*
 * @Author: hhg
 * @Date: 2022-09-06 19:42:50
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-06 19:43:00
 * @FilePath: /slam_ws/src/manual_slam/src/sensor_data/gnss_data.cpp
 * @Description: 静态成员变量必须在类外初始化
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */

#include "sensor_data/gnss_data.hpp"

#include "glog/logging.h"

//静态成员变量必须在类外初始化
bool lidar_localization::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian lidar_localization::GNSSData::geo_converter;

namespace lidar_localization {

void GNSSData::InitOriginPosition() {
    geo_converter.Reset(latitude, longitude, altitude);
    origin_position_inited = true;
}

void GNSSData::UpdateXYZ() {
    if (!origin_position_inited) {
        LOG(WARNING) << "GeoConverter has not set origin position";
    }
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}
}