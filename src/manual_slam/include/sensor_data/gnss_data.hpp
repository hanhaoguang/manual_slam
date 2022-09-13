/*
 * @Author: hhg
 * @Date: 2022-09-06 19:25:51
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-06 19:49:46
 * @FilePath: /slam_ws/src/manual_slam/include/sensor_data/gnss_data.hpp
 * @Description: gnss数据,时间经纬高enu，没有方差
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */

#pragma once
#include <vector>
#include <string>

#include "Geocentric/LocalCartesian.hpp"

using std::vector;
using std::string;

namespace lidar_localization {
class GNSSData {
  public:
    double time = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
    double altitude = 0.0;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;
    int status = 0;
    int service = 0;

  private:
    static GeographicLib::LocalCartesian geo_converter;
    static bool origin_position_inited;

  public: 
    void InitOriginPosition();
    void UpdateXYZ();
};
}
