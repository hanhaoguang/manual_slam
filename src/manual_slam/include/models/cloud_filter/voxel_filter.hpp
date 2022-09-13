/*
 * @Author: hhg
 * @Date: 2022-09-06 20:46:36
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-06 20:55:18
 * @FilePath: /slam_ws/src/manual_slam/include/models/cloud_filter/voxel_filter.hpp
 * @Description: 
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */

#pragma once
#include <pcl/filters/voxel_grid.h>
#include "models/cloud_filter/cloud_filter_interface.hpp"

namespace lidar_localization {
class VoxelFilter: public CloudFilterInterface {
  public:
    VoxelFilter(const YAML::Node& node);
    VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

  private:
    bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

  private:
    pcl::VoxelGrid<CloudData::POINT> voxel_filter_;
};
}
