/*
 * @Author: hhg
 * @Date: 2022-09-06 20:55:55
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-06 20:57:31
 * @FilePath: /slam_ws/src/manual_slam/include/models/registration/ndt_registration.hpp
 * @Description: ndt
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */

#pragma once
#include <pcl/registration/ndt.h>
#include "models/registration/registration_interface.hpp"

namespace lidar_localization {
class NDTRegistration: public RegistrationInterface {
  public:
    NDTRegistration(const YAML::Node& node);
    NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
  
  private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

  private:
    pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
};
}

