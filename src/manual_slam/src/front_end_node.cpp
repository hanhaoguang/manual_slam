/*
 * @Author: hhg
 * @Date: 2022-09-06 16:57:45
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-11 15:06:52
 * @FilePath: /slam_ws/src/manual_slam/src/front_end_node.cpp
 * @Description: 前端里程计node
 * 
 * Copyright (c) 2022 by error: git config user.name && git config user.email & please set dead value or install git, All Rights Reserved. 
 */

#include <ros/ros.h>
#include "glog/logging.h"

#include "manual_slam/saveMap.h"
#include "global_def.hpp"
#include "front_end/front_end_flow.hpp"

using namespace lidar_localization;

std::shared_ptr<FrontEndFlow> _front_end_flow_ptr;

bool save_map_callback(manual_slam::saveMap::Request &request, manual_slam::saveMap::Response &response) {
    response.succeed = _front_end_flow_ptr->SaveMap();
    _front_end_flow_ptr->PublishGlobalMap();
    return response.succeed;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;
    LOG(INFO) << "1111111";
    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);
    _front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);
    LOG(INFO) << "22222";

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        _front_end_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}