/*
 * @Author: hhg
 * @Date: 2022-09-13 17:19:54
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-13 17:20:03
 * @FilePath: /slam_ws/src/manual_slam/src/tools/file_manager.cpp
 * @Description: 
 * 
 * Copyright (c) 2022 by hhg hhg123567@163.com, All Rights Reserved. 
 */

#include "tools/file_manager.hpp"

#include <boost/filesystem.hpp>
#include "glog/logging.h"

namespace lidar_localization {
bool FileManager::CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.open(file_path.c_str(), std::ios::app);
    if (!ofs) {
        LOG(WARNING) << "无法生成文件: " << file_path;
        return false;
    }

    return true;
}

bool FileManager::CreateDirectory(std::string directory_path) {
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }
    if (!boost::filesystem::is_directory(directory_path)) {
        LOG(WARNING) << "无法建立文件夹: " << directory_path;
        return false;
    }
    return true;
}
}