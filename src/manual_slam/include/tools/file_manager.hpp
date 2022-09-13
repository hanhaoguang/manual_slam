/*
 * @Author: hhg
 * @Date: 2022-09-13 17:18:53
 * @LastEditors: hhg
 * @LastEditTime: 2022-09-13 17:19:01
 * @FilePath: /slam_ws/src/manual_slam/include/tools/file_manager.hpp
 * @Description: 
 * 
 * Copyright (c) 2022 by hhg hhg123567@163.com, All Rights Reserved. 
 */
#pragma once
#include <string>
#include <iostream>
#include <fstream>

namespace lidar_localization {
class FileManager{
  public:
    static bool CreateFile(std::ofstream& ofs, std::string file_path);
    static bool CreateDirectory(std::string directory_path);
};
}

