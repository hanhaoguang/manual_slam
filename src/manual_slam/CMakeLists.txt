cmake_minimum_required(VERSION 2.8.3)
project(manual_slam)

SET(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
)

add_service_files(
  DIRECTORY srv
  FILES
  saveMap.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM EIGEN3
  
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)

set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/PCL.cmake)
include(cmake/eigen.cmake)
include(cmake/geographic.cmake)
include(cmake/yaml.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})

add_executable(front_end_node src/front_end_node.cpp ${ALL_SRCS})
add_dependencies(front_end_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(front_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})
