find_package(Eigen3 REQUIRED)
message(Eigen: ${EIGEN3_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
# list(APPEND ALL_TARGET_LIBRARIES ${EIGEN3_LIBRARIES})