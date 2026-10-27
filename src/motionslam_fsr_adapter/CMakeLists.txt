cmake_minimum_required(VERSION 3.16)
project(motionslam_fsr_adapter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(scan_planner_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# fork-spatial-runtime：默认 FetchContent 拉取 main；也可用
# -DFSR_SOURCE_DIR=/path/to/fork-spatial-runtime 指向本地检出
option(FSR_SOURCE_DIR "" "")
set(FSR_GIT_TAG "main" CACHE STRING "fork-spatial-runtime git tag/branch")
if(FSR_SOURCE_DIR)
  add_subdirectory(${FSR_SOURCE_DIR} ${CMAKE_BINARY_DIR}/fsr_build)
else()
  include(FetchContent)
  FetchContent_Declare(fork_spatial_runtime
    GIT_REPOSITORY https://github.com/zxinyu-robot/fork-spatial-runtime.git
    GIT_TAG ${FSR_GIT_TAG})
  FetchContent_MakeAvailable(fork_spatial_runtime)
endif()

add_executable(fsr_mission_adapter src/fsr_mission_adapter.cpp)
ament_target_dependencies(fsr_mission_adapter
  rclcpp nav_msgs geometry_msgs sensor_msgs std_msgs scan_planner_msgs)
target_link_libraries(fsr_mission_adapter fork_spatial::mission yaml-cpp)
target_compile_features(fsr_mission_adapter PRIVATE cxx_std_20)

install(TARGETS fsr_mission_adapter DESTINATION lib/${PROJECT_NAME})

ament_package()
