cmake_minimum_required(VERSION 3.16)
project(motionslam_pipeline)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(motionslam_msgs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration io)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(motionslam_ipc REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})

add_library(map_session_manager_lib src/MapSessionManager.cpp)
set_target_properties(map_session_manager_lib PROPERTIES
  POSITION_INDEPENDENT_CODE ON)
target_include_directories(map_session_manager_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(map_session_manager_lib
  yaml-cpp OpenSSL::Crypto ${PCL_LIBRARIES})

add_executable(demo_navigation_context src/DemoNavigationContextNode.cpp)
ament_target_dependencies(demo_navigation_context
  rclcpp std_msgs)
target_link_libraries(demo_navigation_context map_session_manager_lib)

add_executable(cmd_vel_forwarder src/CmdVelForwarder.cpp)
ament_target_dependencies(cmd_vel_forwarder rclcpp motionslam_msgs unitree_api)

add_executable(estop_go2 src/EstopGo2.cpp)
ament_target_dependencies(estop_go2 rclcpp geometry_msgs unitree_api)

add_executable(restore_go2_remote src/RestoreGo2Remote.cpp)
ament_target_dependencies(restore_go2_remote rclcpp unitree_api)

add_executable(enable_go2_api_walk src/EnableGo2ApiWalk.cpp)
ament_target_dependencies(enable_go2_api_walk rclcpp unitree_api)

add_executable(mapping_motion_guard src/MappingMotionGuard.cpp)
ament_target_dependencies(mapping_motion_guard
  rclcpp nav_msgs std_msgs)

add_executable(mapping_keyframe_manager src/MappingKeyframeManager.cpp)
ament_target_dependencies(mapping_keyframe_manager
  rclcpp nav_msgs std_msgs geometry_msgs)

add_executable(pose_graph_backend src/PoseGraphBackend.cpp)
ament_target_dependencies(pose_graph_backend
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs tf2_ros tf2_geometry_msgs
  pcl_conversions)
target_link_libraries(pose_graph_backend ${PCL_LIBRARIES})

add_executable(loop_detection_node src/LoopDetectionNode.cpp)
ament_target_dependencies(loop_detection_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs tf2_ros tf2_geometry_msgs
  pcl_conversions)
target_link_libraries(loop_detection_node ${PCL_LIBRARIES})

add_executable(local_map_node src/LocalMapNode.cpp)
ament_target_dependencies(local_map_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs tf2_geometry_msgs
  pcl_conversions)
target_link_libraries(local_map_node ${PCL_LIBRARIES})

add_executable(pct_resident_planner src/PctResidentPlanner.cpp)
ament_target_dependencies(pct_resident_planner
  rclcpp sensor_msgs nav_msgs geometry_msgs std_srvs pcl_conversions)
target_link_libraries(pct_resident_planner ${PCL_LIBRARIES})

add_executable(virtual_obstacle_merge src/VirtualObstacleMerge.cpp)
ament_target_dependencies(virtual_obstacle_merge
  rclcpp sensor_msgs pcl_conversions)
target_link_libraries(virtual_obstacle_merge ${PCL_LIBRARIES})

add_executable(bev_glass_layer src/BevGlassLayer.cpp)
ament_target_dependencies(bev_glass_layer
  rclcpp sensor_msgs nav_msgs std_msgs)

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS cmd_vel_forwarder estop_go2 restore_go2_remote enable_go2_api_walk
  mapping_motion_guard mapping_keyframe_manager pose_graph_backend
  loop_detection_node local_map_node demo_navigation_context
  pct_resident_planner virtual_obstacle_merge bev_glass_layer
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS map_session_manager_lib
  EXPORT map_session_manager_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cmd_vel_forwarder test/test_cmd_vel_forwarder.cpp)
  target_include_directories(test_cmd_vel_forwarder PRIVATE include)
  ament_add_gtest(test_traj_tracker test/test_traj_tracker.cpp)
  target_include_directories(test_traj_tracker PRIVATE include)
  ament_add_gtest(test_pos_cmd_timeout test/test_pos_cmd_timeout.cpp)
  target_include_directories(test_pos_cmd_timeout PRIVATE include)
  ament_add_gtest(test_map_session_manager test/test_map_session_manager.cpp)
  target_link_libraries(test_map_session_manager map_session_manager_lib)
endif()

ament_export_targets(map_session_manager_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(map_session_manager_lib)
ament_package()
