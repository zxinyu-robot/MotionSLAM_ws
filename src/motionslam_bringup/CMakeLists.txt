cmake_minimum_required(VERSION 3.16)
project(motionslam_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config behavior_trees
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/cloud_to_grid.py scripts/robot_pose_viz.py
  scripts/planning_trajectory_viz.py
  scripts/pct_planner_env.sh
  scripts/demo_scan_bt_orchestrator.py
  scripts/plan_fail_recovery.py
  scripts/return_home.py
  scripts/edge_uplink_client.py
  scripts/data_layer_types.py
  scripts/data_layer_registry_node.py
  scripts/execution_feedback.py
  scripts/execution_feedback_node.py
  scripts/subgraph_snapshot.py
  scripts/subgraph_publisher_node.py
  scripts/rgb_forward_frame.py
  scripts/rgb_keyframe_uplink_node.py
  scripts/rgb_bev_node.py
  scripts/semantic_bev_frame.py
  scripts/semantic_token_uplink_node.py
  scripts/semantic_directive.py
  scripts/semantic_mission.py
  scripts/task_context.py
  scripts/action_group.py
  scripts/command_executor_node.py
  scripts/directive_receiver_node.py
  scripts/frontier_export.py
  scripts/frontier_scorer.py
  scripts/demo_scan_lifecycle_autostart.py
  scripts/stance_gate_node.py
  scripts/glass_suspect_analysis.py
  scripts/glass_suspect_layer.py
  scripts/motor_monitor_node.py
  scripts/foxglove_bridge_legacy.sh
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_cloud_world_topic_policy
    test/test_cloud_world_topic_policy.py)
  ament_add_pytest_test(
    test_glass_suspect_layer
    test/test_glass_suspect_layer.py)
  ament_add_pytest_test(
    test_plan_fail_recovery
    test/test_plan_fail_recovery.py)
  ament_add_pytest_test(
    test_return_home
    test/test_return_home.py)
  ament_add_pytest_test(
    test_data_layer_catalog
    test/test_data_layer_catalog.py)
  ament_add_pytest_test(
    test_subgraph_snapshot
    test/test_subgraph_snapshot.py)
  ament_add_pytest_test(
    test_rgb_forward_frame
    test/test_rgb_forward_frame.py)
  ament_add_pytest_test(
    test_semantic_directive
    test/test_semantic_directive.py)
  ament_add_pytest_test(
    test_action_group
    test/test_action_group.py)
  ament_add_pytest_test(
    test_behavior_mode
    test/test_behavior_mode.py)
  ament_add_pytest_test(
    test_semantic_mission
    test/test_semantic_mission.py)
  ament_add_pytest_test(
    test_frontier_export
    test/test_frontier_export.py)
  ament_add_pytest_test(
    test_frontier_scorer
    test/test_frontier_scorer.py)
  ament_add_pytest_test(
    test_semantic_bev_frame
    test/test_semantic_bev_frame.py)
endif()

ament_package()
