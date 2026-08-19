# 证据 bag 录制话题 — 供 N6 / NAV-2 / NAV-4 / ATE 共用
# ATE_BAG_LITE=1 时不录点云（体积更小，evo 仍可用 odom+tf）

EVIDENCE_BAG_TOPICS_LITE=(
  /lio/robo/odom
  /lio/odom
  /tf
  /tf_static
  /cmd_vel
  /plan
  /goal_pose
  /local_costmap/costmap
  /global_costmap/costmap
)

EVIDENCE_BAG_TOPICS_FULL=(
  "${EVIDENCE_BAG_TOPICS_LITE[@]}"
  /lio/cloud_world
  /livox/lidar
  /livox/imu
)
