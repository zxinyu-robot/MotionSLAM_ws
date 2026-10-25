# 地形感知观测（legged-terrain-state 契约对齐：感知产出观测，不做控制决策）
std_msgs/Header header
float64 support_height_m      # 机器人脚下支撑面高度
float64[3] surface_normal     # 估计地表法向
float64 roughness_m           # 支撑面粗糙度 (m)
uint32 support_points         # 支撑面实测点数
uint32 expected_points        # 期望点数（支撑置信度分母）
