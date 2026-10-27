# 地形安全评估决策（TerrainAssessor 输出）
std_msgs/Header header

# 故障标志位（与 legged_terrain::FaultFlag 一一对应，bit mask）
uint32 FAULT_NONE = 0
uint32 FAULT_STALE = 1
uint32 FAULT_SPARSE_SUPPORT = 2
uint32 FAULT_INVALID_NORMAL = 4
uint32 FAULT_EXCESSIVE_SLOPE = 8
uint32 FAULT_EXCESSIVE_ROUGHNESS = 16
uint32 FAULT_CONTACT_UNCERTAIN = 32
uint32 faults

# 风险分级
uint8 RISK_NOMINAL = 0
uint8 RISK_CAUTION = 1
uint8 RISK_CRITICAL = 2
uint8 risk

# 安全模式（底层运动约束）
uint8 MODE_NORMAL = 0
uint8 MODE_REDUCE_SPEED = 1
uint8 MODE_HOLD_POSITION = 2
uint8 safety_mode

float64 support_height_m
float64[3] surface_normal
float64 slope_deg
float64 roughness_m
float64 support_confidence
float64 traversability
