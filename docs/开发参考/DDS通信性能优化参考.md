# DDS 通信性能优化参考

> 依据 [eProsima Fast DDS Performance](https://www.eprosima.com/developer-resources/performance/eprosima-fast-dds-performance) 的通用结论整理，并结合 MotionSLAM 项目栈（**CycloneDDS + ROS 2 Humble**）给出可落地的优化思路。
>
> 关联文档：[项目规划_Demo1三步.md](../规划/项目规划_Demo1三步.md) · [项目开发规范准则](../项目规范/项目开发规范准则.md)

---

## 1. 背景与适用范围

### 1.1 基准报告要点

eProsima 对 Fast DDS v2.8.0 的测试表明，DDS 通信性能主要取决于 **传输机制** 与 **是否零拷贝**：

| 指标 | 结论 |
|------|------|
| 延迟 | Intra-process < Shared Memory (SHM) < UDP |
| 吞吐量 | 同上，Intra-process 最高 |
| 零拷贝 | 对大 payload 改善最显著；小消息（< 1 KB）差异相对小 |
| 大消息 | 点云、图像等 payload 越大，减少内存拷贝的收益越高 |

报告中的三种传输层级：

1. **Intra-process（进程内）**：同一进程内 Writer 直接调用 Reader，不经传输层
2. **Shared Memory（SHM）**：同机不同进程间共享内存缓冲区
3. **UDP**：跨进程/跨网络，Fast DDS 默认方式，系统调用与协议栈开销最大

### 1.2 与本项目的关系

MotionSLAM **不使用 Fast DDS**，而是 **CycloneDDS**（与 Unitree Go2 机身 DDS 域 0 原生互通）：

| 项 | 本项目配置 |
|----|-----------|
| RMW | `rmw_cyclonedds_cpp` |
| 域 ID | `ROS_DOMAIN_ID=0` |
| 网卡绑定 | `docker/cyclonedds.xml` → `eth0` |
| 容器网络 | `--network host --ipc host`（DDS 对容器边界透明） |

**不能直接照搬 Fast DDS 的配置项**，但传输分层、零拷贝、大消息优化等 **设计原则通用**，可指导本项目的 topic 设计与节点布局。

---

## 2. 传输机制选型思路

```text
延迟/吞吐（优 → 劣）

  进程内通信 (Intra-process / component)
        ↓
  同机共享内存 (SHM)
        ↓
  UDP / 组播（跨机或默认环回）
```

### 2.1 进程内通信（优先用于热路径）

适用：Publisher 与 Subscriber 在同一进程、且调用链紧密耦合的模块。

本项目候选：

- `livox_ros_driver2` → Super-LIO 的点云/IMU 订阅（若合并为 composable node）
- Super-LIO 内部各子模块（若已同进程，无需额外 DDS  hop）
- PCT / SCAN 内部模块（若已同进程，无需额外 DDS hop）

**思路**：减少「大消息出进程再进进程」的 hops；每多一次 DDS 序列化/拷贝，对大点云都是 measurable 开销。

### 2.2 同机共享内存（SHM）

适用：必须拆成独立进程、但仍在 NX 本机运行的节点。

本项目典型拓扑（均在 NX `.18` 上）：

```text
livox_ros_driver2 ──→ super_lio ──→ PCT / SCAN ──→ motionslam_pipeline
         │                    │
         └──────── 同机 CycloneDDS / SHM ────────┘
```

CycloneDDS 支持 SHM（Iceoryx 等实现，视 Humble 打包版本而定）。同机节点间优先走 SHM，避免 UDP loopback 拷贝。

**注意**：容器已 `--ipc host`，SHM 段在宿主机与容器间可共享；仍需保证所有参与者使用同一 DDS 实现与域配置。

### 2.3 UDP / 组播

适用：

- 与 **Go2 机身**（`rt/sportmodestate` 等）跨板通信——不可避免走网络
- 开发机 Foxglove 远程可视化——带宽与延迟要求相对宽松

**思路**：控制高频大 topic 不要经 UDP 广播到不需要的订阅者；用 namespace、topic 名与 QoS 限制订阅范围。

---

## 3. 零拷贝与大消息优化

### 3.1 原则

Fast DDS 基准的核心结论：**延迟与吞吐瓶颈常来自 buffer-to-buffer 拷贝**。对大 payload：

- 启用 **loaned messages / zero-copy**（ROS 2 侧 API：`borrow_loaned_message()` 等）
- 避免在回调中不必要的 `PointCloud2` / `sensor_msgs` 深拷贝
- 使用 **固定大小或可预测 layout** 的消息类型，便于 SHM 与 loan

### 3.2 本项目高负载 Topic

| Topic（示例） | 类型 | 优化优先级 | 建议 |
|--------------|------|-----------|------|
| `/livox/lidar` | 点云 | 高 | 降低 publish 频率或 ROI 裁剪；同机 SHM；Super-LIO 侧避免重复拷贝 |
| `/lio/cloud_world` | 点云 | 高 | costmap 投影尽量就地读字段，少建中间 PCL 副本 |
| `/rt/sportmodestate` 等 | 宇树状态 | 中 | 与机身 CycloneDDS 对齐 QoS；仅 pipeline 订阅 |
| `/cmd_vel` | Twist | 低 | 小消息，优化收益有限；重点是超时保护与限幅 |

### 3.3 ROS 2 侧实践（C++）

- 订阅大消息时使用 `rclcpp::SubscriptionOptions` 与合适的 callback group，避免 executor 排队放大延迟
- 发布端评估 `SensorDataQoS` vs `Reliable`：点云/IMU 常用 **best effort + volatile**（与宇树侧对齐，见开发计划阶段 1 QoS 验证）
- 能用 **intra-process comms** 则在 launch 中 `use_intra_process_comms:=true`（仅当 pub/sub 同进程且消息类型支持）

---

## 4. 本项目已有配置（基线）

### 4.1 环境变量（`docker/Dockerfile`）

```bash
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/ws/docker/cyclonedds.xml
ENV ROS_DOMAIN_ID=0
```

### 4.2 CycloneDDS 网卡绑定（`docker/cyclonedds.xml`）

- 绑定 `eth0`（`${LIDAR_SUBNET}`），与 Go2 机身 DDS 域 0 互通
- `AllowMulticast=spdp`：发现走组播，注意与现场 WiFi/交换机 IGMP 行为一致

### 4.3 容器运行（`docker/run_container.sh`）

- `--network host`：DDS 与狗板、mid360 同网段直连
- `--ipc host`：为同机 SHM / 大缓冲共享提供前提

**结论**：架构层已做对「同域、同网、IPC 共享」三件大事；后续优化重点在 **节点拆分方式、topic QoS、大消息路径**。

---

## 5. 优化检查清单（落地时可对照）

### 5.1 架构层

- [ ] 热路径（雷达 → LIO → costmap）是否最少 DDS hop？
- [ ] 能否将 driver + LIO 或 LIO + 投影 合并为 component/container，启用 intra-process？
- [ ] 远程 Foxglove 是否只订阅诊断用降采样 topic，而非原始全量点云？

### 5.2 QoS 与互通

- [ ] 与宇树 `rt/*` topic 的 QoS 是否显式对齐（`best_effort` / `volatile` 等）？
- [ ] 是否存在「可靠 + 大队列」导致内存暴涨或延迟尖峰？
- [ ] `ros2 topic info -v` 检查 pub/sub 的 QoS 兼容性

### 5.3 大消息路径

- [ ] 点云回调中是否避免多次 `pcl::fromROSMsg` + 全量拷贝？
- [ ] costmap 投影是否可降频（如 5–10 Hz）而 LIO 仍保持高频率？
- [ ] 评估 CycloneDDS SHM / ROS 2 loaned message 在 Humble 上的可用性与实测收益

### 5.4 勿踩坑

- **不要** 为追求 Fast DDS 特性而更换 RMW——Unitree 生态绑定 CycloneDDS 0.10.x 系，换实现会导致与机身无法互通
- **不要** 在跑自研栈时与 `unitree_slam` 同时抢 mid360（UDP 独占，见开发计划 NOTE）
- **不要** 假设「容器内」一定比宿主机慢；当前 `--network host --ipc host` 下，瓶颈更常在消息拷贝与 QoS，而非容器本身

---

## 6. 性能验证建议

| 步骤 | 命令/方法 | 目的 |
|------|----------|------|
| 延迟观感 | `ros2 topic hz /livox/lidar`、`/lio/odom` | 频率是否稳定 |
| 端到端 | 记录 IMU 时间戳 → LIO 输出 odom 时间戳差 | 估算法延迟 |
| 带宽 | `ros2 topic bw /livox/lidar` | 确认网络/CPU 压力 |
| QoS | `ros2 topic info /rt/sportmodestate -v` | 与宇树侧匹配 |
| 对比 | 调整 QoS / 降采样 / 合并节点前后各跑一段 bag | 量化优化收益 |

可配合 `ros2 bag record`（MCAP，容器内已装 `rosbag2-storage-mcap`）做 A/B 对比。

---

## 7. 参考资料

- [eProsima Fast DDS Performance](https://www.eprosima.com/developer-resources/performance/eprosima-fast-dds-performance) — 传输机制与零拷贝基准（Fast DDS，原理可借鉴）
- [ROS 2 About Different Middleware Vendors](https://docs.ros.org/en/humble/Concepts/Advanced/About-Different-Middleware-Vendors.html) — RMW 选型背景
- [CycloneDDS 配置](https://cyclonedds.io/docs/cyclonedds/latest/config/config_file.html) — `CYCLONEDDS_URI` 细项
- 本项目：`src/unitree_ros2/README.md` — 宇树 ROS 2 + CycloneDDS 互通说明

---

## 8. 与开发计划阶段的对应

| 开发计划阶段 | 本文档相关动作 |
|-------------|---------------|
| 阶段 1：DDS 互通 | QoS 对齐、eth0 绑定、Foxglove 只订必要 topic |
| 阶段 2：Super-LIO | 点云/IMU 路径少拷贝；livox → LIO 频率与队列 |
| 阶段 3D：PCT+SCAN | `/lio/cloud_world` → SHM / LayerFactory；PCT on-demand tomogram |
| 阶段 4：Pipeline | `/cmd_vel` 小消息；`rt/*` 状态订阅最小化 |

优化应 **先测量再改动**；每项变更保留可回滚的 bag 与参数快照，符合 [项目开发规范准则](../项目规范/项目开发规范准则.md) 中的可验证交付要求。
