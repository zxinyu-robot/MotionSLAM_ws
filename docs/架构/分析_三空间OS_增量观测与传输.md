# 分析：三空间 OS、增量观测与非 WiFi-TCP 传输

> **性质**：设计分析（问答展开），**不是**架构勾选、**不是**规划进度、**不是**验收结论。  
> **权威入口仍是**：[架构_端侧系统架构.md](架构_端侧系统架构.md)（目标态）· [项目规划_Demo1三步.md](../规划/项目规划_Demo1三步.md)（进度）· [边端协同接口规范.md](../项目规范/边端协同接口规范.md)（报文）· [测试记录.md](../测试验收/测试记录.md)（真机事实）。  
> **日期**：2026-08-19 · 从连续设计问答整理，保留层次与推理，压缩现场网段为占位符。

本文按提问顺序展开。每一节先写「问的是什么」，再写「为什么会这么问」，然后才是分层分析；末尾的收束只服务于下一问，不当成已经落地。

---

## 0. 阅读地图

```text
问1  现在项目整体什么情况？
        ↓ 底座是技能，不是 OS
问2  度量—符号—世界 的空间交互中间件 OS 应如何理解？
        ↓ OS 管翻译；导航只是第一个原子技能
问3  契约基础是 JSON，如何实现 Token 化？
        ↓ JSON 是表面语法；不要用 Token 替换物模型
问4  要的是增量版本化、端侧离线；TCP-WiFi 不适用动态网；大疆 MQTT 并未 Token 化
        ↓ 控制面学大疆；数据面改成本地增量日志 + 适配器
问5  后面可能不传栅格给模型；应传端侧生产的带语义标签的世界状态观测；有无效只能测
        ↓ 观测是假说；栅格留度量内核；消融决定字段
```

贯穿五问的一条线：**不要把「给模型看的东西」「控制机器人的东西」「内核自己用的几何」焊在同一条 WiFi TCP 上。**

---

## 1. 项目整体情况（第一问）

### 1.1 问的是什么

不是要一份功能清单，而是要分清：

- 仓库声称在建什么；
- 真机已经能重复什么；
- 代码已接但未关单什么；
- 下一刀必须砍在哪，而不能跳去 Token / frontier / 大模型。

### 1.2 产品定位（仓库自己的说法）

Go2 + Jetson Orin NX 上的自研栈，Humble 容器，host 网络 + CycloneDDS 与机身 Sport 域 0 互通。Demo 1 要交付的不是「再做一套 Nav2」，而是：

- 无先验建图下的长航时 frontier 探索（规划中，未关）；
- 边端用**结构化任务**驱动 PCT 全局粗引导 + SCAN 局部；
- 端侧归档行为策略；**端侧不做大模型实时决策**，边端**不得**下发 `/cmd_vel`。

日常主链（板载，不经世界空间 JSON）：

```text
pct_resident_planner → BT 写 /initial_path → SCAN navi_mode=3 → closed_loop → forwarder → Sport
```

### 1.3 逻辑分层（实现锚点，不是 OS 本身）

| 层 | 职责 | 典型锚点 |
|----|------|----------|
| 感知定位 | 位姿、点云、前向 RGB | Super-LIO、livox、`/frontvideostream` |
| 地图与语义 | 占据增量、BEV/玻璃、Token 生产（规划中） | OctVox、`bev_glass_layer`、MSTOK |
| 编排 | 唯一执行写口、恢复、技能生命周期 | `demo_scan_bt_orchestrator` |
| 规划 | 全局粗路径；局部连续轨迹 | PCT；SCAN B-spline |
| 执行 | 跟踪、限幅、Sport | `closed_loop`、`cmd_vel_forwarder` |

约束（架构 A1–A5，分析里反复用到）：

- 只有编排层写执行 goal / `/initial_path`；
- PCT 不直写执行口；
- SCAN 不解析边端 JSON；
- JSON 只出现在边端边界；端内是 ROS。

### 1.4 阶段与「已经具备 / 尚未具备」

规划是单一进度源。Demo 1 因果链是 **1a → 1b → 1c → 2 → 3**。分析时点（2026-08-19）大致是：

**已经具备、可当事实引用（仍须看测试记录原文）：**

- Lean SCAN 闭环（空场 / 走廊复测）；
- 不经 PCT 的边端定向到达（历史 10/10，旧语义链）；
- Gate0：PCT 与 BT 可同栈、PCT 不写执行 goal（干跑 + 代码审查）；
- PCT C++ 常驻预热有体素；
- **板载 Navigate 短程闭环**（BOARD-CL）：技能实现已通，**不是**边端 JSON 闭环。

**尚未具备：**

- `ReturnHome` 真机（断网/倒序面包屑）；
- `RecoverGlass` 真机；
- 执行中重规划 + 路径头可通行；
- frontier 长航时 + 地图淘汰（1b）；
- TensorRT 真 BEV + 所谓 Token round-trip（1c）；
- 世界空间 `Navigate`（EDGE-CL / DEV-6）；
- PolicyDB。

第一问的要点：**能走的是 lean / 板载技能底座；接上的是 PCT/BT/玻璃代码；缺的是带世界空间信封的真机闭环，以及返航/玻璃关单。** 不是缺一个「更完整的导航产品」。

### 1.5 为何第一问要挡在后面所有愿景前面

后面四问都是 OS / 传输 / 观测。若把它们理解成「先做 Token 再补导航」，会跳过规划写明的因果顺序。分析立场是：

- 愿景可以写清、可以分层；
- 落地仍须技能 ABI 先闭合（至少板载已通，世界空间调用同一 ABI 仍未过）；
- 传输与观测实验不能替代 `ReturnHome` / EDGE-CL。

---

## 2. 度量—符号—世界：中间件 OS（第二问）

### 2.1 问的是什么

提出的不是「再加几个 ROS 节点」，而是一套 **空间交互中间件 OS**：

1. 度量空间对符号空间的**行为翻译**；
2. 符号空间与世界空间的**行为决策翻译**；
3. 度量空间的数据与世界空间的**交互**；
4. 对机器人操作做**原子技能**化；
5. 当前导航链路是其中一个原子技能，且面向嵌入式资源受限裁剪。

并列出五条「新一代」能力：3D 行为规划内核、世界模型调度（非 RPC）、策略数据库、版本化 3D 物料闭环、异构多机。

### 2.2 为什么现有 L0–L4 不够当 OS 定义

L0–L4（LIO / forwarder / SCAN / BT / PCT）回答的是 **单机导航怎么跑**。OS 要回答的是：

- 谁允许跨层说话；
- 什么东西可以离开机舱；
- 任务失败后谁有权再决策；
- 记忆记什么、不记什么。

所以分析里把 L0–L4 降为 **度量/技能内部实现**，把三空间升为 **与分层正交的 OS 模型**。架构文档后来吸收了这一层；本文保留当时的推理，避免把「目标态已经写进架构」误读成「已经做成」。

### 2.3 三空间各自是什么（以及故意不负责什么）

```text
世界空间  World     语音 / VLM / 因果地图 / 上游世界模型 / 仿真 / 多机意图
    ↕ ② 决策翻译（任务、拒绝、策略回放；禁止下发速度）
符号空间  Symbolic  技能名 / 模式 / 事件 / 观测标签 / session·代数 / Policy
    ↕ ① 行为翻译（占据→可通行/可探索/可归档；轨迹→技能结果）
度量空间  Metric    点云·位姿·tomogram·B-spline·Sport / 本体控制器
    ↕ ③ 数据交互（版本化物料或观测 ↔ 世界；不是原始路径 dump）
```

| 空间 | 负责 | 不负责 |
|------|------|--------|
| 度量 | 连续几何、安全限幅、本体控制 | 开放词汇推理、改任务 |
| 符号 | 技能表、仲裁、失败码、观测标签 | B-spline 优化、Sport 协议 |
| 世界 | 因果、再规划、仿真/多机意图 | 直写 `/motion/command` |

**OS 内核 = ①②③ 的契约 + ④ 把操作收成原子技能。** PCT/SCAN/BT/LIO 都不是 OS 本身。

OS0（总约束，后面所有传输讨论的底线）：翻译可跨空间；**执行只发生在度量空间**。

### 2.4 四条翻译如何对应现网（当时的映射，含缺口）

| 翻译 | 当时仓库里能指到的 | 目标态 |
|------|-------------------|--------|
| ① 度量 → 符号 | 点云、tomogram、轨迹、frontier 快照、事件码；BEV/Token 为 stub | 几何变成带代数的符号对象（可通行 / 玻璃 / frontier / 技能成败） |
| ② 符号 ↔ 世界 | 物模型信封 JSON（`behavior_mode.execute`）+ 反馈事件；禁 `cmd_vel` | 世界模型出技能意图，不是 RPC 调 topic |
| ③ 度量 ↔ 世界 | 端口 9876–9878 预留；`PRODUCE/EXECUTE` 角色已拆 | 真 3D 或观测按代数生产/消费 |
| ④ 操作 → 技能 | 导航链事实上已是技能：入 goal，出到达/拒绝/返航 | 统一 ABI：名字、配额、超时、失败语义 |

当时已存在、容易误当成「OS 已完成」的胚胎：

- `thing_envelope.v1` 七块（profile / properties / services / policy / models / events / world_fragments）；
- `DataLayerRole.PRODUCE | EXECUTE`；
- `context_generation` + `LayerCatalog` 的 STALE；
- 双模式：frontier 探索 vs 边端任务，共用同一度量内核。

胚胎 ≠ 运行时。架构后文补的 RT-1～RT-4（技能注册表、世界闭环、记忆查询面、监控树）在分析时点仍是缺口。

### 2.5 导航链作为第一个原子技能（嵌入式裁剪）

第二问第 5 点与现状一致，必须保持边界：

```text
技能 Navigate
  输入: 符号（waypoint / frontier / 信封里的 skill+goal）
  内核: PCT 粗引导 + SCAN 连续局部 + closed_loop
  执行: forwarder → Sport（控制器仍在度量空间）
  输出: 符号事件（accepted / dispatched / reached / rejected / plan_fail）
  约束: Orin 上常驻 C++ 预热；端侧不跑大模型
```

ABI 目标形态（与实现可替换）：

```text
Navigate(goal, session, context_generation, resource_quota)
  → { reached | failed, reason, policy_digest }
```

同表技能：`ReturnHome`（断网可本地）、`RecoverGlass`、`ExploreFrontier`。  
**路径、航点 YAML、`/global_path` 是技能瞬时工件，不是 OS 记忆主键。**

这一层解释了为何卡点必须先关技能契约：技能没闭合，世界模型调度、PolicyDB、多机都没有可调度对象。

### 2.6 五条「新一代」相对现状：哪块是胚胎，哪块是跃迁

#### （1）3D 空间行为规划引导内核 + 导航原子技能

- 内核 v0：PCT tomogram + 2D A*（`pct_resident_planner`）+ SCAN B-spline。
- 技能壳 v0：BT 写口、返航/玻璃代码已接。
- 尚未：执行中重规划、路径头可通行、玻璃真机、世界空间调用同一 ABI。
- 分析判断：继续加厚导航栈 **不会自动长出 OS**；把导航做成可调度技能，OS 才有第一块积木。

#### （2）以世界模型为底座的任务调度 ≠ 传统 RPC

| 当时（RPC/话题） | 目标（世界状态差分） |
|------------------|----------------------|
| 调 `build_tomogram`、发 goal、JSON ingress → ROS | 世界对象状态变化触发技能 |
| BT 按模式切分支 | 调度器消费符号世界（观测 + 因果 + 策略记忆） |
| 边端 ActionGroup ≈ 远程函数 | 边端更新世界假设；端侧只承认技能契约 |

双模式已经是两个**意图源**，但仲裁仍是 BT 规则。Step 2 的优先级/互斥只是从 RPC 迈向调度的第一步。

#### （3）新一代 DB：记行为策略，不记原始路径点

规划 Step 3 的 PolicyDB 已点名：

- 不存 pose 序列、B-spline 点为主键；
- 要存：场景、`mode_id`、技能、控制器选择、成败码；
- Demo 1：**不上送后再喂回端侧实时规划**（避免记忆模型进安全环）。

当时没有 PolicyDB 实现。`execution_feedback` + `MapSessionManager` 的 session/代数是 **版本信封**，不是策略库。

#### （4）以真实 3D 为物料、版本化生产消费

`data_layer_types.py` 已拆 PRODUCE / EXECUTE。缺的是：代数驱动淘汰（1b）、可被世界空间消费的契约产物（当时设想是 Token）、仿真/WM 的消费端（不在本仓）。

专利交底里的版本标识是 git HEAD + session，那是**工程版本**；OS 要的是**世界物料/观测版本**（能否按代数 replay）。

#### （5）异构多机

当时部署是 1×Go2 + 1×边端 PC。可迁移的是：端口分层、JSON 只在边界、身份信封、边端不下发速度。要补的是：技能 ABI 跨机、符号世界共享代数、度量仍本地闭环。

异构应出现在符号层与调度层，**不应把度量控制器做成分布式 RPC**。

### 2.7 与智源 RoboOS 的对照（分析时用的尺子）

对齐的是「成 OS 所必需的运行时」，不是技能商店：

| 智源 | 本 OS | 故意不对齐 |
|------|-------|------------|
| 云端大脑 | 世界空间 | 端侧不做大模型 |
| 小脑技能库 | OS4 ABI + **可检查的度量内核** | 内核不是黑盒 |
| 共享记忆 | 物料/观测引用 + PolicyDB，查询在世界空间 | 不把点云塞进控制 JSON |
| Profile | `thing_envelope.profile` | 不是直控关节 |
| MCP / 技能商店 / 群体 | 稳态可演进 | **不是本 OS 内核定义** |

### 2.8 第二问收束（交给第三问）

OS 与技能的边界：

```text
OS：翻译、调度、记忆、物料/观测版本、多机意图
技能内部：PCT+SCAN 内核 + BT 运行时 + Sport
```

物模型信封是 OS2 的**输出单元**；技能是 OS4 的 syscall。下一步自然问：信封现在是 JSON，怎样「Token 化」才不会打穿这条边界。

---

## 3. JSON 契约如何「Token 化」（第三问）

### 3.1 问的是什么

控制面已经是 JSON（后来收成大疆风格物模型）。要不要、以及如何变成 Token。风险是把三件不同的事叫同一个词。

### 3.2 必须先拆开的三种「Token」

| 种类 | 空间 | 当时形态 | 若「化」指什么 |
|------|------|----------|----------------|
| 控制契约 | 符号 ↔ 世界 | `thing_envelope.v1` JSON，字段已是封闭枚举 | 字符串枚举 → 稳定词表 ID；JSON 仍是 codec |
| 空间物料 | 度量 → 符号 | `:9876` MSTOK：JSON meta + OccupancyGrid 0–100 字节 | 栅格/特征 → codebook 索引（当时仍按「空间 Token」设想） |
| VL 图像 Token | 世界模型内部 | Qwen 吃 RGB，自己分词 | **不归端侧**；端侧只给原图 + 代数 |

原则：**端侧永远不吃模型内部的 BPE/视觉 token。** 世界空间必须 decode 成物模型信封，ingress 再 `validate_thing_envelope()`。这与「端侧不会用模型生成下行 JSON」一致。

### 3.3 为什么说 JSON 已经是 Token 化的地基

`behavior_mode.py` 里的封闭集合已经是词表，只是还用字符串当 ID：

- `skill`：Navigate / ReturnHome / RecoverGlass / ExploreFrontier
- `scene_type`：office_corridor / intersection / glass_facade（elevator_lobby 预留）
- `mode_id`：navigate_office / navigate_intersection_cautious / …
- `mode_state`：INIT / ARMED / ACTIVE / EXCEPTION / TERMINATED
- `reason`：plan_fail / stuck / glass_trap / link_lost
- `layer_id`：spatial_semantic_token / topology_subgraph / perception_rgb / policy_archive / …
- `method`：behavior_mode.execute / progress / event

身份字段本来就是 Token：`tid`、`bid`、`sn`、`context_generation`。  
**分析结论：不要把 xyz、路径点、点云编进词表。** 规范已禁止 `world_fragments` 内联几何；goal 的 xyz 是附着在 `semantic_ref` 上的度量参数。

因此：JSON 是**带名字的 Token 序列**；当时缺的是冻结 codebook 与 JSON↔ID 纯函数，不是再发明一种控制协议。

### 3.4 当时建议的四层（后文第四、五问会修正第 2 层的产品形态）

**第 0 层：词表登记。** 只追加、不改号、不复用删除号。未知 ID 走现有拒绝码。JSON 字符串保持线上主字段。

**第 1 层：符号 Token 给世界模型，不给狗。**

- Constrained decode：Qwen 钉在 `ALLOWED_*` 上，再校验信封；
- 可选 `skill_id` 与字符串双写，不一致则拒；
- 上行 `event`/`reason` 走同一词表；
- `world_fragments` 只带引用。

端侧 `directive_receiver` **继续只收 JSON**。

**第 2 层（第三问当时的设想）：升级 MSTOK payload。**  
帧结构「JSON meta + 二进制」是对的，但 payload 是 OccupancyGrid，不是离散符号。设想过 FSQ/VQ 的 patch 索引。  
**第五问会否定「把栅格/特征给模型」这条产品路径**；保留的是「meta + 引用 + 代数」这一帧思想，改载荷为观测而非栅格。见 §5。

**第 3 层：技能 ABI 的 Token = syscall 号。**  
`skill_id` / `mode_id` / `reason_id` / `policy_digest`。PolicyDB 按 ID 索引，JSON 只供人读。

### 3.5 明确不要做的（第三问里列出、后面仍然有效）

1. 用 LLM BPE 当控制协议；
2. 把 JSON 整包当 blob embedding（无法拒绝 `cmd_vel`，无法做代数对齐）；
3. 把 xyz / B-spline 编进词表；
4. 把 MSTOK 内联进控制 JSON；
5. 在端侧用 Token 直接调度 SCAN；
6. 等 TensorRT 才开始符号词表（词表不依赖 1c）。

### 3.6 第三问收束（交给第四问）

当时的一句话是：JSON 是表面语法；Token 化是封闭枚举的整数词表 + 空间 codebook。  
第四问立刻指出：**现网 Token 跑在 TCP-WiFi 上，这个承载本身就不成立**；「用不用令牌」也还没想好；大疆 MQTT 物模型**并没有** Token 化。于是「怎么 Token 化」让位给「数据面到底是什么、在哪落盘、用什么链路搬」。

---

## 4. 增量版本化、端侧离线、非 TCP-WiFi（第四问）

### 4.1 问的是什么（四个约束叠在一起）

1. 要的是 **增量化版本** 的数据，不是全量帧；
2. **端侧离线必须可用**（技能不等人）；
3. 现有 Token 在 **TCP-WiFi** 上，动态网络不适用，需要非该链路的交互；
4. **还没决定用不用 token 令牌**；对照大疆：MQTT 物模型控制机器人，但没有 Token 化。

### 4.2 现网为何完全不适合动态网络

当时实现本质是：狗端 TCP **客户端**硬连边端 IP，`EdgeTcpUplinkClient.sendall(整帧)`，队列满则 **drop**，无对端游标、无 replay。

| 问题 | 现实现 | 动态网络下 |
|------|--------|------------|
| 连接取向 | 狗连固定边端 host:port | 漫游/掉线 = 物料面死 |
| 全量帧 | OccupancyGrid 整包 | 弱网必堵 |
| 无游标 | 只有本地 seq 日志 | 重连不能增量追平 |
| 生产绑传输 | 节点直接 `enqueue_binary` | 边端不在 ≈ 没生产过 |

技能侧其实已经按断网设计：`ReturnHome` 不依赖 PCT/边端；`uplink_is_lost` 只触发返航。矛盾是：**导航能离线，Token 不能离线。**

### 4.3 和大疆对齐：物模型 ≠ Token，MQTT ≠ 数据面

大疆 Cloud API 的选择被分析为**正确的拆分**，不要在上面叠令牌：

```text
大疆：MQTT 物模型 = 谁 / 什么状态 / 调什么服务 / 报什么事件
      OSD 是属性，不是把点云编成 token 给云端「看懂」

本仓：thing_envelope.v1 已抄四维到 JSON
      不该再把 Navigate 编成 MQTT token
```

| 平面 | 学大疆 | 本仓要多做的 |
|------|--------|----------------|
| 控制面 | services / events / properties | 继续 JSON，或以后 MQTT；**不 Token 化** |
| 数据面 | 大疆基本不传三维增量 | 端侧可回放的增量日志（当时称 WorldDelta / VoxelTxn） |
| 执行面 | 飞控在机上 | 度量内核 + 技能 ABI |

「令牌」若指鉴权：那是 broker 连接凭证，与世界片段无关。  
「Token」若指 LLM 词表：世界空间内部，端侧离线路径用不到。

**命名建议（第四问）：** 不要叫 token/令牌；建议 **WorldDelta / VoxelTxn / fragment**。控制面继续叫物模型信封。专利交底里的 `VoxelTransaction` 指向的是这一层，当时未实现。

### 4.4 正确形态：本地日志是真相，链路只是搬运工

```text
度量空间（LIO / BEV / 占据）—— 或第五问之后：观测标签器
        │  只写差量
        ▼
┌─────────────────────────────────────────┐
│  端侧 Fragment Log（离线可用的唯一真相）  │
│  key: (bid, layer_id, generation, seq)  │
│  value: base_digest + delta + codec     │
└───────────┬─────────────────────────────┘
            ├─ 本地：SHM / 本机 DDS → 技能、玻璃、frontier、catalog
            └─ 有网才搬运：TCP（实验室）、DDS 另域、Zenoh、5G…
               MQTT 若用：只 notify head，不搬 delta 本体
```

与已有结构同构：`layer_version` + `context_generation` + `STALE_ON_CONTEXT_GENERATION`。缺的是 catalog 后面那本**可回放日志**。  
同类已有 IPC：`shm_cloud_ring`（点云，带 generation + sequence）。世界数据应走同一思路：**POSIX SHM / append log 是一等公民，TCP 降为可选 sink。**

`PRODUCE` 成功条件应从「TCP 发送成功」改为「**log commit 成功**」。

### 4.5 增量版本协议（与传输无关）

```text
记录
  bid, layer_id, context_generation, seq
  base_seq / base_digest
  codec, flags(KEYFRAME|DELTA|TOMBSTONE)
  payload   # 相对 base 的差
```

规则：

1. 同一 generation 内只传 delta；对端 `HAVE(gen, seq)` 拉缺口；
2. 代数跳变 → 新 generation，先写 KEYFRAME，旧链作废；
3. 弱网丢「可重建的中间 delta」，不能丢代数锚点（现有 queue-full drop 是反的）；
4. 控制 JSON 只带 `{layer_id, gen, seq, digest}`；
5. 本地消费者读已提交头，**不**读「是否已发到边端」。

第四问里为空间 codec 排过阶段（sparse_cells → subgraph_diff → vq_patch）。第五问把「给模型的 payload」从栅格差量改成语义观测差量；**协议骨架（generation/seq/digest/游标）仍然适用**，只是 codec 与内容变了。

### 4.6 非 TCP-WiFi：做适配器，不换 ABI

动态网络要 store-and-forward + 多承载，不是再焊一种「Token 专用 WiFi 协议」。

| 顺序 | 承载 | 角色 |
|------|------|------|
| 必须先做 | 本机 SHM / 文件 log | 拔网后技能与本地读者仍能读 `HAVE(gen,seq)` |
| 可选 | CycloneDDS **另开 domain** | 勿与 Sport 域 0 混大包 |
| 机会上行 | WiFi / 5G / 以太 / 串口 | 按游标搬缺失段；现 TCP 客户端可降为其中一个 adapter |
| 控制面 | MQTT（学大疆） | 只搬 execute/progress；properties 最多放 fragment_head |
| 以后 | Zenoh | key=`bid/layer/gen/seq` 的拉取；替代不了本地 log |

现场采集曾记 Zenoh 未装；分析明确：**不要当成 Demo 1 门禁。**

### 4.7 第四问收束（交给第五问）

一句话当时是：大疆用 MQTT 物模型、不 Token 化——控制面照抄；「增量版本 Token」不是令牌，而是端侧 `(generation, seq, delta)` 日志。

未决问题：delta **里面装什么**。若仍装栅格，则「模型看不懂、弱网也差」会在第五问被正面提出。

---

## 5. 带语义标签的世界状态观测（第五问）

### 5.1 问的是什么

产品假设，不是又一个传输细节：

- 后面都可能 **不会** 把栅格传给模型交互；
- 传了模型也看不懂；即使能看懂，也不适合弱网；
- 最好传 **端侧生产的、带语义标签的世界状态观测**；
- 这套观测有没有效，**只有测了才知道**。

### 5.2 为什么这是对第四问的修正，而不是另起炉灶

第四问解决了 **在哪存、怎么增量、怎么搬**。第五问解决 **搬什么给世界模型**。

三件事曾被焊在一条链（MSTOK over TCP）上：

| 数据 | 谁该看 | 是否作为「与模型交互」的上行 |
|------|--------|------------------------------|
| 占据 / BEV 栅格 / tomogram | PCT、SCAN、玻璃虚拟墙 | **默认否** |
| 原图 RGB | 仅当实验「VL 是否还需要像素」 | 可选、可关、弱网先关 |
| 带标签的世界状态观测 | 世界模型 / 调度 | **是（假说，须测）** |

栅格是度量空间的工作内存。世界模型要的是 OS1 已经翻译过的结果：「路口、右侧疑似玻璃、前方 frontier、上一技能 `plan_fail`」。OccupancyGrid 既吃带宽，也没有稳定词表可拒错。

因此：`:9876` MSTOK 按第五问应降为 **本机 SHM 物料**（若还需要给 SCAN），不再当作边端交互主链。`:9877` 子图更接近观测（位姿 + frontier），但标签几乎仍是几何（`geom_score`、xyz），没有 scene / glass / blocked / skill_outcome。

### 5.3 观测是什么：小、离散、可增量、可离线

一次观测 = 端侧对「此刻世界对我意味着什么」的摘要，不是地图拷贝。分析里建议的记录形态：

```text
WorldObs
  identity:   bid, generation, seq, digest
  ego:        pose, stance, safety_state
  scene:      scene_type, semantic_tags[]
  affordance: frontiers[] {id, xyz, label, score}
  hazards:    [{type: glass|stairs|crowd, where, conf}]
  skill:      active_skill, mode_id, last_reason
  map_head:   只引用本地栅格版本，不带格子
```

与物模型的分工：

- **观测** = OS3 生产的世界状态（给模型看「现在怎样」）；
- **信封** = OS2 的行为模式（模型看完后决定「用哪套技能」）；
- 两者都是符号，都不是栅格。

弱网只传观测 **delta**（标签变了、frontier 集合变了、失败码变了）。端侧离线：观测写入同一本 Fragment Log；SCAN 继续吃本地栅格；边端不在时技能不等人。

### 5.4 仓库里已经能当「观测胚」的东西（不必等 1c）

| 已有 | 离语义观测还差 |
|------|----------------|
| `execution_feedback`：event / reason / safety_state / pose | 最像；缺场景标签与障碍类型 |
| subgraph：frontiers xyz + geom_score | 有「哪能扩」，无「那是什么」 |
| `glass_trap` / `plan_fail_hold` | 已是 hazard / 失败标签 |
| 信封里的 `scene_type` / `semantic_tags` | 现在是 **下行**，不是端侧观测上行 |
| RGB 关键帧 | 像素不是标签；弱网实验应能关掉 |
| MSTOK 栅格 | 给模型基本无效；留本地 |

第一版甚至可以只由 **反馈事件 + frontier + 少量端侧规则标签** 拼出，不碰 BEV。BEV 若以后只在端侧打 `glass` / `corridor` 标签，特征图仍不出舱；1c 变成 **打标签实现的替换**，不是传输格式实验。

### 5.5 「有没有效果」只能测：观测是假说

架构只能保证 **可替换、可关、可对比**，不能保证这组标签让模型决策变好。分析里把观测当实验因子：

```text
对照 0  无世界空间（板载航点）           ← 已有 BOARD-CL 底座
对照 A  只要失败码 + pose                ← 最小观测
对照 B  A + frontier 集合
对照 C  B + 端侧语义标签（scene/glass）
对照 D  C + 偶发 RGB 关键帧              ← 验证图是否还必要
对照 X  栅格 / MSTOK                     ← 负例：弱网差、模型收益预期低
```

每条对照看技能闭环，不看「模型是否看懂图」：

- 信封被接受后是否到达（EDGE-CL）；
- 弱网：断线期间技能是否继续；恢复后观测能否按 seq 追平；
- 错误标签注入：标错 `scene_type`，`mode_id` 是否变差（说明标签被用了）；
- 带宽：观测 delta 字节/分钟，对比 MSTOK、RGB。

**无效就减字段，不要加栅格「补信息」。** 标签无效通常是词表、时延、或与技能 ABI 没对齐。

### 5.6 第五问对前四问的改写（必须写明，避免文档内部打架）

| 前问曾 implicit 的假设 | 第五问之后 |
|------------------------|------------|
| 空间 Token ≈ 给模型的 BEV/栅格/codebook | 给模型的是 WorldObs；栅格/codebook 若存在只服务内核 |
| 1c 关闭条件 ≈ MSTOK round-trip | 应改读「端侧标签观测能否被世界空间消费并改变技能选择」；规划原文未改之前，本文 **不** 擅自勾选 1c |
| PRODUCE 层以 voxel/MSTOK 为主载荷 | 主载荷改为观测；voxel 层可降为 local_only |
| 「Token 化」容易被理解成压缩地图 | 若还用 Token 一词，只表示观测记录的 `(generation, seq, digest)` |

规划门禁、架构目标态仍以原文档为准；本文只记录分析过程中的产品转向。

---

## 6. 五问合在一起的层次（分析总图，不是实施清单）

```text
世界空间
  消费 WorldObs（假说）→ 产出物模型信封（JSON 或日后 MQTT）
  不消费栅格；可选消费 RGB（实验开关）
        ↕ OS2 决策翻译（封闭词表，不 Token 化控制面）
符号空间
  技能 ABI · 模式生命周期 · 失败码 · 观测标签
  Fragment Log：(generation, seq, digest) 增量
        ↕ OS1 行为翻译（端侧规则或日后 BEV 只打标签）
度量空间
  栅格 / tomogram / B-spline / Sport     本机
  技能离线可跑；不依赖 adapter 是否连通
        ↕ OS3
  适配器（WiFi TCP 仅实验室；DDS/5G/Zenoh 可替换）
  按 HAVE(gen,seq) 搬观测 delta，不搬地图
```

命名收束：

| 建议使用 | 不要当成 |
|----------|----------|
| 物模型 / thing envelope | Token、令牌 |
| WorldObs / 世界状态观测 | 栅格、MSTOK 产品 |
| Fragment Log / WorldDelta | 「又一条 WiFi Token 协议」 |
| skill_id 等词表 ID | 控制面与 MQTT 的替代品 |
| `(generation, seq, digest)` | 鉴权令牌、LLM token |

与 Demo 1 卡点的关系（分析立场，不改规划顺序）：

- 本文不授权跳过 `ReturnHome` / `RecoverGlass` / EDGE-CL 去实现观测栈；
- 若做观测实验，最小对照 A 可以只复用现有 9880 事件，不必先上 TensorRT；
- MQTT 迁控制面、Zenoh、VQ codebook 都不是当前关单条件。

---

## 7. 仍未决、只能靠下一轮实验/决策的问题

1. 观测词表第一版具体枚举（scene / hazard / frontier label）——现在只有信封下行枚举可借用。  
2. 端侧第一版标签器：纯规则 vs 已有玻璃层 vs 日后 BEV。  
3. RGB 在对照 D 里是否值得为 Qwen 保留一条可关通道。  
4. 控制面是否迁 MQTT：与观测 log 独立；迁了也不等于数据面 MQTT。  
5. 观测无效时的失败策略：减字段、改词表、或退回「仅失败码」，而不是恢复传栅格。

---

## 8. 相关代码与文档锚点（便于对照，不当进度）

| 主题 | 锚点 |
|------|------|
| 目标态 OS / 技能 ABI | [架构_端侧系统架构.md](架构_端侧系统架构.md) |
| 物模型信封与拒绝码 | `behavior_mode.py` · [边端协同接口规范.md](../项目规范/边端协同接口规范.md) |
| TCP 上行客户端（应降为 adapter） | `edge_uplink_client.py` |
| MSTOK 帧（给模型的路径被第五问否定） | `semantic_bev_frame.py` · `semantic_token_uplink_node.py` |
| 子图 / frontier（观测胚） | `subgraph_snapshot.py` · `frontier_export.py` |
| 执行事件（最小观测） | `execution_feedback.py` |
| 层角色 PRODUCE/EXECUTE | `data_layer_types.py` |
| 本机 SHM 先例 | `motionslam_ipc/shm_cloud_ring.hpp` |
| 断网返航 | `return_home.py` |
| 边侧 Qwen 物料表 | [边侧Qwen2.5-VL接入说明.md](../开发参考/边侧Qwen2.5-VL接入说明.md) |

---

## 9. 文档自身的边界

- 不修改规划勾选，不把分析写成「OS 已交付」。  
- 不含现场 IP、SN、bag 路径。  
- 与架构正文冲突时：**进度以规划为准，报文以接口规范为准，目标态分层以架构为准；本文只保留问答推理。**
