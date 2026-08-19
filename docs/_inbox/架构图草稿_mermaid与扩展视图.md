# 架构图草稿（非权威）

> **状态**：草稿 · 不进架构正文  
> **权威图**：[`架构_端侧系统架构.md`](../架构/架构_端侧系统架构.md) §2 / §5 的 PNG（`assets/架构_逻辑分层.png`、`assets/架构_双模式.png`）  
> **重新导出 PNG**：`python3 scripts/dev/export_arch_diagram_png.py`  
> **交互预览**：Cursor Canvas `motionslam-system-arch.canvas.tsx`（逻辑分层 + 双模式）

本文保留 **mermaid 可编辑源** 与 **未纳入 PNG 的扩展视图**（部署、BEV 链路），供改图 / 评审用。定稿后由人工决定是否合并进架构正文。

---

## 1. 逻辑分层（mermaid 草稿）

对应 PNG：`assets/架构_逻辑分层.png`

```mermaid
flowchart TB
  subgraph edgeLayer [边端域]
    EdgePlanner[任务与语义决策]
    EdgeModel[远端模型与策略消费]
  end

  subgraph go2 [端侧域_GO2_NX]
    subgraph L_sense [感知定位层]
      Sensors[mid360_L1]
      RGBCam[前向RGB相机]
      LIO[Super_LIO]
    end

    subgraph L_map [地图与语义层]
      BEVNet[RGB_BEV网络]
      ENUReg[ENU_world配准]
      OccMap[占据增量地图]
      SemMap[语义三维增量地图]
      MapLife[版本化与淘汰]
      TokenEnc[Token编码]
      RgbUplink[RGB关键帧上行]
    end

    subgraph L_orch [编排层]
      Ingress[边端任务入口]
      BT[BehaviorTree编排]
      Frontier[Frontier探索]
      PolicyDB[行为策略归档]
    end

    subgraph L_plan [规划层]
      PCT[PCT全局粗规划]
      SCAN[SCAN局部Bspline]
    end

    subgraph L_exec [执行层]
      CL[闭环跟踪]
      FWD[Sport转发]
      Safety[限幅急停看门狗]
    end
  end

  EdgePlanner -->|控制面_任务| Ingress
  Ingress --> BT
  Frontier --> BT
  BT --> PCT
  PCT --> BT
  BT --> SCAN
  SCAN --> CL --> FWD
  Sensors --> LIO
  RGBCam --> BEVNet
  LIO --> ENUReg
  BEVNet --> ENUReg
  ENUReg --> SemMap
  LIO --> OccMap
  OccMap -.->|强耦合| SemMap
  SemMap --> TokenEnc
  RGBCam --> RgbUplink
  RgbUplink -->|数据面_RGB_9878| EdgePlanner
  TokenEnc -->|数据面_Token_9876| EdgePlanner
  PolicyDB -->|数据面_策略| EdgeModel
  BT -->|控制面_反馈| EdgePlanner
```

**PNG 布局约定**（定稿）：五行网格；边端 **任务与语义 = 第 2 行**，**远端模型消费 = 第 4 行**。

---

## 2. 双模式（mermaid 草稿）

对应 PNG：`assets/架构_双模式.png`

```mermaid
flowchart TB
  subgraph sources [编排输入源]
    A[模式A_Frontier探索]
    B[模式B_边端任务]
  end

  BT[编排层_BT]
  PCT[PCT全局]
  SCAN[SCAN局部]
  EXEC[执行层]

  A --> BT
  B --> BT
  BT -->|"NAV时触发"| PCT
  PCT -->|"粗路径回传"| BT
  BT -->|"唯一执行goal"| SCAN --> EXEC
```

---

## 3. 部署拓扑（扩展视图 · 未出 PNG）

架构正文 §3 仍用文字 + 下表；图仅草稿。

```mermaid
flowchart LR
  subgraph edgeHost [边端PC]
    EdgeSW[Planner_VLM]
  end

  subgraph go2Body [GO2机身]
    Sport[Sport_API]
    L1[L1避障]
  end

  subgraph nx [Orin_NX_Humble容器]
    Stack[端侧栈]
  end

  Mid360[mid360] -->|UDP| Stack
  FrontRGB[前向RGB] -->|DDS_frontvideostream| Stack
  Stack -->|CycloneDDS域0| Sport
  EdgeSW -->|"TCP_ZMQ 9876-9880"| Stack
  L1 -.->|机身兜底| Sport
```

---

## 4. BEV–RGB 链路（扩展视图 · 未出 PNG）

架构正文 §2.1 文字为准；下图便于 Step 1b/1c 评审。

```mermaid
flowchart LR
  Cam[GO2前向相机]
  Uplink[rgb_keyframe_uplink]
  BEV[RGB_BEV网络]
  LIO[Super_LIO]
  ENU[ENU配准]
  Sem[语义三维地图]
  Token[Token编码]
  Edge[边端VLM_Planner]

  Cam -->|Step1b| Uplink
  Cam -->|Step1c| BEV
  LIO --> ENU
  BEV --> ENU --> Sem --> Token
  Uplink -->|:9878| Edge
  Token -->|:9876| Edge
```

---

## 5. 变更记录

| 日期 | 说明 |
|------|------|
| 2026-08-12 | 权威图改为 PNG；mermaid 与部署/BEV 扩展视图迁入本草稿 |
