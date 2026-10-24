#!/usr/bin/env python3
"""生成 README 用的数据状态流转时序图（空间状态→任务意图→安全动作→新状态）。

泳道 = 链路组件（时间自上而下）；箭头按三空间语义着色；
loop/alt 框表达子目标循环与失败分支。输出 docs/media/state_flow_seq.png
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle

for name in ("Noto Sans CJK SC", "WenQuanYi Micro Hei", "SimHei", "DejaVu Sans"):
    if any(name in f.name for f in fm.fontManager.ttflist):
        plt.rcParams["font.family"] = name
        break
plt.rcParams["axes.unicode_minus"] = False

LANES = [
    ("harness", "Thin Harness\n任务请求入口", "#7048e8"),
    ("bt", "BT 任务编排\n状态黑板（符号空间）", "#1864ab"),
    ("pct", "PCT 全局规划\n（度量空间）", "#0b7285"),
    ("scan", "SCAN 局部规划\nB 样条（度量空间）", "#087f5b"),
    ("ctrl", "closed_loop\n轨迹跟踪控制器", "#e8590c"),
    ("adapter", "go2_adapter\nCmdVelForwarder", "#862e9c"),
    ("sport", "Sport API / 底层运动\n云端 = vkernel 模拟", "#862e9c"),
    ("hub", "LIO + 空间状态管理\nSuper-LIO（世界空间）", "#2b8a3e"),
]
# 五层逻辑职责（不对应进程数）
LAYERS = [
    ("任务层", ["harness", "bt"], "#1864ab"),
    ("规划层", ["pct", "scan"], "#0b7285"),
    ("控制层", ["ctrl"], "#e8590c"),
    ("设备层", ["adapter", "sport"], "#862e9c"),
    ("状态层", ["hub"], "#2b8a3e"),
]
C = {"state": "#2b8a3e", "intent": "#7048e8", "action": "#e8590c",
     "exec": "#c92a2a", "newstate": "#868e96"}

# (src, dst, label, phase)；src==dst 为自处理
MSGS = [
    ("harness", "bt", "① GoalRequest{task_id, action_type, target_pose,\n target_frame, position_tolerance, timeout}", "intent"),
    ("hub", "bt", "② 空间状态 → BT 黑板（持续）：/lio/robo/odom 位姿·速度\n+ frame_id/stamp + 定位有效性 /lio/relocation/*", "state"),
    ("hub", "pct", "③ /lio/cloud_world + 当前位姿 → PCT 内部 tomogram\n可通行性：坡度 / 台阶高度差 / 垂直障碍 / 足迹", "state"),
    ("bt", "pct", "④ 全局目标 goal_pose{map, task_id}", "intent"),
    ("pct", "bt", "⑤ PlanResult{status, plan_id, /global_path, failure_code}", "intent"),
    ("bt", "bt", "⑥ 前视截取全局路径段 + 共线抽稀 → 局部参考", "intent"),
    ("bt", "scan", "⑦ 局部参考路径段 /initial_path{map, plan_id, subgoal_idx}", "intent"),
    ("hub", "scan", "⑧ 点云 + 位姿（持续）→ 在线占据栅格", "state"),
    ("scan", "ctrl", "⑨ 局部轨迹 B 样条\n/planning/bspline{traj_id, start_time, frame}", "action"),
    ("ctrl", "adapter", "⑩ 线速度+角速度\n/cmd_vel(vx, vy, ωz)", "action"),
    ("adapter", "sport", "⑪ 限幅 + 看门狗 400ms\n→ Move(1008)；超时 StopMove", "action"),
    ("sport", "hub", "⑫ 机体运动 → 新观测", "newstate"),
    ("ctrl", "bt", "⑬ ExecStatus{execution_state, failure_code, tracking_error,\n goal_reached, trajectory_valid, traj_id}", "exec"),
    ("bt", "bt", "⑭ 判定：|pose−subgoal| ≤ subgoal_reach_m 且速度稳定 → 下一子目标\n无进展 > stuck_timeout / 定位失效 → 停旧轨迹 → 恢复分支", "exec"),
    ("bt", "harness", "⑮ task_terminal{state=succeeded|failed|canceled, reason}\n仅最终目标到达且满足终止条件才 succeeded", "newstate"),
]

W, H = 18.4, 14.6
fig, ax = plt.subplots(figsize=(W, H), dpi=130)
ax.set_xlim(0, W); ax.set_ylim(0, H); ax.axis("off")
top, bot = H - 1.5, 1.35
lx = {}
for i, (k, label, col) in enumerate(LANES):
    x = 1.25 + i * 2.27
    lx[k] = x
    ax.plot([x, x], [top - 0.45, bot], color="#ced4da", lw=1.1, ls=(0, (4, 3)), zorder=1)
    ax.add_patch(FancyBboxPatch((x - 1.02, top - 0.42), 2.04, 0.66,
                 boxstyle="round,pad=0.05,rounding_size=0.09",
                 fc="#f8f9fa", ec=col, lw=1.8, zorder=3))
    ax.text(x, top - 0.09, label, ha="center", va="center", fontsize=9.6,
            color="#212529", zorder=4, linespacing=1.3)

GAP_AFTER = {4: 0.75, 13: 0.5}  # 给 alt 框与 loop 框尾留空
for name, keys, col in LAYERS:
    xs = [lx[k] for k in keys]
    x0, x1 = min(xs) - 1.08, max(xs) + 1.08
    ax.add_patch(Rectangle((x0, top + 0.32), x1 - x0, 0.36, fc=col, ec=col, alpha=0.14, zorder=2))
    ax.text((x0 + x1) / 2, top + 0.5, name, ha="center", va="center",
            fontsize=10.5, color=col, weight="bold", zorder=3)
ys, y = [], top - 1.2
for i in range(len(MSGS)):
    ys.append(y)
    y -= 0.66 + GAP_AFTER.get(i, 0.0)
for (s, d, label, ph), y in zip(MSGS, ys):
    col = C[ph]
    ls = (0, (5, 3)) if ph == "newstate" else "-"
    if s == d:
        x = lx[s]
        ax.add_patch(FancyArrowPatch((x + 0.05, y + 0.12), (x + 0.05, y - 0.18),
                     connectionstyle="arc3,rad=-1.6", arrowstyle="-|>",
                     mutation_scale=12, color=col, lw=1.6, zorder=5))
        ax.text(x + 0.42, y - 0.03, label, ha="left", va="center", fontsize=8.6,
                color=col, zorder=6, bbox=dict(fc="white", ec="none", alpha=0.9, pad=1))
        continue
    x1, x2 = lx[s], lx[d]
    sgn = 1 if x2 > x1 else -1
    ax.add_patch(FancyArrowPatch((x1 + sgn * 0.06, y), (x2 - sgn * 0.06, y),
                 arrowstyle="-|>", mutation_scale=14, color=col, lw=1.8, ls=ls, zorder=5))
    ax.text((x1 + x2) / 2, y + 0.06, label, ha="center", va="bottom", fontsize=8.6,
            color=col, zorder=6, linespacing=1.2,
            bbox=dict(fc="white", ec="none", alpha=0.9, pad=1))

def frame(y0, y1, x0, x1, tag, note, col):
    ax.add_patch(Rectangle((x0, y1), x1 - x0, y0 - y1, fill=False, ec=col, lw=1.3,
                 ls=(0, (6, 3)), zorder=2))
    ax.add_patch(Rectangle((x0, y0 - 0.28), 0.62, 0.28, fc=col, ec=col, zorder=3))
    ax.text(x0 + 0.31, y0 - 0.14, tag, ha="center", va="center", fontsize=9,
            color="white", zorder=4, weight="bold")
    ax.text(x0 + 0.72, y0 - 0.14, note, ha="left", va="center", fontsize=8.6,
            color=col, zorder=4)

frame(ys[4] - 0.12, ys[4] - 0.62, lx["bt"] - 0.95, lx["pct"] + 1.05, "alt",
      "status≠OK（不可达/地图不完整/定位无效/超时）→ 重试 · 重规划 · 恢复 · 终止", "#c92a2a")
frame(ys[5] + 0.45, ys[13] - 0.42, lx["bt"] - 1.0, lx["hub"] + 1.05, "loop",
      "逐子目标推进：中间子目标到达 → 回到 ⑥；最终目标到达 → 跳出", "#1864ab")

ax.text(0.25, 0.95,
        "时空契约：所有接口携带 frame_id（map）+ stamp + 有效性；关联 ID 链 task_id → plan_id → traj_id，"
        "规划更新 / 定位重置后旧版本数据作废，不得继续跟踪。\n"
        "执行三事件区分：速度指令被接受 ≠ 机器人实际运动 ≠ 到达目标；设备层看门狗超时自动 StopMove，不依赖 BT 响应。\n"
        "五层为逻辑职责划分，不对应进程数；状态层对任务/规划/控制层是持续反馈，执行链向 BT 返回结果，构成任务级闭环。",
        fontsize=9, color="#343a40", va="center", linespacing=1.6)
x = 0.4
for name, k in [("空间状态", "state"), ("任务意图 / 规划结果", "intent"),
                ("安全动作", "action"), ("执行状态 / 判定", "exec"), ("新状态回环", "newstate")]:
    ax.plot([x, x + 0.5], [0.35, 0.35], color=C[k], lw=2.4,
            ls=(0, (5, 3)) if k == "newstate" else "-")
    ax.text(x + 0.6, 0.35, name, va="center", fontsize=9.5, color="#343a40")
    x += 2.6
ax.set_title("MotionSLAM 数据状态流转时序图\n空间状态 → 任务意图 → 安全动作 → 新状态（任务级闭环）",
             fontsize=14.5, color="#212529", pad=8, linespacing=1.5)
plt.tight_layout()
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                   "docs", "media", "state_flow_seq.png")
plt.savefig(os.path.normpath(out), dpi=130, bbox_inches="tight", facecolor="white")
print("WROTE", os.path.normpath(out))
