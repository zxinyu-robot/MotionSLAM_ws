#!/usr/bin/env python3
"""生成 README 用的数据状态流转时序图（空间状态→任务意图→安全动作→新状态）。

风格：竖向泳道 = 链路组件，时间自上而下；消息箭头按三空间语义着色。
输出：docs/media/state_flow_seq.png（GitHub README 内嵌）
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
import matplotlib.font_manager as fm

# 中文字体
for name in ("Noto Sans CJK SC", "WenQuanYi Micro Hei", "SimHei",
             "Microsoft YaHei", "DejaVu Sans"):
    if any(name in f.name for f in fm.fontManager.ttflist):
        plt.rcParams["font.family"] = name
        break
plt.rcParams["axes.unicode_minus"] = False

# ---------- 泳道（从左到右 = 消息链路上的组件） ----------
LANES = [
    ("thin_harness", "Thin Harness\n任务意图入口", "#7048e8"),
    ("bt", "BT 任务编排\n（符号空间）", "#1864ab"),
    ("pct", "PCT 全局规划", "#0b7285"),
    ("scan", "SCAN 局部规划\n（度量空间）", "#087f5b"),
    ("exec", "closed_loop\n+ forwarder", "#e8590c"),
    ("vkernel", "vkernel\n虚拟设备内核", "#862e9c"),
]

# ---------- 消息（from_lane, to_lane, label, 语义组, 是否自环/返回线型） ----------
# 语义组着色：空间状态=青  任务意图=紫  安全动作=橙  新状态=灰虚线
PHASE_COLOR = {
    "state": "#0b7285",      # 空间状态
    "intent": "#7048e8",     # 任务意图
    "action": "#e8590c",     # 安全动作
    "newstate": "#868e96",   # 新状态
}
MSGS = [
    # (src, dst, label, phase)
    ("thin_harness", "bt", "① 任务意图（语义目标）\nintent: navigate(x,y,z)", "intent"),
    ("bt", "pct", "② 全局目标下发", "intent"),
    ("vkernel", "pct", "③ 空间状态\n/lio/cloud_world + /lio/robo/odom", "state"),
    ("pct", "bt", "④ /global_path", "intent"),
    ("bt", "scan", "⑤ /initial_path（子目标）", "intent"),
    ("vkernel", "scan", "⑥ 空间状态\n点云 + 位姿（持续）", "state"),
    ("scan", "exec", "⑦ 安全动作\n局部轨迹 navi_mode=3", "action"),
    ("exec", "vkernel", "⑧ /cmd_vel → 电机指令", "action"),
    ("vkernel", "bt", "⑨ 新状态回环\nodom 积分更新 → 判定子目标", "newstate"),
    ("bt", "thin_harness", "⑩ task_terminal\nstate=succeeded", "newstate"),
]

# ---------- 几何 ----------
fig, ax = plt.subplots(figsize=(13.2, 7.6), dpi=150)
fig.patch.set_facecolor("#ffffff")
ax.set_xlim(0, 13.2)
ax.set_ylim(0, 7.6)
ax.axis("off")

lane_x = {}
top_y, bot_y = 6.9, 0.55
for i, (key, label, color) in enumerate(LANES):
    x = 1.5 + i * 2.1
    lane_x[key] = x
    # 泳道竖线
    ax.plot([x, x], [top_y - 0.42, bot_y], color="#ced4da",
            lw=1.1, ls=(0, (4, 3)), zorder=1)
    # 顶部节点
    bb = FancyBboxPatch((x - 0.86, top_y - 0.40), 1.72, 0.52,
                        boxstyle="round,pad=0.06,rounding_size=0.09",
                        fc="#f1f3f5", ec=color, lw=1.6, zorder=3)
    ax.add_patch(bb)
    ax.text(x, top_y - 0.14, label, ha="center", va="center",
            fontsize=10.5, color="#212529", zorder=4, linespacing=1.25)

n = len(MSGS)
msg_ys = [top_y - 1.05 - i * ((top_y - 1.05 - bot_y - 0.35) / (n - 1))
          for i in range(n)]

for (src, dst, label, phase), y in zip(MSGS, msg_ys):
    x1, x2 = lane_x[src], lane_x[dst]
    color = PHASE_COLOR[phase]
    dashed = (phase == "newstate")
    if x2 > x1:
        arrow = FancyArrowPatch((x1 + 0.06, y), (x2 - 0.06, y),
                                arrowstyle="-|>", mutation_scale=15,
                                color=color, lw=1.9,
                                ls=(0, (5, 3)) if dashed else "-",
                                zorder=5)
        lx = (x1 + x2) / 2
    else:
        arrow = FancyArrowPatch((x1 - 0.06, y), (x2 + 0.06, y),
                                arrowstyle="-|>", mutation_scale=15,
                                color=color, lw=1.9,
                                ls=(0, (5, 3)) if dashed else "-",
                                zorder=5)
        lx = (x1 + x2) / 2
    ax.add_patch(arrow)
    ax.text(lx, y + 0.075, label, ha="center", va="bottom",
            fontsize=9.2, color=color, zorder=6,
            bbox=dict(fc="white", ec="none", alpha=0.85, pad=1.1))

# ---------- 左侧语义组标注（泳道下方的相位条） ----------
# 图例
legend_items = [
    ("空间状态", PHASE_COLOR["state"]),
    ("任务意图", PHASE_COLOR["intent"]),
    ("安全动作", PHASE_COLOR["action"]),
    ("新状态回环", PHASE_COLOR["newstate"]),
]
lx = 1.15
for name, c in legend_items:
    ax.plot([lx, lx + 0.5], [0.18, 0.18], color=c, lw=2.4,
            ls=(0, (5, 3)) if name == "新状态回环" else "-")
    ax.text(lx + 0.6, 0.18, name, va="center", fontsize=10, color="#343a40")
    lx += 1.55

ax.set_title("MotionSLAM 数据状态流转时序图\n空间状态 → 任务意图 → 安全动作 → 新状态（闭环）",
             fontsize=14.5, color="#212529", pad=14, linespacing=1.5)

plt.tight_layout()
import os
out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "..", "docs", "media", "state_flow_seq.png")
plt.savefig(out, dpi=150, bbox_inches="tight", facecolor="#ffffff")
print(f"WROTE {out}")
