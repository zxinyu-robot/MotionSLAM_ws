#!/usr/bin/env python3
"""objectnav_to_mission.py — MFMON/HM3D ObjectNav episode -> MotionSLAM mission waypoints YAML.

输入：objectnav_hm3d_MFMON 场景的 <scene>.json.gz（episodes + goals_by_category + floor_info）。
输出：fsr_mission_adapter 可读的 waypoints YAML（demo_scan_waypoints.ros__parameters.waypoints）。

锚定策略：每个语义目标不取物体坐标，取该物体 view_points 里 IoU 最高的
agent 观测位姿——ObjectNav 的"到达"语义是"站在能看见它的地方"，不是"贴到物体上"。
同类多实例取 max-IoU 视点最高的实例。

坐标约定：HM3D 为 y-up（floor_info.thresholds 是 y 值）；本仓库 ROS 为 z-up。
映射：ros_x = h.x, ros_y = -h.z, ros_z = h.y + z_offset（默认 +0.35 机体高度）。
在 vkernel/mock 环境中这些坐标只是"管线合法的目标点"，不代表真实场景几何；
接 HM3D 场景时需要按场景原点再做平移对齐。

label 格式：<object_category>_<object_id>@f<floor>，floor 由 y 与
floor_info.thresholds 逐层比较得出（thresholds[i] 为第 i+1 层下界）。

用法：
  objectnav_to_mission.py SCENE.json.gz [--episode N] [--all] [-o OUT.yaml] [--z-offset 0.35]
"""
import argparse, gzip, json, sys


def load(path):
    with gzip.open(path, "rt", encoding="utf-8") as f:
        return json.load(f)


def floor_of(y, thresholds):
    fl = 0
    for t in thresholds or []:
        if y >= t:
            fl += 1
    return fl


def best_viewpoint(entry):
    """返回 (position, iou)；无 view_points 退化到物体 position。"""
    vps = entry.get("view_points") or []
    if not vps:
        return entry["position"], -1.0
    best = max(vps, key=lambda v: v.get("iou", 0.0))
    return best["agent_state"]["position"], best.get("iou", 0.0)


def pick_instance(cat, goals):
    """同类别多个物体实例时，取全局最高 IoU 视点的那个。"""
    cands = []
    for key, entries in goals.items():
        for e in entries or []:
            if e.get("object_category") == cat:
                pos, iou = best_viewpoint(e)
                cands.append((iou, pos, e))
    if not cands:
        return None
    return max(cands, key=lambda c: c[0])


def ros_pos(hp, z_off):
    return {"x": round(hp[0], 4), "y": round(-hp[2], 4), "z": round(hp[1] + z_off, 4)}


def convert(scene, ep_idx, z_off):
    eps = scene["episodes"]
    goals = scene.get("goals_by_category", {})
    ths = (scene.get("floor_info") or {}).get("thresholds") or []
    ep = eps[ep_idx]
    wps, missing = [], []
    for cat in ep.get("object_goals", []):
        sel = pick_instance(cat, goals)
        if sel is None:
            missing.append(cat)
            continue
        iou, pos, entry = sel
        wp = ros_pos(pos, z_off)
        fl = floor_of(pos[1], ths)
        wp["label"] = f"{cat}_{entry.get('object_id','?')}@f{fl}"
        wps.append(wp)
    return ep, wps, missing


def emit_yaml(out, scene_name, ep, wps, z_off):
    lines = [
        "demo_scan_waypoints:",
        "  ros__parameters:",
        f"    # scene={scene_name} episode_start={ep.get('start_position')} goals={[wp['label'] for wp in wps]}",
        "    waypoints:",
    ]
    for wp in wps:
        lines.append(
            f"      - {{x: {wp['x']}, y: {wp['y']}, z: {wp['z']}, label: \"{wp['label']}\"}}")
    txt = "\n".join(lines) + "\n"
    if out == "-":
        sys.stdout.write(txt)
    else:
        with open(out, "w", encoding="utf-8") as f:
            f.write(txt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scene", help="<scene>.json.gz")
    ap.add_argument("--episode", type=int, default=0, help="episode 序号（默认 0）")
    ap.add_argument("--all", action="store_true", help="列出全部 episode 概览后退出")
    ap.add_argument("-o", "--out", default="-", help="输出 YAML 路径（默认 stdout）")
    ap.add_argument("--z-offset", type=float, default=0.35)
    a = ap.parse_args()

    scene = load(a.scene)
    if a.all:
        for i, ep in enumerate(scene["episodes"]):
            print(f"[{i}] start={ep['start_position']} goals={ep['object_goals']}")
        return
    ep, wps, missing = convert(scene, a.episode, a.z_offset)
    if missing:
        print(f"# warn: categories without gt instances: {missing}", file=sys.stderr)
    emit_yaml(a.out, a.scene.rsplit("/", 1)[-1].split(".")[0], ep, wps, a.z_offset)


if __name__ == "__main__":
    main()
