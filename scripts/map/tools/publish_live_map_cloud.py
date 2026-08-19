#!/usr/bin/env python3
"""建图过程中合并 PCD/scans_*.pcd 并发布到 /map_cloud (Foxglove 全局 3D).

Super-LIO 的 /lio/global_map 只含当前缓冲 (~100 帧), 落盘后会清空;
本脚本周期性合并已保存分片, 供 Foxglove 看完整累积地图.

用法 (mapping_stack 运行时):
  python3 scripts/publish_live_map_cloud.py
  python3 scripts/publish_live_map_cloud.py --period 8 --leaf 0.25
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

# 复用 downsample 读取
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from downsample_reloc_map import read_pcd_xyz_i, voxel_mean  # noqa: E402


def to_pointcloud2(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = points.astype(np.float32).tobytes()
    return msg


def merge_scans(pcd_dir: str, leaf: float) -> np.ndarray:
    paths = sorted(glob.glob(os.path.join(pcd_dir, "scans_*.pcd")))
    if not paths:
        return np.zeros((0, 4), dtype=np.float32)
    chunks: list[np.ndarray] = []
    for p in paths:
        try:
            chunks.append(read_pcd_xyz_i(p))
        except OSError:
            continue
    if not chunks:
        return np.zeros((0, 4), dtype=np.float32)
    merged = np.vstack(chunks)
    return voxel_mean(merged, leaf)


class LiveMapCloudNode(Node):
    def __init__(self, pcd_dir: str, leaf: float, period: float, topic: str) -> None:
        super().__init__("publish_live_map_cloud")
        self.pcd_dir = pcd_dir
        self.leaf = leaf
        self.pub = self.create_publisher(PointCloud2, topic, 1)
        self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f"live map: {pcd_dir}/scans_*.pcd -> {topic} every {period}s leaf={leaf}"
        )

    def _on_timer(self) -> None:
        t0 = time.time()
        pts = merge_scans(self.pcd_dir, self.leaf)
        if pts.size == 0:
            self.get_logger().warn("no scans_*.pcd yet")
            return
        msg = to_pointcloud2(pts, "world", self.get_clock().now().to_msg())
        self.pub.publish(msg)
        self.get_logger().info(
            f"published {len(pts)} pts from {len(glob.glob(os.path.join(self.pcd_dir, 'scans_*.pcd')))} fragments "
            f"({time.time() - t0:.1f}s)"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcd-dir", default=os.environ.get("MOTIONSLAM_MAPS", "/ws/maps") + "/PCD")
    parser.add_argument("--leaf", type=float, default=0.25)
    parser.add_argument("--period", type=float, default=8.0)
    parser.add_argument("--topic", default="/map_cloud")
    args = parser.parse_args()

    rclpy.init()
    node = LiveMapCloudNode(args.pcd_dir, args.leaf, args.period, args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
