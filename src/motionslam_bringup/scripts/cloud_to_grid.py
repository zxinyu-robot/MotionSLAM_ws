#!/usr/bin/env python3
"""点云投影为 2D 占用栅格, 供 Foxglove Map 面板显示.

输入: /lio/global_map
输出: /map (nav_msgs/OccupancyGrid, frame=world)

NOTE: 必须在 Foxglove 的 Map 面板(2D俯视图)查看, 不要用 3D 面板看点云.
"""
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class CloudToGrid(Node):
    def __init__(self):
        super().__init__("cloud_to_grid")
        self.declare_parameter("input_topic", "/lio/global_map")
        self.declare_parameter("output_topic", "/map")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("width_m", 30.0)
        self.declare_parameter("height_m", 30.0)
        self.declare_parameter("z_min", 0.08)
        self.declare_parameter("z_max", 0.50)
        self.declare_parameter("center_on_robot", True)
        self.declare_parameter("inflation_cells", 2)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.res = self.get_parameter("resolution").value
        self.width_m = self.get_parameter("width_m").value
        self.height_m = self.get_parameter("height_m").value
        self.w_cells = int(self.width_m / self.res)
        self.h_cells = int(self.height_m / self.res)
        self.z_min = self.get_parameter("z_min").value
        self.z_max = self.get_parameter("z_max").value
        self.center_on_robot = self.get_parameter("center_on_robot").value
        self.inflation = int(self.get_parameter("inflation_cells").value)

        self.robot_x = 0.0
        self.robot_y = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub = self.create_publisher(OccupancyGrid, self.output_topic, 10)
        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self._cloud_cb, sensor_qos)
        self.odom_sub = self.create_subscription(
            Odometry, "/lio/odom", self._odom_cb, sensor_qos)

        self.get_logger().info(
            f"cloud_to_grid: {self.input_topic} -> {self.output_topic} "
            f"({self.w_cells}x{self.h_cells} @ {self.res}m, centered={self.center_on_robot})")

    def _odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def _origin(self):
        if self.center_on_robot:
            return (self.robot_x - self.width_m / 2.0,
                    self.robot_y - self.height_m / 2.0)
        return (-self.width_m / 2.0, -self.height_m / 2.0)

    def _mark(self, grid, col, row):
        if 0 <= col < self.w_cells and 0 <= row < self.h_cells:
            grid[row, col] = 100
            for dx in range(-self.inflation, self.inflation + 1):
                for dy in range(-self.inflation, self.inflation + 1):
                    c, r = col + dx, row + dy
                    if 0 <= c < self.w_cells and 0 <= r < self.h_cells:
                        grid[r, c] = 100

    def _cloud_cb(self, msg: PointCloud2):
        origin_x, origin_y = self._origin()
        # 0=空闲(白), 100=占据(黑). 不用 -1 未知, 否则 Foxglove 显示大片灰色
        grid = np.zeros((self.h_cells, self.w_cells), dtype=np.int8)

        for x, y, z in point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True):
            if z < self.z_min or z > self.z_max:
                continue
            col = int((x - origin_x) / self.res)
            row = int((y - origin_y) / self.res)
            self._mark(grid, col, row)

        occ = OccupancyGrid()
        occ.header.stamp = msg.header.stamp
        occ.header.frame_id = "world"
        occ.info.resolution = self.res
        occ.info.width = self.w_cells
        occ.info.height = self.h_cells
        occ.info.origin.position.x = origin_x
        occ.info.origin.position.y = origin_y
        occ.info.origin.orientation.w = 1.0
        occ.data = grid.flatten().tolist()
        self.pub.publish(occ)


def main():
    rclpy.init()
    node = CloudToGrid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
