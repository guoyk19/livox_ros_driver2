#!/usr/bin/env python3
"""Convert Livox spherical point clouds to a fixed 5 x 36 range grid.

The output layout is row-major: [theta_index, phi_index], where
theta = 75, 80, ..., 95 degrees and phi = 0, 10, ..., 350 degrees.
Distances are in metres.  Empty cells are filled with max_range and are
marked as zero in the accompanying validity-mask topic.
"""

import math
from statistics import median

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from livox_ros_driver2.msg import CustomSphericalMsg


class SphericalRangePreprocessor(Node):
    def __init__(self):
        super().__init__('spherical_range_preprocessor')

        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('range_topic', '/livox/range_grid')
        self.declare_parameter('valid_topic', '/livox/range_grid_valid')
        self.declare_parameter('raw_visualization_topic', '/livox/raw_points')
        self.declare_parameter('grid_visualization_topic', '/livox/range_grid_points')
        self.declare_parameter('max_range', 70.0)
        self.declare_parameter('min_range', 0.10)
        self.declare_parameter('pooling', 'min')  # min, median, or mean
        self.declare_parameter('publish_debug', False)

        self.max_range = float(self.get_parameter('max_range').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.pooling = str(self.get_parameter('pooling').value).lower()
        if self.pooling not in ('min', 'median', 'mean'):
            raise ValueError("pooling must be 'min', 'median', or 'mean'")

        self.theta_deg = tuple(range(75, 96, 5))
        self.phi_deg = tuple(range(0, 360, 10))
        self.n_theta = len(self.theta_deg)
        self.n_phi = len(self.phi_deg)
        self.cell_count = self.n_theta * self.n_phi

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        input_topic = str(self.get_parameter('input_topic').value)
        self.range_pub = self.create_publisher(
            Float32MultiArray, str(self.get_parameter('range_topic').value), 10)
        self.valid_pub = self.create_publisher(
            Float32MultiArray, str(self.get_parameter('valid_topic').value), 10)
        self.raw_visualization_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter('raw_visualization_topic').value), 10)
        self.grid_visualization_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter('grid_visualization_topic').value), 10)
        self.subscription = self.create_subscription(
            CustomSphericalMsg, input_topic, self.cloud_callback, qos)

        self.get_logger().info(
            f'Listening to {input_topic}; output grid is '
            f'{self.n_theta} theta x {self.n_phi} phi, pooling={self.pooling}')

    def _cell_index(self, theta_rad, phi_rad):
        theta = math.degrees(theta_rad)
        phi = math.degrees(phi_rad) % 360.0

        # Round to the closest requested direction.  Points farther than
        # half a bin from a direction are outside the requested grid.
        theta_index = int(math.floor((theta - 75.0) / 5.0 + 0.5))
        if theta_index < 0 or theta_index >= self.n_theta:
            return None

        phi_index = int(math.floor(phi / 10.0 + 0.5)) % self.n_phi
        if abs(theta - self.theta_deg[theta_index]) > 2.5:
            return None
        # Circular distance is required around phi=0/360.
        requested_phi = self.phi_deg[phi_index]
        phi_error = abs((phi - requested_phi + 180.0) % 360.0 - 180.0)
        if phi_error > 5.0:
            return None
        return theta_index * self.n_phi + phi_index

    def cloud_callback(self, msg):
        cells = [[] for _ in range(self.cell_count)]
        raw_xyz = []
        accepted = 0

        for point in msg.points:
            depth = float(point.depth)
            theta = float(point.theta)
            phi = float(point.phi)

            # MID-360 spherical data uses depth=0 for invalid/no-return
            # samples.  Also reject NaN/Inf, out-of-range data, and points
            # outside the requested angular sector.  Reflectivity is not
            # used as a validity test because a valid dark target may have 0.
            if not math.isfinite(depth) or not math.isfinite(theta) or not math.isfinite(phi):
                continue
            if depth < self.min_range or depth > self.max_range:
                continue
            raw_xyz.append(self._to_xyz(depth, theta, phi))
            index = self._cell_index(theta, phi)
            if index is None:
                continue
            cells[index].append(depth)
            accepted += 1

        ranges = []
        valid = []
        for values in cells:
            if not values:
                ranges.append(self.max_range)
                valid.append(0.0)
            elif self.pooling == 'min':
                ranges.append(min(values))
                valid.append(1.0)
            elif self.pooling == 'median':
                ranges.append(float(median(values)))
                valid.append(1.0)
            else:
                ranges.append(sum(values) / len(values))
                valid.append(1.0)

        self.range_pub.publish(self._make_array(ranges))
        self.valid_pub.publish(self._make_array(valid))
        self.raw_visualization_pub.publish(
            point_cloud2.create_cloud_xyz32(msg.header, raw_xyz))
        grid_xyz = [
            self._to_xyz(
                ranges[index],
                math.radians(self.theta_deg[index // self.n_phi]),
                math.radians(self.phi_deg[index % self.n_phi]),
            )
            for index in range(self.cell_count) if valid[index] > 0.5
        ]
        self.grid_visualization_pub.publish(
            point_cloud2.create_cloud_xyz32(msg.header, grid_xyz))

        if self.get_parameter('publish_debug').value:
            filled = sum(1 for item in valid if item > 0.5)
            self.get_logger().debug(
                f'points={len(msg.points)}, accepted={accepted}, '
                f'filled_cells={filled}/{self.cell_count}')

    @staticmethod
    def _to_xyz(depth, theta, phi):
        return (
            depth * math.sin(theta) * math.cos(phi),
            depth * math.sin(theta) * math.sin(phi),
            depth * math.cos(theta),
        )

    def _make_array(self, data):
        message = Float32MultiArray()
        message.layout.dim = [
            MultiArrayDimension(label='theta', size=self.n_theta, stride=self.cell_count),
            MultiArrayDimension(label='phi', size=self.n_phi, stride=self.n_phi),
        ]
        message.layout.data_offset = 0
        message.data = data
        return message


def main(args=None):
    rclpy.init(args=args)
    node = SphericalRangePreprocessor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
