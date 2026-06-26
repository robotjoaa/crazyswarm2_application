#!/usr/bin/env python3
"""
trajectory_recorder.py

Records drone pose trajectories to a timestamped JSON file for later
comparison with PyBullet simulation output.

Topics subscribed (one per drone, discovered from /cf_N/pose):
  /cf_N/pose  (geometry_msgs/PoseStamped)

Output file: <log_dir>/traj_<ISO-timestamp>.json
Schema:
{
  "start_time_ns": <int>,
  "drones": {
    "cf_1": [{"t_ns": ..., "x": ..., "y": ..., "z": ...,
              "qx": ..., "qy": ..., "qz": ..., "qw": ...}, ...],
    ...
  }
}

Parameters:
  crazyflies_path  (str)  Path to crazyflies.yaml  (same as smaclike_aviary)
  log_dir          (str)  Directory for output files   [default: /tmp]
  record_hz        (int)  Decimation rate for logging  [default: 10]
"""

import os
import json
import yaml
import math
from datetime import datetime
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        self.declare_parameter('crazyflies_path', '')
        self.declare_parameter('log_dir', '/tmp')
        self.declare_parameter('record_hz', 10)

        crazyflies_path = self.get_parameter('crazyflies_path').value
        self.log_dir = self.get_parameter('log_dir').value
        record_hz = self.get_parameter('record_hz').value

        self.record_period = 1.0 / max(1, record_hz)
        self.last_record_time: dict = {}
        self.trajectories: dict = {}
        self.start_time_ns: int = self.get_clock().now().nanoseconds

        cf_names = self._load_cf_names(crazyflies_path)
        if not cf_names:
            self.get_logger().warn('No crazyflies found — recording nothing.')
            return

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        for name in sorted(cf_names):
            self.trajectories[name] = []
            self.last_record_time[name] = 0.0
            self.create_subscription(
                PoseStamped,
                f'/{name}/pose',
                partial(self._pose_cb, drone_id=name),
                qos,
            )
            self.get_logger().info(f'Recording /{name}/pose')

        self.get_logger().info(
            f'TrajectoryRecorder ready — writing to {self.log_dir}')

    def _load_cf_names(self, path):
        if not path or not os.path.isfile(path):
            return []
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        return list(data.get('robots', {}).keys())

    def _pose_cb(self, msg: PoseStamped, drone_id: str):
        now_ns = self.get_clock().now().nanoseconds
        elapsed = (now_ns - self.last_record_time.get(drone_id, 0)) * 1e-9
        if elapsed < self.record_period:
            return

        self.last_record_time[drone_id] = now_ns
        p = msg.pose.position
        q = msg.pose.orientation
        self.trajectories[drone_id].append({
            't_ns': now_ns - self.start_time_ns,
            'x': round(p.x, 4),
            'y': round(p.y, 4),
            'z': round(p.z, 4),
            'qx': round(q.x, 5),
            'qy': round(q.y, 5),
            'qz': round(q.z, 5),
            'qw': round(q.w, 5),
        })

    def save(self):
        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.log_dir, f'traj_{stamp}.json')
        payload = {
            'start_time_ns': self.start_time_ns,
            'drones': self.trajectories,
        }
        with open(path, 'w') as f:
            json.dump(payload, f, indent=2)
        total = sum(len(v) for v in self.trajectories.values())
        self.get_logger().info(
            f'Saved {total} samples across {len(self.trajectories)} drones → {path}')
        return path


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
