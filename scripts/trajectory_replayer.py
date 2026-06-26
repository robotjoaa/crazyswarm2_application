#!/usr/bin/env python3
"""
trajectory_replayer.py

Loads a JSON file saved by trajectory_recorder and publishes the recorded
paths as:
  • visualization_msgs/MarkerArray on 'rviz/replay_paths'   (persistent PATH markers)
  • visualization_msgs/MarkerArray on 'rviz/replay_poses'   (animated sphere per drone)

Can also compare against live /cf_N/pose topics by simultaneously
publishing the live drone spheres on 'rviz/live_poses'.

Parameters:
  trajectory_file  (str)  Path to the JSON file produced by trajectory_recorder
  speed_factor     (float) Playback speed multiplier  [default: 1.0]
  loop_playback    (bool)  Restart when the recording ends  [default: false]
  compare_live     (bool)  Overlay live poses alongside replay  [default: false]
  crazyflies_path  (str)  Path to crazyflies.yaml (needed when compare_live=true)

Usage:
  ros2 run crazyswarm_application trajectory_replayer.py \
       --ros-args -p trajectory_file:=/tmp/traj_20240101_120000.json
"""

import os
import json
import math
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, PoseStamped

import yaml


# Distinct hue per drone index (HSV-like, fixed palette)
_DRONE_COLORS = [
    ColorRGBA(r=0.2, g=1.0, b=0.4, a=0.9),   # green
    ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.9),   # blue
    ColorRGBA(r=1.0, g=0.8, b=0.1, a=0.9),   # yellow
    ColorRGBA(r=1.0, g=0.3, b=0.9, a=0.9),   # magenta
    ColorRGBA(r=0.1, g=1.0, b=1.0, a=0.9),   # cyan
]


def _color(idx: int) -> ColorRGBA:
    return _DRONE_COLORS[idx % len(_DRONE_COLORS)]


class TrajectoryReplayer(Node):
    def __init__(self):
        super().__init__('trajectory_replayer')

        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('speed_factor', 1.0)
        self.declare_parameter('loop_playback', False)
        self.declare_parameter('compare_live', False)
        self.declare_parameter('crazyflies_path', '')

        traj_file = self.get_parameter('trajectory_file').value
        self.speed = max(0.01, self.get_parameter('speed_factor').value)
        self.loop = self.get_parameter('loop_playback').value
        compare_live = self.get_parameter('compare_live').value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.path_pub = self.create_publisher(MarkerArray, 'rviz/replay_paths', qos)
        self.pose_pub = self.create_publisher(MarkerArray, 'rviz/replay_poses', qos)
        self.live_pub = self.create_publisher(MarkerArray, 'rviz/live_poses', qos) if compare_live else None

        self.drone_names: list = []
        self.samples: dict = {}     # drone_id -> list of sample dicts
        self.cursor: dict = {}      # drone_id -> int index into samples
        self.playback_start_ros: int = 0
        self.recording_start_ns: int = 0

        if traj_file:
            self._load(traj_file)
        else:
            self.get_logger().warn('No trajectory_file given — publishing nothing.')
            return

        # Publish the full path markers once (latched-style by re-publishing at 0.5 Hz)
        self.create_timer(2.0, self._publish_paths)
        # Animate the pose markers at 10 Hz
        self.create_timer(0.1, self._tick_poses)

        if compare_live:
            cf_path = self.get_parameter('crazyflies_path').value
            self._subscribe_live(cf_path, qos)

        self.get_logger().info(
            f'Replaying {len(self.drone_names)} drones at {self.speed}x speed')

    # ------------------------------------------------------------------
    def _load(self, path: str):
        with open(path, 'r') as f:
            data = json.load(f)

        self.recording_start_ns = data.get('start_time_ns', 0)
        drones = data.get('drones', {})

        for idx, (name, samples) in enumerate(sorted(drones.items())):
            self.drone_names.append(name)
            self.samples[name] = samples
            self.cursor[name] = 0

        self.playback_start_ros = self.get_clock().now().nanoseconds
        total = sum(len(v) for v in self.samples.values())
        self.get_logger().info(f'Loaded {total} samples from {path}')

    # ------------------------------------------------------------------
    def _publish_paths(self):
        """Publish full recorded trajectories as LINE_STRIP markers."""
        now = self.get_clock().now().to_msg()
        array = MarkerArray()
        for idx, name in enumerate(self.drone_names):
            m = Marker()
            m.header.frame_id = '/world'
            m.header.stamp = now
            m.ns = 'replay_path'
            m.id = idx
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.012

            c = _color(idx)
            c.a = 0.6
            m.color = c
            m.lifetime.sec = 6

            for s in self.samples[name]:
                m.points.append(Point(x=s['x'], y=s['y'], z=s['z']))

            array.markers.append(m)
        self.path_pub.publish(array)

    # ------------------------------------------------------------------
    def _tick_poses(self):
        """Animate a sphere marker at each drone's current replayed position."""
        elapsed_ns = (self.get_clock().now().nanoseconds - self.playback_start_ros) * self.speed
        now = self.get_clock().now().to_msg()
        array = MarkerArray()

        all_done = True
        for idx, name in enumerate(self.drone_names):
            slist = self.samples[name]
            if not slist:
                continue

            # Advance cursor to match elapsed replay time
            while (self.cursor[name] + 1 < len(slist) and
                   slist[self.cursor[name] + 1]['t_ns'] <= elapsed_ns):
                self.cursor[name] += 1

            if self.cursor[name] < len(slist) - 1:
                all_done = False

            s = slist[self.cursor[name]]
            m = Marker()
            m.header.frame_id = '/world'
            m.header.stamp = now
            m.ns = 'replay_pose'
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = s['x']
            m.pose.position.y = s['y']
            m.pose.position.z = s['z']
            m.pose.orientation.x = s.get('qx', 0.0)
            m.pose.orientation.y = s.get('qy', 0.0)
            m.pose.orientation.z = s.get('qz', 0.0)
            m.pose.orientation.w = s.get('qw', 1.0)
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.color = _color(idx)
            m.lifetime.sec = 1

            # Label above the sphere
            label = Marker()
            label.header = m.header
            label.ns = 'replay_label'
            label.id = idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = s['x']
            label.pose.position.y = s['y']
            label.pose.position.z = s['z'] + 0.15
            label.pose.orientation.w = 1.0
            label.scale.z = 0.08
            label.color = _color(idx)
            label.text = f'{name}\n(replay)'
            label.lifetime.sec = 1

            array.markers.extend([m, label])

        self.pose_pub.publish(array)

        if all_done:
            if self.loop:
                self.playback_start_ros = self.get_clock().now().nanoseconds
                for name in self.drone_names:
                    self.cursor[name] = 0
                self.get_logger().info('Replay looped.')
            else:
                self.get_logger().info('Replay finished.', once=True)

    # ------------------------------------------------------------------
    def _subscribe_live(self, crazyflies_path: str, qos):
        self.live_poses: dict = {}

        cf_names = []
        if crazyflies_path and os.path.isfile(crazyflies_path):
            with open(crazyflies_path, 'r') as f:
                data = yaml.safe_load(f)
            cf_names = sorted(data.get('robots', {}).keys())
        else:
            cf_names = self.drone_names

        for name in cf_names:
            self.live_poses[name] = None
            self.create_subscription(
                PoseStamped,
                f'/{name}/pose',
                partial(self._live_cb, drone_id=name),
                qos,
            )

        self.create_timer(0.1, self._publish_live)

    def _live_cb(self, msg: PoseStamped, drone_id: str):
        self.live_poses[drone_id] = msg

    def _publish_live(self):
        now = self.get_clock().now().to_msg()
        array = MarkerArray()
        for idx, (name, msg) in enumerate(sorted(self.live_poses.items())):
            if msg is None:
                continue
            p = msg.pose.position
            q = msg.pose.orientation

            m = Marker()
            m.header.frame_id = '/world'
            m.header.stamp = now
            m.ns = 'live_pose'
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = p
            m.pose.orientation = q
            m.scale.x = m.scale.y = m.scale.z = 0.06
            c = _color(idx)
            c.r = min(1.0, c.r + 0.3)   # slightly brighter to distinguish from replay
            m.color = c
            m.lifetime.sec = 1

            label = Marker()
            label.header = m.header
            label.ns = 'live_label'
            label.id = idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = p.x
            label.pose.position.y = p.y
            label.pose.position.z = p.z + 0.15
            label.pose.orientation.w = 1.0
            label.scale.z = 0.08
            label.color = c
            label.text = f'{name}\n(live)'
            label.lifetime.sec = 1

            array.markers.extend([m, label])

        if array.markers:
            self.live_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryReplayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
