#!/usr/bin/env python3
"""Validate ROS sensors; optionally move briefly in the simulator and reset.

Run only against the configured, single-agent F1TENTH simulator.
Wall time here measures message delivery and limits the diagnostic command;
it is not a lap timer or a measurement of simulated race time.
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--move', action='store_true', help='Move at 0.5 m/s, stop, test steering and reset')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('validate_zandvoort')
    state = {'scan': None, 'odom': None, 'map': None}
    arrivals = {'scan': [], 'odom': []}
    trajectory = []

    def receive(key, msg):
        state[key] = msg
        if key in arrivals:
            arrivals[key].append(time.monotonic())
        if key == 'odom':
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            trajectory.append([p.x, p.y, yaw])

    subscriptions = [
        node.create_subscription(LaserScan, '/scan', lambda m: receive('scan', m), 10),
        node.create_subscription(Odometry, '/ego_racecar/odom', lambda m: receive('odom', m), 10),
        node.create_subscription(OccupancyGrid, '/map', lambda m: receive('map', m),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE)),
    ]
    drive = node.create_publisher(AckermannDriveStamped, '/drive', 10)
    reset = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def spin(seconds, speed=None, steering=0.0):
        end = time.monotonic() + seconds
        next_publish = 0.0
        while time.monotonic() < end:
            if speed is not None and time.monotonic() >= next_publish:
                command = AckermannDriveStamped()
                command.header.stamp = node.get_clock().now().to_msg()
                command.drive.speed = speed
                command.drive.steering_angle = steering
                drive.publish(command)
                next_publish = time.monotonic() + 0.05
            rclpy.spin_once(node, timeout_sec=0.01)

    def pose():
        return trajectory[-1].copy()

    def reset_pose(initial):
        spin(1.0, 0.0)
        for _ in range(3):
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.pose.position.x, msg.pose.pose.position.y = initial[:2]
            msg.pose.pose.orientation.z = math.sin(initial[2]/2)
            msg.pose.pose.orientation.w = math.cos(initial[2]/2)
            reset.publish(msg)
            spin(0.15, 0.0)
        spin(0.5, 0.0)

    moved = False
    initial = None
    try:
        spin(3.0)
        assert all(v is not None for v in state.values()), 'Missing scan, odometry or map'
        assert node.count_subscribers('/drive') == 1, 'Expected one simulator drive subscriber'
        assert node.count_publishers('/drive') == 1, 'Another controller is publishing commands'
        initial = pose()
        scan = state['scan']
        ranges = np.asarray(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges))*scan.angle_increment
        assert len(ranges) == 1080 and np.isfinite(ranges).all(), 'Invalid LiDAR'
        assert (ranges > 0).all(), 'Nonpositive LiDAR ranges'
        grid = state['map']
        assert grid.info.width == 2000 and grid.info.height == 2000, 'Wrong map dimensions'
        assert abs(grid.info.resolution - 0.05109) < 1e-7, 'Wrong map resolution'
        report = {
            'initial_pose': initial,
            'map': {'width': grid.info.width, 'height': grid.info.height,
                    'resolution': grid.info.resolution,
                    'origin': [grid.info.origin.position.x, grid.info.origin.position.y],
                    'frame': grid.header.frame_id},
            'scan': {'beams': len(ranges), 'frame': scan.header.frame_id,
                     'angle_min': scan.angle_min, 'angle_increment': scan.angle_increment,
                     'front_m': float(ranges[np.argmin(abs(angles))]),
                     'minimum_m': float(ranges.min()), 'finite_fraction': float(np.isfinite(ranges).mean())},
            'delivery_hz_wall_time': {k: (len(v)-1)/(v[-1]-v[0]) for k, v in arrivals.items()},
            'clock_publishers': node.count_publishers('/clock'),
            'motion_executed': args.move,
        }
        if args.move:
            moved = True
            spin(1.5, 0.5)
            spin(1.5, 0.0)
            straight_end = pose()
            displacement = math.dist(initial[:2], straight_end[:2])
            assert 0.2 < displacement < 1.2, f'Unexpected displacement: {displacement}'
            assert abs(state['odom'].twist.twist.linear.x) < 0.02, 'Vehicle did not stop'
            reset_pose(initial)
            spin(1.5, 0.5, 0.1)
            spin(1.5, 0.0)
            turn_end = pose()
            yaw_change = math.atan2(math.sin(turn_end[2]-initial[2]), math.cos(turn_end[2]-initial[2]))
            assert yaw_change > 0.03, 'Positive steering must increase yaw'
            reset_pose(initial)
            assert math.dist(pose()[:2], initial[:2]) < 0.02, 'Reset position failed'
            assert abs(state['odom'].twist.twist.linear.x) < 0.02, 'Vehicle moving after reset'
            report['motion'] = {'command_speed_mps': 0.5, 'command_duration_wall_s': 1.5,
                                'straight_end_pose': straight_end, 'straight_displacement_m': displacement,
                                'positive_steering_rad': 0.1, 'turn_end_pose': turn_end,
                                'positive_steering_yaw_change_rad': yaw_change,
                                'final_pose': pose(), 'final_speed_mps': state['odom'].twist.twist.linear.x}
        report['trajectory'] = trajectory
        report['scan']['ranges'] = ranges.tolist()
        report['status'] = 'passed'
        out = Path(args.output)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({k: v for k, v in report.items() if k not in ('trajectory', 'scan')}, indent=2))
    finally:
        if moved:
            spin(0.5, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
