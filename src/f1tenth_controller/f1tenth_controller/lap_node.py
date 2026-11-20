"""ROS lap tracker and terminal panel, with generated logs outside the repo."""
from collections import deque
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import math
import shutil
import sys
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from .laps import LapCounter, LapParameters
from .lap_panel import format_panel


class LapTracker(Node):
    def __init__(self):
        super().__init__('lap_tracker')
        defaults = asdict(LapParameters())
        def declare_fixed(name, value):
            # rclpy fills descriptor.name/type in place; each parameter needs
            # its own descriptor rather than sharing a mutable instance.
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        flat_regions = [value for region in defaults.pop('checkpoints') for value in region]
        for name, value in defaults.items():
            declare_fixed(name, value)
        declare_fixed('checkpoint_regions', flat_regions)
        declare_fixed('data_timeout_s', 1.0)
        declare_fixed('expected_frame', 'map')
        declare_fixed('display', True)
        declare_fixed('vehicle_id', '')
        declare_fixed('event_log_path', '~/.ros/f1tenth_zandvoort/laps.jsonl')
        values = {name: self.get_parameter(name).value for name in defaults}
        regions = self.get_parameter('checkpoint_regions').value
        if len(regions) % 3:
            raise ValueError('checkpoint_regions must contain x, y, radius triples')
        values['checkpoints'] = tuple(tuple(regions[i:i+3]) for i in range(0, len(regions), 3))
        self.counter = LapCounter(LapParameters(**values))
        self.timeout = float(self.get_parameter('data_timeout_s').value)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('data_timeout_s must be positive')
        self.expected_frame = self.get_parameter('expected_frame').value
        self.display = self.get_parameter('display').value
        self.tty = self.display and sys.stdout.isatty()
        self.last_printed = None
        self.last_panel = None
        self.pending = deque(maxlen=256)
        self.last_status_received = time.monotonic()
        self.last_pose_received = time.monotonic()
        self.latest_source = {}
        self.configuration = asdict(self.counter.p)
        self.configuration.update(data_timeout_s=self.timeout, expected_frame=self.expected_frame)
        self.vehicle_id = self.get_parameter('vehicle_id').value
        if self.vehicle_id:
            self.configuration['vehicle_id'] = self.vehicle_id
        files = [Path(__file__), Path(__file__).with_name('laps.py'),
                 Path(__file__).with_name('lap_panel.py')]
        self.code_sha256 = hashlib.sha256(b''.join(p.read_bytes() for p in files)).hexdigest()
        path = Path(self.get_parameter('event_log_path').value).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self.log = path.open('a', buffering=1)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, '/laps/status', retained)
        self.event_publisher = self.create_publisher(String, '/laps/events', 100)
        self.status_subscription = self.create_subscription(
            String, '/sim/status', self.on_status, retained)
        self.pose_subscription = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.on_pose, 100)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.2, self.update, clock=self.steady_clock)
        if self.tty:
            sys.stdout.write('\x1b[2J\x1b[?25l')
            sys.stdout.flush()

    def on_status(self, msg):
        try:
            status = json.loads(msg.data)
            identity = (status['run_id'], status['session_id'])
            if identity != self.counter.identity:
                self.pending.clear()
            self.latest_source = status
            self.counter.on_status(status)
            self.last_status_received = time.monotonic()
            self.flush_poses()
            self.check_simulated_gap()
            self.flush_events()
        except (ValueError, KeyError, TypeError, OverflowError):
            self.counter.invalidate('invalid_simulator_status')

    def on_pose(self, msg):
        if self.counter.identity is None:
            return
        self.last_pose_received = time.monotonic()
        if msg.header.frame_id != self.expected_frame:
            if self.counter.state == 'running':
                self.counter.invalidate('unexpected_odometry_frame')
            return
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        position = msg.pose.pose.position
        if len(self.pending) == self.pending.maxlen and self.counter.state == 'running':
            self.counter.invalidate('odometry_buffer_overrun')
        self.pending.append((stamp, position.x, position.y))
        self.flush_poses()

    def flush_poses(self):
        # Status and odometry are separate topics. Process a pose only after a
        # status watermark confirms its simulation time and session state.
        remaining = deque(maxlen=self.pending.maxlen)
        max_ahead = round(self.counter.p.max_sample_gap_s*1e9)
        for stamp, x, y in self.pending:
            if stamp <= self.counter.now_ns:
                self.counter.on_pose(stamp, x, y)
            elif stamp-self.counter.now_ns <= max_ahead:
                remaining.append((stamp, x, y))
        self.pending = remaining

    def check_simulated_gap(self):
        counter = self.counter
        if counter.valid is True and counter.state == 'running' and not counter.paused:
            last = counter.start_ns if counter.previous is None else counter.previous[0]
            if (counter.now_ns-last)/1e9 > counter.p.max_sample_gap_s:
                reason = 'odometry_missing_at_start' if counter.previous is None else 'odometry_gap'
                counter.invalidate(reason)

    def flush_events(self, publish=True):
        for event in self.counter.events:
            event.update(recorded_at_utc=datetime.now(timezone.utc).isoformat(),
                         configuration=self.configuration, code_sha256=self.code_sha256,
                         simulator_configuration=self.latest_source.get('configuration'),
                         controller=self.latest_source.get('controller'))
            if self.vehicle_id:
                event['vehicle_id'] = self.vehicle_id
            line = json.dumps(event)
            self.log.write(line+'\n')
            if publish:
                self.event_publisher.publish(String(data=line))
        self.counter.events.clear()

    def update(self):
        now = time.monotonic()
        if self.counter.valid is True and self.counter.state == 'running':
            if now-self.last_status_received > self.timeout:
                self.counter.invalidate('source_status_timeout')
            elif not self.counter.paused and now-self.last_pose_received > self.timeout:
                self.counter.invalidate('odometry_timeout')
        self.flush_events()
        status = self.counter.snapshot()
        status.update(configuration=self.configuration, code_sha256=self.code_sha256)
        if self.vehicle_id:
            status['vehicle_id'] = self.vehicle_id
            status['awaiting_joint_start'] = self.latest_source.get('awaiting_joint_start', False)
        self.publisher.publish(String(data=json.dumps(status)))
        if self.display:
            size = shutil.get_terminal_size(fallback=(78, 26))
            panel = format_panel(status, size.columns, size.lines)
            if self.tty:
                if panel != self.last_panel:
                    sys.stdout.write('\x1b[H\x1b[J'+panel+'\n')
                    sys.stdout.flush()
                    self.last_panel = panel
            else:
                # Logs without a TTY stay concise; a real terminal redraws live.
                key = (status['state'], status['paused'], status['lap_count'], status['reason'])
                if key != self.last_printed:
                    print(panel, flush=True)
                    self.last_printed = key

    def close(self):
        if self.counter.state == 'running':
            self.counter.invalidate('tracker_shutdown')
        self.flush_events(publish=rclpy.ok())
        self.log.close()
        if self.tty:
            sys.stdout.write('\x1b[?25h\n')
            sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = LapTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
