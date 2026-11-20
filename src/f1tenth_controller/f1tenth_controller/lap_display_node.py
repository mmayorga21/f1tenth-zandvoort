"""Read-only terminal viewer; it can join after an attempt has already ended."""
import json
import shutil
import sys
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from .lap_panel import format_panel


class LapDisplay(Node):
    def __init__(self):
        super().__init__('lap_display')
        self.latest = None
        self.received = None
        self.last_panel = None
        self.last_summary = None
        self.tty = sys.stdout.isatty()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = self.create_subscription(String, '/laps/status', self.on_status, qos)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.2, self.draw, clock=self.steady_clock)
        if self.tty:
            sys.stdout.write('\x1b[2J\x1b[?25l')
            sys.stdout.flush()

    def on_status(self, msg):
        try:
            status = json.loads(msg.data)
            format_panel(status)  # Verify display fields before retaining it.
            self.latest = status
            self.received = time.monotonic()
        except (ValueError, KeyError, TypeError):
            self.received = None

    def draw(self):
        size = shutil.get_terminal_size(fallback=(78, 26))
        if self.latest is None:
            panel = 'F1TENTH - ZANDVOORT\nWaiting for /laps/status data...'
            summary = None
        else:
            stale = self.received is None or time.monotonic()-self.received > 2.
            panel = format_panel(self.latest, size.columns, size.lines-1 if stale else size.lines)
            if stale:
                panel += '\nSTALE DATA: showing the last received status'
            summary = (self.latest['state'], self.latest['paused'],
                       self.latest['lap_count'], self.latest['reason'], stale)
        if self.tty:
            if panel != self.last_panel:
                sys.stdout.write('\x1b[H\x1b[J'+panel+'\n')
                sys.stdout.flush()
                self.last_panel = panel
        elif panel != self.last_panel and summary != self.last_summary:
            print(panel, flush=True)
            self.last_panel, self.last_summary = panel, summary

    def close(self):
        if self.tty:
            sys.stdout.write('\x1b[?25h\n')
            sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = LapDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
