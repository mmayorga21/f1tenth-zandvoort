"""Explicit activation and session identity, independently testable."""
import math


class ControlGate:
    def __init__(self, scan_timeout=0.3, status_timeout=0.5):
        self.scan_timeout = scan_timeout
        self.status_timeout = status_timeout
        self.armed = False
        self.reason = 'disabled_at_startup'
        self.identity = None
        self.status = None
        self.status_received = -math.inf
        self.scan_received = -math.inf

    def disarm(self, reason):
        self.armed, self.reason = False, reason

    def on_status(self, status, now):
        identity = (status['run_id'], status['session_id'])
        if identity != self.identity:
            self.disarm('new_session_requires_enable')
            self.identity = identity
            self.scan_received = -math.inf
        self.status, self.status_received = status, now
        if status.get('stopping_requested'):
            self.disarm('manual_stop')
        if status.get('completion_requested'):
            self.disarm('target_laps_completed')
        if status['paused'] or status['state'] not in ('ready', 'running') or status['ego_collision']:
            self.disarm('simulator_paused_or_terminal')

    def health(self, now, publishers):
        if publishers != 1:
            return 'another_drive_publisher'
        if self.status is None or now-self.status_received > self.status_timeout:
            return 'stale_simulator_status'
        if self.status.get('stopping_requested'):
            return 'manual_stop'
        if self.status.get('completion_requested'):
            return 'target_laps_completed'
        if (self.status['paused'] or self.status['state'] not in ('ready', 'running')
                or self.status['ego_collision']):
            return 'simulator_paused_or_terminal'
        if now-self.scan_received > self.scan_timeout:
            return 'stale_scan'
        return None

    def enable(self, now, publishers):
        reason = self.health(now, publishers)
        if reason:
            self.disarm(reason)
            return False
        self.armed, self.reason = True, 'enabled'
        return True
