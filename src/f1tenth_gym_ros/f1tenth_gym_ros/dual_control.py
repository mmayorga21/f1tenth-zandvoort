"""Shared start barrier and independent command/controller freshness checks."""
import math


def following_pose(pose, distance):
    """Place the second center ahead along the initial straight's heading."""
    x, y, yaw = pose
    if not math.isfinite(distance) or distance <= 0:
        raise ValueError('Starting separation must be positive and finite')
    return [x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw]


class DualControlGate:
    def __init__(self, vehicles, drive_timeout=.3, status_timeout=.5):
        self.vehicles = tuple(vehicles)
        if len(self.vehicles) != 2 or len(set(self.vehicles)) != 2:
            raise ValueError('Two distinct vehicle identifiers are required')
        self.drive_timeout, self.status_timeout = drive_timeout, status_timeout
        self.reset(None)

    def reset(self, identity):
        self.identity = identity
        self.released = False
        self.drive_received = dict.fromkeys(self.vehicles, -math.inf)
        self.status_received = dict.fromkeys(self.vehicles, -math.inf)
        self.armed = dict.fromkeys(self.vehicles, False)

    def on_drive(self, vehicle, now):
        self.drive_received[vehicle] = now

    def on_controller(self, vehicle, status, now):
        self.status_received[vehicle] = now
        config = status.get('configuration') or {}
        self.armed[vehicle] = (tuple(status.get('identity') or ()) == self.identity
                               and status.get('armed') is True and isinstance(config, dict)
                               and config.get('vehicle_id', vehicle) == vehicle)

    def health(self, now, publishers):
        for vehicle in self.vehicles:
            if publishers.get(vehicle) != 1:
                return vehicle + '_drive_publisher_count'
            if now-self.drive_received[vehicle] > self.drive_timeout:
                return vehicle + '_drive_timeout'
            if now-self.status_received[vehicle] > self.status_timeout:
                return vehicle + '_controller_timeout'
            if not self.armed[vehicle]:
                return vehicle + '_controller_disabled'
        return None
