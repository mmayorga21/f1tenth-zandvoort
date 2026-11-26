"""Physics-time attempt lifecycle, independent of ROS and the wall clock."""

import math
from uuid import uuid4


def finish_speed_limit(pose, laps, deceleration=6.0):
    """Brake on the final straight, stopping just beyond the finish gate."""
    if (laps.get('lap_count') != laps.get('target_laps', 0)-1
            or not laps.get('checkpoint_count')
            or laps.get('checkpoints_passed') != laps['checkpoint_count']):
        return math.inf
    config = laps['configuration']
    yaw = config['gate_yaw']
    dx, dy = pose[0]-config['gate_x'], pose[1]-config['gate_y']
    along = dx*math.cos(yaw)+dy*math.sin(yaw)
    lateral = -dx*math.sin(yaw)+dy*math.cos(yaw)
    if not -25.0 <= along <= 1.1 or abs(lateral) > config['gate_half_width_m']:
        return math.inf
    # The small offset permits the counter to observe the crossing. Include
    # the velocity servo's response while leaving steering and physics active.
    distance = max(0.0, 0.1-along)
    response = deceleration*0.06
    return max(0.0, math.sqrt(response*response+2*deceleration*distance)-response)


class SimulationSession:
    """Track resets, pauses and collision validity without counting laps."""

    def __init__(self, timestep):
        self.timestep = timestep
        self.run_id = str(uuid4())
        self.events = []
        self.reset_index = -1
        self.reset('startup')

    @property
    def can_step(self):
        return not self.paused and self.state in ('ready', 'running')

    @property
    def time_ns(self):
        return round(self.steps * self.timestep * 1_000_000_000)

    def snapshot(self):
        elapsed = 0 if self.start_step is None else self.steps - self.start_step
        return {
            'schema_version': 1, 'run_id': self.run_id,
            'session_id': self.session_id, 'reset_index': self.reset_index,
            'attempt_id': self.attempt_id, 'state': self.state,
            'paused': self.paused, 'physics_steps': self.steps,
            'sim_time_ns': self.time_ns,
            'elapsed_sim_s': elapsed * self.timestep,
            'attempt_valid': self.valid, 'ego_collision': self.collision,
            'end_reason': self.end_reason,
            'completion_requested': self.completion_requested,
        }

    def emit(self, event):
        self.events.append({'event': event, **self.snapshot()})

    def reset(self, reason):
        if self.reset_index >= 0:
            self.abort(reason)
        self.reset_index += 1
        self.session_id = str(uuid4())
        self.attempt_id = None
        self.steps = 0
        self.start_step = None
        self.state = 'ready'
        self.valid = None
        self.collision = False
        self.paused = False
        self.end_reason = None
        self.completion_requested = False
        self.emit('session_ready')

    def start(self):
        if self.state == 'ready' and not self.paused:
            self.attempt_id = str(uuid4())
            self.start_step = self.steps
            self.state, self.valid = 'running', True
            self.emit('attempt_started')

    def step(self, collision=False):
        if self.can_step:
            self.steps += 1
            if collision:
                self.mark_collision()

    def mark_collision(self):
        if not self.collision:
            self.collision = True
            self.state, self.valid, self.end_reason = 'collision', False, 'collision'
            self.emit('collision')

    def pause(self, paused):
        if self.paused != paused:
            self.paused = paused
            self.emit('paused' if paused else 'resumed')

    def request_completion(self, laps):
        """Accept only a valid finish measurement belonging to this attempt."""
        if (self.state != 'running' or self.valid is not True or self.paused
                or self.completion_requested):
            return False
        if any(laps.get(k) != self.snapshot()[k]
               for k in ('run_id', 'session_id', 'attempt_id')):
            return False
        target, count = laps.get('target_laps'), laps.get('lap_count')
        stamp = laps.get('sim_time_ns')
        if (type(target) is not int or target < 1 or type(count) is not int
                or count != target or laps.get('target_reached') is not True
                or laps.get('measurement_valid') is not True
                or laps.get('state') != 'running' or laps.get('pause_used') is not False
                or laps.get('paused') is not False
                or type(stamp) is not int or stamp < 0
                or not 0 <= self.time_ns-stamp <= 1_000_000_000):
            return False
        self.completion_requested = True
        return True

    def finish_if_stopped(self, speed):
        if (self.completion_requested and self.state == 'running'
                and math.isfinite(speed) and abs(speed) < 0.02):
            return self.end('target_laps_completed')
        return False

    def end(self, reason='manual_stop'):
        if self.state != 'running':
            return False
        self.state, self.end_reason = 'ended', reason
        self.emit('attempt_ended')
        return True

    def abort(self, reason):
        if self.state == 'running':
            self.state, self.valid, self.end_reason = 'ended', False, reason
            self.emit('attempt_ended')
