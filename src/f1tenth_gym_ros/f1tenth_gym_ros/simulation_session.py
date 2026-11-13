"""Physics-time attempt lifecycle, independent of ROS and the wall clock."""

from uuid import uuid4


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

    def end(self):
        if self.state != 'running':
            return False
        self.state, self.end_reason = 'ended', 'manual_stop'
        self.emit('attempt_ended')
        return True

    def abort(self, reason):
        if self.state == 'running':
            self.state, self.valid, self.end_reason = 'ended', False, reason
            self.emit('attempt_ended')
