"""Lap measurement from simulation timestamps, independent of ROS and display."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class LapParameters:
    gate_x: float = 0.0
    gate_y: float = 0.0
    gate_yaw: float = 1.1954730407943466
    gate_half_width_m: float = 1.3
    start_radius_m: float = 0.5
    minimum_lap_distance_m: float = 200.0
    max_sample_gap_s: float = 0.25
    max_position_speed_mps: float = 25.0
    position_slack_m: float = 0.2
    target_laps: int = 10
    stop_counting_at_target: bool = False
    checkpoints: tuple = ((15., 29., 5.), (73., -4., 6.), (57., -29., 5.),
                          (48., -21., 3.), (60., -12., 5.), (1., -42., 7.))

    def validate(self):
        if not all(math.isfinite(v) for v in (self.gate_x, self.gate_y, self.gate_yaw)):
            raise ValueError('Gate pose must be finite')
        positive = (self.gate_half_width_m, self.start_radius_m,
                    self.minimum_lap_distance_m, self.max_sample_gap_s,
                    self.max_position_speed_mps, self.position_slack_m)
        if not all(math.isfinite(v) and v > 0 for v in positive):
            raise ValueError('Measurement limits must be finite and positive')
        if not isinstance(self.target_laps, int) or self.target_laps < 1:
            raise ValueError('target_laps must be a positive integer')
        if not self.checkpoints:
            raise ValueError('Ordered checkpoints are required')
        for region in self.checkpoints:
            if len(region) != 3 or not all(math.isfinite(v) for v in region) or region[2] <= 0:
                raise ValueError('Each checkpoint needs finite x, y and positive radius')


class LapCounter:
    """Track one source session; a reset clears displayed laps, not event history."""
    def __init__(self, parameters=None):
        self.p = parameters or LapParameters()
        self.p.validate()
        self.events = []
        self.identity = None
        self._clear()

    def _clear(self):
        self.state = 'waiting'
        self.reason = 'waiting_for_simulator'
        self.valid = None
        self.ready_seen = False
        self.attempt_id = None
        self.start_ns = None
        self.lap_start_ns = None
        self.now_ns = 0
        self.paused = False
        self.pause_used = False
        self.laps = []
        self.next_checkpoint = 0
        self.distance_m = 0.0
        self.previous = None
        self.invalid_at_ns = None

    def snapshot(self):
        measurement_time = self.now_ns if self.invalid_at_ns is None else self.invalid_at_ns
        capped = self.p.stop_counting_at_target and len(self.laps) >= self.p.target_laps
        if capped:
            measurement_time = min(measurement_time, self.laps[-1]['finish_sim_time_ns'])
        elapsed = 0 if self.start_ns is None else max(0, measurement_time-self.start_ns)
        current = 0 if self.lap_start_ns is None else max(0, measurement_time-self.lap_start_ns)
        times = [lap['duration_s'] for lap in self.laps]
        return {
            'schema_version': 1,
            'run_id': None if self.identity is None else self.identity[0],
            'session_id': None if self.identity is None else self.identity[1],
            'attempt_id': self.attempt_id, 'state': self.state,
            'reason': self.reason, 'measurement_valid': self.valid,
            'paused': self.paused, 'pause_used': self.pause_used,
            'lap_count': len(self.laps), 'target_laps': self.p.target_laps,
            'target_reached': self.valid is True and len(self.laps) >= self.p.target_laps,
            'sim_time_ns': self.now_ns, 'elapsed_sim_s': elapsed/1e9,
            'current_lap_time_s': current/1e9,
            'last_lap_time_s': times[-1] if times else None,
            'best_lap_time_s': min(times) if times else None,
            'completed_time_s': sum(times), 'lap_times_s': times,
            'checkpoints_passed': self.next_checkpoint,
            'checkpoint_count': len(self.p.checkpoints),
            'distance_since_finish_m': self.distance_m,
        }

    def emit(self, event, **extra):
        self.events.append({'event': event, **self.snapshot(), **extra})

    def invalidate(self, reason):
        if self.state != 'invalid':
            self.valid = False
            self.invalid_at_ns = self.now_ns
            self.state, self.reason = 'invalid', reason
            self.emit('measurement_invalid')

    def on_status(self, status):
        identity = (status['run_id'], status['session_id'])
        stamp = status['sim_time_ns']
        source_state = status['state']
        if (not all(isinstance(v, str) and v for v in identity)
                or not isinstance(stamp, int) or stamp < 0
                or source_state not in ('ready', 'running', 'ended', 'collision')):
            raise ValueError('Invalid simulator status')
        if identity != self.identity:
            if self.state == 'running':
                self.invalidate('session_reset')
            self._clear()
            self.identity = identity
            self.now_ns = stamp
            self.state, self.reason = 'ready', 'waiting_for_attempt'
            self.ready_seen = source_state == 'ready'
            self.emit('measurement_session_ready')
        elif stamp < self.now_ns:
            self.invalidate('simulation_time_reversed')
        self.now_ns = stamp
        self.paused = bool(status['paused'])
        if source_state == 'ready' and self.state != 'invalid':
            self.ready_seen = True
        source_attempt = status.get('attempt_id')
        if source_attempt is not None and self.attempt_id is None:
            self.attempt_id = source_attempt
            if not self.ready_seen or source_state != 'running':
                self.invalidate('attempt_start_not_observed')
            elif self.state != 'invalid':
                elapsed = float(status['elapsed_sim_s'])
                if not math.isfinite(elapsed) or elapsed < 0:
                    raise ValueError('Invalid attempt time')
                start = stamp-round(elapsed*1e9)
                if start < 0:
                    raise ValueError('Attempt starts before the simulation epoch')
                self.start_ns = self.lap_start_ns = start
                self.previous = None
                self.state, self.reason, self.valid = 'running', 'measuring', True
                self.emit('measurement_started')
        elif source_attempt is not None and source_attempt != self.attempt_id:
            self.invalidate('attempt_changed_without_reset')
        self.pause_used = self.pause_used or (self.start_ns is not None and self.paused)
        if status.get('ego_collision') or source_state == 'collision':
            self.invalidate('collision')
        elif source_state == 'ended':
            if status.get('attempt_valid') is not True:
                self.invalidate(status.get('end_reason') or 'attempt_interrupted')
            elif self.state == 'running':
                self.state, self.reason = 'ended', status.get('end_reason') or 'attempt_ended'
                self.emit('measurement_ended')

    def _coordinates(self, x, y):
        dx, dy = x-self.p.gate_x, y-self.p.gate_y
        cosine, sine = math.cos(self.p.gate_yaw), math.sin(self.p.gate_yaw)
        return dx*cosine+dy*sine, -dx*sine+dy*cosine

    def on_pose(self, stamp, x, y):
        if self.valid is not True or self.state not in ('running', 'ended'):
            return
        if not isinstance(stamp, int) or not all(math.isfinite(v) for v in (x, y)):
            self.invalidate('invalid_odometry')
            return
        if stamp < self.start_ns or stamp > self.now_ns:
            return  # The ROS adapter buffers samples until source status catches up.
        if self.previous is None:
            if (stamp-self.start_ns)/1e9 > self.p.max_sample_gap_s:
                self.invalidate('odometry_missing_at_start')
            elif math.hypot(x-self.p.gate_x, y-self.p.gate_y) > self.p.start_radius_m:
                self.invalidate('start_outside_finish_area')
            else:
                self.previous = (stamp, x, y)
            return
        old_stamp, old_x, old_y = self.previous
        if stamp == old_stamp:
            return
        if stamp < old_stamp:
            self.invalidate('odometry_time_reversed')
            return
        dt = (stamp-old_stamp)/1e9
        segment = math.hypot(x-old_x, y-old_y)
        if dt > self.p.max_sample_gap_s:
            self.invalidate('odometry_gap')
            return
        if segment > self.p.max_position_speed_mps*dt+self.p.position_slack_m:
            self.invalidate('position_jump')
            return
        if self.p.stop_counting_at_target and len(self.laps) >= self.p.target_laps:
            self.previous = (stamp, x, y)
            return  # Keep checking odometry while retaining only the target lap times.
        self.distance_m += segment
        if self.next_checkpoint < len(self.p.checkpoints):
            cx, cy, radius = self.p.checkpoints[self.next_checkpoint]
            if math.hypot(x-cx, y-cy) <= radius:
                self.next_checkpoint += 1
        old_signed, old_lateral = self._coordinates(old_x, old_y)
        signed, lateral = self._coordinates(x, y)
        forward = old_signed < 0 <= signed
        reverse = old_signed >= 0 > signed
        if forward or reverse:
            fraction = -old_signed/(signed-old_signed)
            crossing_lateral = old_lateral+fraction*(lateral-old_lateral)
            if abs(crossing_lateral) <= self.p.gate_half_width_m:
                if reverse:
                    self.next_checkpoint = 0
                    self.distance_m = (1-fraction)*segment
                else:
                    distance_at_crossing = self.distance_m-(1-fraction)*segment
                    if (self.next_checkpoint == len(self.p.checkpoints)
                            and distance_at_crossing >= self.p.minimum_lap_distance_m):
                        cross_ns = old_stamp+round(fraction*(stamp-old_stamp))
                        lap = {'number': len(self.laps)+1,
                               'start_sim_time_ns': self.lap_start_ns,
                               'finish_sim_time_ns': cross_ns,
                               'duration_s': (cross_ns-self.lap_start_ns)/1e9,
                               'distance_m': distance_at_crossing}
                        self.laps.append(lap)
                        self.lap_start_ns = cross_ns
                        self.next_checkpoint = 0
                        self.distance_m = (1-fraction)*segment
                        self.emit('lap_completed', lap=lap.copy())
        self.previous = (stamp, x, y)
