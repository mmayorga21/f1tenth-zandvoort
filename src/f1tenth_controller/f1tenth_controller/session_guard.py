"""Fail-closed checks for a consecutive-lap experiment; never drives the car."""
import json
import math


class SessionError(ValueError):
    """An observation cannot certify the requested uninterrupted session."""


def identity(status):
    return tuple(status.get(k) for k in ('run_id', 'session_id', 'attempt_id'))


def fingerprints(simulator, laps, controller):
    return (
        json.dumps(simulator['configuration'], sort_keys=True),
        json.dumps(laps['configuration'], sort_keys=True), laps['code_sha256'],
        controller['config_sha256'], controller['code_sha256'],
    )


class SessionGuard:
    def __init__(self, target_laps, simulator, laps, controller,
                 max_stale_s=2.0, max_lap_sim_s=400.0):
        if target_laps < 1 or max_stale_s <= 0 or max_lap_sim_s <= 0:
            raise ValueError('Session limits must be positive')
        self.target = target_laps
        self.expected_identity = identity(simulator)
        if not all(self.expected_identity) or identity(laps) != self.expected_identity:
            raise SessionError('attempt_identity_not_aligned')
        if laps['lap_count'] != 0:
            raise SessionError('session_did_not_start_at_zero')
        if laps['target_laps'] < target_laps:
            raise SessionError('tracker_target_is_too_small')
        self.expected_fingerprints = fingerprints(simulator, laps, controller)
        self.max_stale = max_stale_s
        self.max_lap_time = max_lap_sim_s
        self.last_count = 0
        self.last_elapsed = -1.0

    def observe(self, simulator, laps, controller, received_at, now):
        if any(now-received_at.get(topic, -math.inf) > self.max_stale
               for topic in ('sim', 'laps', 'controller')):
            raise SessionError('status_timeout')
        if identity(simulator) != self.expected_identity or identity(laps) != self.expected_identity:
            raise SessionError('attempt_identity_changed')
        if fingerprints(simulator, laps, controller) != self.expected_fingerprints:
            raise SessionError('effective_configuration_changed')
        if simulator['ego_collision'] or simulator['attempt_valid'] is not True:
            raise SessionError('simulator_attempt_invalid')
        automatic_finish = (simulator.get('completion_requested') is True
                            and simulator.get('end_reason') in (None, 'target_laps_completed'))
        allowed_states = ('running', 'ended') if automatic_finish else ('running',)
        if simulator['state'] not in allowed_states or laps['state'] not in allowed_states:
            raise SessionError('attempt_not_running')
        if simulator['paused'] or laps['paused'] or laps['pause_used']:
            raise SessionError('pause_used_during_session')
        if laps['measurement_valid'] is not True:
            raise SessionError('lap_measurement_invalid: '+str(laps['reason']))
        if not controller['armed'] and not automatic_finish:
            raise SessionError('controller_disabled')
        count, times = laps['lap_count'], laps['lap_times_s']
        if not isinstance(count, int) or count < self.last_count or count > self.last_count+1:
            raise SessionError('unexpected_lap_count_change')
        if count > self.target or len(times) != count:
            raise SessionError('lap_count_and_times_disagree')
        if not all(math.isfinite(t) and 0 < t <= self.max_lap_time for t in times):
            raise SessionError('invalid_completed_lap_time')
        if (not math.isfinite(laps['completed_time_s'])
                or abs(laps['completed_time_s']-sum(times)) > 1e-6):
            raise SessionError('completed_time_sum_mismatch')
        if count and (laps['last_lap_time_s'] != times[-1] or laps['best_lap_time_s'] != min(times)):
            raise SessionError('last_or_best_lap_mismatch')
        elapsed, current = laps['elapsed_sim_s'], laps['current_lap_time_s']
        if not all(math.isfinite(t) and t >= 0 for t in (elapsed, current)):
            raise SessionError('invalid_elapsed_time')
        if abs(elapsed-laps['completed_time_s']-current) > 1e-6:
            raise SessionError('elapsed_and_lap_times_disagree')
        if elapsed < self.last_elapsed or current > self.max_lap_time:
            raise SessionError('lap_time_limit_or_time_reversal')
        self.last_count, self.last_elapsed = count, elapsed
        return count == self.target
