"""A ten-lap success must survive identity, time and configuration checks."""
import unittest

from f1tenth_controller.session_guard import SessionGuard, SessionError


def observations():
    ids = {'run_id': 'run', 'session_id': 'session', 'attempt_id': 'attempt'}
    sim = {**ids, 'configuration': {'map': 'Zandvoort'}, 'state': 'running',
           'ego_collision': False, 'attempt_valid': True, 'paused': False}
    laps = {**ids, 'configuration': {'gate_x': 0.}, 'code_sha256': 'counter',
            'state': 'running', 'paused': False, 'pause_used': False,
            'measurement_valid': True, 'reason': 'measuring', 'lap_count': 0,
            'target_laps': 10, 'lap_times_s': [], 'completed_time_s': 0.,
            'last_lap_time_s': None, 'best_lap_time_s': None,
            'elapsed_sim_s': 0., 'current_lap_time_s': 0.}
    controller = {'armed': True, 'config_sha256': 'profile', 'code_sha256': 'controller'}
    return sim, laps, controller


class SessionGuardTests(unittest.TestCase):
    def setUp(self):
        self.sim, self.laps, self.controller = observations()
        self.guard = SessionGuard(10, self.sim, self.laps, self.controller)

    def observe(self, now=0., received=None):
        return self.guard.observe(self.sim, self.laps, self.controller,
                                  received or dict.fromkeys(('sim', 'laps', 'controller'), now), now)

    def completed(self, count):
        times = [321.0]*count
        self.laps.update(lap_count=count, lap_times_s=times, completed_time_s=sum(times),
                         last_lap_time_s=times[-1] if times else None,
                         best_lap_time_s=min(times) if times else None,
                         elapsed_sim_s=sum(times), current_lap_time_s=0.)

    def test_success_requires_all_ten_laps(self):
        for count in range(11):
            self.completed(count)
            self.assertEqual(self.observe(), count == 10)

    def test_duplicate_status_does_not_complete_a_lap(self):
        for _ in range(20):
            self.assertFalse(self.observe())
        self.assertEqual(self.guard.last_count, 0)

    def test_collision_or_invalid_measurement_cannot_pass(self):
        for field, value in [('ego_collision', True), ('attempt_valid', False)]:
            with self.subTest(field=field):
                self.setUp(); self.sim[field] = value
                with self.assertRaises(SessionError): self.observe()
        self.setUp(); self.laps['measurement_valid'] = False
        with self.assertRaises(SessionError): self.observe()

    def test_pause_is_rejected_even_after_resume(self):
        self.laps['pause_used'] = True
        with self.assertRaises(SessionError): self.observe()

    def test_restarting_either_identity_is_rejected(self):
        for source in ['sim', 'laps']:
            self.setUp()
            getattr(self, source)['session_id'] = 'reset'
            with self.assertRaises(SessionError): self.observe()

    def test_controller_loss_or_stale_status_is_rejected(self):
        self.controller['armed'] = False
        with self.assertRaises(SessionError): self.observe()
        self.setUp()
        with self.assertRaises(SessionError):
            self.observe(now=3., received=dict.fromkeys(('sim', 'laps', 'controller'), 0.))

    def test_changed_effective_configuration_is_rejected(self):
        self.controller['config_sha256'] = 'faster'
        with self.assertRaises(SessionError): self.observe()

    def test_skipped_or_reversed_counter_is_rejected(self):
        self.completed(2)
        with self.assertRaises(SessionError): self.observe()
        self.setUp(); self.completed(1); self.observe(); self.completed(0)
        with self.assertRaises(SessionError): self.observe()

    def test_sum_and_best_time_must_match_recorded_laps(self):
        self.completed(1); self.laps['completed_time_s'] = 320.
        with self.assertRaises(SessionError): self.observe()
        self.setUp(); self.completed(1); self.laps['best_lap_time_s'] = 1.
        with self.assertRaises(SessionError): self.observe()

    def test_stalled_or_reversed_clock_is_rejected(self):
        self.laps['current_lap_time_s'] = 401.
        with self.assertRaises(SessionError): self.observe()
        self.setUp(); self.laps.update(elapsed_sim_s=3., current_lap_time_s=3.); self.observe()
        self.laps.update(elapsed_sim_s=2., current_lap_time_s=2.)
        with self.assertRaises(SessionError): self.observe()

    def test_nan_total_or_inconsistent_current_time_is_rejected(self):
        self.laps['completed_time_s'] = float('nan')
        with self.assertRaises(SessionError): self.observe()
        self.setUp(); self.laps['current_lap_time_s'] = 1.
        with self.assertRaises(SessionError): self.observe()

    def test_nonzero_initial_count_is_not_a_new_session(self):
        self.completed(1)
        with self.assertRaises(SessionError):
            SessionGuard(10, self.sim, self.laps, self.controller)
