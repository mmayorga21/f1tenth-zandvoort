"""Lifecycle invariants required for trustworthy attempt measurement."""

import unittest

from f1tenth_gym_ros.simulation_session import SimulationSession


class SessionTests(unittest.TestCase):
    def test_idle_time_does_not_count_as_attempt_time(self):
        session = SimulationSession(0.01)
        for _ in range(100):
            session.step()
        self.assertEqual(session.time_ns, 1_000_000_000)
        self.assertEqual(session.snapshot()['elapsed_sim_s'], 0)
        session.start()
        for _ in range(123):
            session.step()
        self.assertAlmostEqual(session.snapshot()['elapsed_sim_s'], 1.23)

    def test_pause_freezes_physics_and_elapsed_time(self):
        session = SimulationSession(0.01)
        session.start()
        session.step()
        session.pause(True)
        before = session.snapshot()
        for _ in range(100):
            session.step()
        self.assertEqual(before, session.snapshot())
        session.pause(False)
        session.step()
        self.assertEqual(session.steps, 2)

    def test_reset_invalidates_old_attempt_and_changes_identity(self):
        session = SimulationSession(0.01)
        session.start()
        session.step()
        old = session.snapshot()
        session.reset('initialpose')
        event = session.events[-2]
        self.assertEqual(event['attempt_id'], old['attempt_id'])
        self.assertFalse(event['attempt_valid'])
        self.assertEqual(event['end_reason'], 'initialpose')
        self.assertNotEqual(old['session_id'], session.session_id)
        self.assertIsNone(session.attempt_id)
        self.assertEqual(session.time_ns, 0)

    def test_collision_cannot_be_cleared_by_resume_or_start(self):
        session = SimulationSession(0.01)
        session.start()
        session.step(collision=True)
        for _ in range(100):
            session.start()
            session.pause(False)
            session.step()
        self.assertEqual(session.state, 'collision')
        self.assertEqual(session.steps, 1)
        self.assertFalse(session.valid)
        session.reset('reset_service')
        self.assertFalse(session.collision)
        self.assertEqual(session.state, 'ready')

    def test_manual_end_preserves_duration_without_claiming_race_success(self):
        session = SimulationSession(0.01)
        session.start()
        session.step()
        self.assertTrue(session.end())
        session.step()
        self.assertEqual(session.steps, 1)
        self.assertEqual(session.end_reason, 'manual_stop')
        self.assertFalse(session.end())

    def test_process_restart_and_shutdown_are_distinguishable(self):
        a, b = SimulationSession(0.01), SimulationSession(0.01)
        self.assertNotEqual(a.run_id, b.run_id)
        a.start()
        a.abort('shutdown')
        self.assertFalse(a.valid)
        self.assertEqual(a.end_reason, 'shutdown')

    def completion_status(self, session):
        return {**session.snapshot(), 'state': 'running', 'target_laps': 10,
                'lap_count': 10, 'target_reached': True, 'measurement_valid': True,
                'pause_used': False}

    def test_automatic_finish_waits_for_braking_and_freezes_physics(self):
        session = SimulationSession(.01)
        session.start()
        session.step()
        self.assertTrue(session.request_completion(self.completion_status(session)))
        for speed in [6., .03, float('nan')]:
            self.assertFalse(session.finish_if_stopped(speed))
        self.assertEqual(session.state, 'running')
        self.assertTrue(session.finish_if_stopped(.01))
        self.assertEqual(session.end_reason, 'target_laps_completed')
        self.assertTrue(session.valid)
        before = session.snapshot()
        session.step()
        self.assertEqual(before, session.snapshot())
        session.reset('reset_service')
        self.assertFalse(session.completion_requested)

    def test_stale_invalid_incomplete_or_foreign_finish_is_ignored(self):
        for change in [{'attempt_id': 'previous'}, {'run_id': 'other'},
                       {'session_id': 'old'}, {'lap_count': 9}, {'target_laps': 0},
                       {'target_reached': False}, {'measurement_valid': False},
                       {'pause_used': True}, {'paused': True}, {'state': 'invalid'},
                       {'sim_time_ns': -1}, {'sim_time_ns': 10**12}]:
            with self.subTest(change=change):
                session = SimulationSession(.01)
                session.start()
                self.assertFalse(session.request_completion(
                    {**self.completion_status(session), **change}))
                self.assertFalse(session.finish_if_stopped(0.))
        session = SimulationSession(.01)
        session.start()
        old_status = self.completion_status(session)
        for _ in range(101):
            session.step()
        self.assertFalse(session.request_completion(old_status))

    def test_collision_during_braking_cannot_become_success(self):
        session = SimulationSession(.01)
        session.start()
        session.request_completion(self.completion_status(session))
        session.step(collision=True)
        self.assertFalse(session.finish_if_stopped(0.))
        self.assertEqual(session.end_reason, 'collision')
        self.assertFalse(session.valid)

    def test_joint_finish_requires_both_fresh_valid_targets(self):
        session = SimulationSession(.01)
        session.start()
        measurements = {v: {**self.completion_status(session), 'vehicle_id': v} for v in ('ego', 'opp')}
        self.assertFalse(session.request_joint_completion({'ego': measurements['ego']}, ('ego', 'opp'), 10))
        self.assertFalse(session.completion_requested)
        for change in [{'vehicle_id': 'ego'}, {'target_laps': 9}, {'lap_count': 9},
                       {'session_id': 'old'}, {'measurement_valid': False},
                       {'pause_used': True}, {'sim_time_ns': 10**12}]:
            bad = {**measurements, 'opp': {**measurements['opp'], **change}}
            self.assertFalse(session.request_joint_completion(bad, ('ego', 'opp'), 10), change)
        # The first finisher's old notification must be refreshed, not latched forever.
        for _ in range(101): session.step()
        self.assertFalse(session.request_joint_completion(measurements, ('ego', 'opp'), 10))
        for v in measurements: measurements[v]['sim_time_ns'] = session.time_ns
        self.assertTrue(session.request_joint_completion(measurements, ('ego', 'opp'), 10))
        self.assertFalse(session.finish_if_stopped(.1))
        self.assertTrue(session.finish_if_stopped(0.))
        self.assertTrue(session.valid)
        self.assertEqual(session.end_reason, 'target_laps_completed')
