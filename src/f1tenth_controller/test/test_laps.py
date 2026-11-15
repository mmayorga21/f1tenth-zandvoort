"""Lap counting, interpolation, session validity and terminal formatting."""
from dataclasses import replace
import unittest

from f1tenth_controller.laps import LapCounter, LapParameters
from f1tenth_controller.lap_panel import format_panel, format_time


class LapTests(unittest.TestCase):
    def setUp(self):
        self.parameters = LapParameters(
            gate_yaw=0., minimum_lap_distance_m=8., max_sample_gap_s=2.,
            checkpoints=((2., 0., .3), (2., 2., .3), (-2., 2., .3)), target_laps=2)
        self.counter = LapCounter(self.parameters)
        self.status(0., state='ready')
        self.status(0.)
        self.pose(0., 0., 0.)

    def status(self, seconds, state='running', session='one', paused=False, valid=True):
        self.counter.on_status({
            'run_id': 'process', 'session_id': session, 'state': state,
            'sim_time_ns': round(seconds*1e9), 'paused': paused,
            'attempt_id': None if state == 'ready' else 'attempt-'+session,
            'elapsed_sim_s': seconds, 'attempt_valid': valid,
            'ego_collision': state == 'collision', 'end_reason': 'manual_stop'})

    def pose(self, seconds, x, y):
        self.status(seconds)
        self.counter.on_pose(round(seconds*1e9), x, y)

    def circuit(self, offset=0.):
        for t, x, y in [(1., 2., 0.), (2., 2., 2.), (3., -2., 2.),
                        (4., -1., 0.), (5., 1., 0.)]:
            self.pose(offset+t, x, y)

    def test_start_and_local_finish_oscillation_do_not_count(self):
        for t, x in [(1., -1.), (2., 1.), (3., -1.), (4., 1.)]:
            self.pose(t, x, 0.)
        self.assertEqual(self.counter.snapshot()['lap_count'], 0)

    def test_crossing_interpolates_time_and_accepts_exactly_one_lap(self):
        self.circuit()
        result = self.counter.snapshot()
        self.assertEqual(result['lap_count'], 1)
        self.assertEqual(result['last_lap_time_s'], 4.5)
        self.assertEqual(result['completed_time_s'], 4.5)
        self.assertEqual(result['current_lap_time_s'], .5)
        self.assertEqual(result['checkpoints_passed'], 0)
        self.pose(5.1, -1., 0.)
        self.pose(5.2, 1., 0.)
        self.assertEqual(self.counter.snapshot()['lap_count'], 1)

    def test_multiple_laps_keep_last_best_total_and_target(self):
        self.circuit()
        self.circuit(5.)
        self.circuit(10.)
        result = self.counter.snapshot()
        self.assertEqual(result['lap_times_s'], [4.5, 5., 5.])
        self.assertEqual(result['best_lap_time_s'], 4.5)
        self.assertEqual(result['last_lap_time_s'], 5.)
        self.assertEqual(result['completed_time_s'], 14.5)
        self.assertTrue(result['target_reached'])

    def test_reverse_crossing_clears_progress(self):
        for t, x, y in [(1., 2., 0.), (2., 2., 2.), (3., -2., 2.),
                        (4., 1., 2.), (5., 1., 0.), (6., -1., 0.), (7., 1., 0.)]:
            self.pose(t, x, y)
        self.assertEqual(self.counter.snapshot()['lap_count'], 0)
        self.assertEqual(self.counter.next_checkpoint, 0)

    def test_finish_outside_segment_is_not_counted(self):
        self.pose(1., 2., 0.)
        self.pose(2., 2., 2.)
        self.pose(3., -2., 2.)
        self.pose(4., 1., 2.)
        self.assertEqual(self.counter.snapshot()['lap_count'], 0)

    def test_wrong_checkpoint_order_does_not_complete_a_lap(self):
        for t, x, y in [(1., 2., 2.), (2., -2., 2.), (3., 2., 0.),
                        (4., -1., 0.), (5., 1., 0.)]:
            self.pose(t, x, y)
        self.assertEqual(self.counter.snapshot()['lap_count'], 0)

    def test_insufficient_distance_is_rejected(self):
        self.counter = LapCounter(replace(self.parameters, minimum_lap_distance_m=100.))
        self.status(0., state='ready'); self.status(0.); self.pose(0., 0., 0.)
        self.circuit()
        self.assertEqual(self.counter.snapshot()['lap_count'], 0)

    def test_pause_does_not_add_wall_time_or_new_laps(self):
        self.pose(1., 2., 0.)
        self.status(1., paused=True)
        before = self.counter.snapshot()
        for _ in range(50):
            self.status(1., paused=True)
        after = self.counter.snapshot()
        self.assertEqual(before, after)
        self.status(1.)
        self.assertTrue(self.counter.snapshot()['pause_used'])

    def test_reset_invalidates_old_attempt_and_clears_measurements(self):
        self.circuit()
        self.status(0., state='ready', session='two')
        result = self.counter.snapshot()
        self.assertEqual(result['lap_count'], 0)
        self.assertEqual(result['elapsed_sim_s'], 0.)
        self.assertIsNone(result['attempt_id'])
        invalid = [e for e in self.counter.events if e['event'] == 'measurement_invalid']
        self.assertEqual(invalid[-1]['session_id'], 'one')
        self.assertEqual(invalid[-1]['reason'], 'session_reset')

    def test_collision_invalidates_completed_laps_and_freezes_measurement(self):
        self.circuit(); self.circuit(5.)
        self.status(10., state='collision', valid=False)
        result = self.counter.snapshot()
        self.assertFalse(result['measurement_valid'])
        self.assertFalse(result['target_reached'])
        self.assertEqual(result['lap_count'], 2)
        self.status(11., state='collision', valid=False)
        self.assertEqual(result['elapsed_sim_s'], self.counter.snapshot()['elapsed_sim_s'])

    def test_joining_active_attempt_requires_reset(self):
        self.counter = LapCounter(self.parameters)
        self.status(10.)
        self.assertEqual(self.counter.reason, 'attempt_start_not_observed')
        self.circuit(10.)
        self.assertEqual(self.counter.snapshot()['lap_count'], 0)

    def test_position_jump_invalidates_measurement(self):
        self.pose(.01, 20., 0.)
        self.assertEqual(self.counter.reason, 'position_jump')
        self.assertFalse(self.counter.valid)

    def test_nonfinite_pose_is_rejected(self):
        self.pose(.01, float('nan'), 0.)
        self.assertEqual(self.counter.reason, 'invalid_odometry')

    def test_first_pose_must_arrive_near_attempt_start(self):
        self.counter = LapCounter(self.parameters)
        self.status(0., state='ready'); self.status(0.)
        self.status(3.)
        self.counter.on_pose(3_000_000_000, 0., 0.)
        self.assertEqual(self.counter.reason, 'odometry_missing_at_start')

    def test_future_sample_waits_for_status_watermark(self):
        self.counter.on_pose(1_000_000_000, 2., 0.)
        self.assertEqual(self.counter.next_checkpoint, 0)
        self.pose(1., 2., 0.)
        self.assertEqual(self.counter.next_checkpoint, 1)

    def test_gap_and_reversed_odometry_are_rejected(self):
        self.pose(3., 1., 0.)
        self.assertEqual(self.counter.reason, 'odometry_gap')
        self.setUp(); self.pose(.5, .1, 0.)
        self.counter.on_pose(round(.4*1e9), .1, 0.)
        self.assertEqual(self.counter.reason, 'odometry_time_reversed')

    def test_duplicate_sample_cannot_duplicate_lap(self):
        self.circuit()
        for _ in range(10):
            self.counter.on_pose(5_000_000_000, 1., 0.)
        self.assertEqual(self.counter.snapshot()['lap_count'], 1)

    def test_start_away_from_finish_is_not_a_complete_measurement(self):
        self.counter = LapCounter(self.parameters)
        self.status(0., state='ready'); self.status(0.)
        self.counter.on_pose(0, 4., 0.)
        self.assertEqual(self.counter.reason, 'start_outside_finish_area')

    def test_source_time_reversal_without_reset_is_invalid(self):
        self.status(1.)
        self.status(.5)
        self.assertEqual(self.counter.reason, 'simulation_time_reversed')

    def test_manual_end_keeps_completed_lap_times(self):
        self.circuit()
        self.status(5., state='ended')
        self.assertEqual(self.counter.state, 'ended')
        self.assertTrue(self.counter.valid)
        self.assertEqual(self.counter.snapshot()['lap_times_s'], [4.5])

    def test_configuration_is_checked(self):
        for parameters in [replace(self.parameters, gate_yaw=float('nan')),
                           replace(self.parameters, checkpoints=()),
                           replace(self.parameters, target_laps=0)]:
            with self.assertRaises(ValueError): LapCounter(parameters)

    def test_panel_contains_laps_times_and_invalid_state(self):
        self.circuit()
        panel = format_panel(self.counter.snapshot(), 80, 24)
        self.assertIn('Vueltas completadas: 1 / 2', panel)
        self.assertIn('00:04.50', panel)
        self.status(5., state='collision', valid=False)
        self.assertIn('INTENTO INVALIDO', format_panel(self.counter.snapshot()))
        self.assertEqual(format_time(59.999), '01:00.00')
        self.assertEqual(format_time(None), '--:--.--')
