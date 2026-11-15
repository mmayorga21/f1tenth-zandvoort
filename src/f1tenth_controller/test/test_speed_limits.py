"""Speed must respect both curvature and available stopping distance."""
import math
import unittest

from f1tenth_controller.gap import FollowGap, GapParameters


class SpeedLimitTests(unittest.TestCase):
    def core(self):
        core=FollowGap(GapParameters(max_speed_mps=8.,max_acceleration_mps2=3.,
                                     lateral_acceleration_limit_mps2=3.,
                                     braking_deceleration_mps2=4.))
        core.previous_speed=8.
        return core

    def test_curvature_limit_anticipates_upcoming_turn(self):
        core=self.core()
        speed=core.speed_limit(.3,0.,6.,.01)
        expected=math.sqrt(3.*core.p.wheelbase_m/math.tan(.3))
        self.assertAlmostEqual(speed,expected)

    def test_curvature_limit_remains_while_steering_unwinds(self):
        core=self.core()
        self.assertAlmostEqual(core.speed_limit(0.,.3,6.,.01),
                               core.speed_limit(.3,0.,6.,.01))

    def test_braking_uses_clearance_after_margin(self):
        core=self.core()
        self.assertAlmostEqual(core.speed_limit(0.,0.,.95,.01),2.)
        self.assertEqual(core.speed_limit(0.,0.,.45,.01),0.)

    def test_straight_acceleration_is_bounded(self):
        core=self.core();core.previous_speed=0.
        self.assertAlmostEqual(core.speed_limit(0.,0.,20.,.01),.03)
        core.previous_speed=8.
        self.assertEqual(core.speed_limit(0.,0.,20.,.01),8.)

    def test_optional_limits_require_a_consistent_pair(self):
        for params in [GapParameters(lateral_acceleration_limit_mps2=3.),
                       GapParameters(braking_deceleration_mps2=-1.)]:
            with self.assertRaises(ValueError):FollowGap(params)

    def test_legacy_policy_keeps_original_formula(self):
        core=FollowGap();core.previous_speed=1.2
        expected=1.2-(1.2-.65)*(.2/.4189)
        self.assertAlmostEqual(core.speed_limit(.2,.1,6.,.01),expected)
