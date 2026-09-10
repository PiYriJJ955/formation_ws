#!/usr/bin/env python3
"""Offline controller integration, numerical gradients and closed-loop simulations.
No ROS Master, SSH connection or chassis is started.
"""
import math
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for package in ('five_ugv_formation_control', 'five_ugv_mpc_formation_control'):
    sys.path.insert(0, str(ROOT / 'src' / package / 'src'))
from five_ugv_mpc_formation_control.mpc import MPCSolver, SolverFailure, advance, formation_reference


class Stamp(float):
    def __new__(cls, seconds=0, nsecs=0):
        return float.__new__(cls, seconds + nsecs / 1e9)
    def __sub__(self, other):
        return Stamp(float(self) - float(other))
    def to_sec(self):
        return float(self)
    @staticmethod
    def from_sec(value):
        return Stamp(value)
    @staticmethod
    def now():
        return Stamp(100)


def xyz():
    return SimpleNamespace(x=0.0, y=0.0, z=0.0)


def pose():
    return SimpleNamespace(position=xyz(), orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))


def stamped_pose():
    return SimpleNamespace(header=SimpleNamespace(frame_id='', stamp=Stamp()), pose=pose())


def twist():
    return SimpleNamespace(linear=xyz(), angular=xyz())


class Scalar:
    def __init__(self, data=None):
        self.data = data


class Sink:
    def publish(self, message):
        self.last = message


def ros_modules():
    ros = Mock()
    ros.Time = Stamp
    ros.Duration = Stamp
    ros.get_param.side_effect = lambda name, default: default
    ros.Publisher.side_effect = lambda *a, **k: Sink()
    return {'rospy': ros,
            'geometry_msgs.msg': SimpleNamespace(PoseStamped=stamped_pose,
                PoseWithCovarianceStamped=lambda: SimpleNamespace(
                    header=SimpleNamespace(stamp=Stamp()), pose=SimpleNamespace(pose=pose())),
                Twist=twist, Vector3Stamped=lambda: SimpleNamespace(
                    header=SimpleNamespace(), vector=xyz())),
            'nav_msgs.msg': SimpleNamespace(Odometry=lambda: SimpleNamespace(
                header=SimpleNamespace(stamp=Stamp()), twist=SimpleNamespace(twist=twist()))),
            'std_msgs.msg': SimpleNamespace(Bool=Scalar, Float64=Scalar, String=Scalar)}


with patch.dict(sys.modules, ros_modules()):
    from five_ugv_formation_control import follower as shared
    from five_ugv_mpc_formation_control.controller import MPCFollower


class MPCChecks(unittest.TestCase):
    def test_analytic_gradient_matches_finite_difference(self):
        solver = MPCSolver(horizon=5)
        rng = np.random.RandomState(4)
        u = rng.uniform(-.1, .1, 10)
        refs = rng.uniform(-1, 1, (5, 3))
        args = ((1., -2., 3.1), refs, np.zeros((5, 2)), np.full(5, .15), (.08, -.1))
        _, gradient = solver.objective(u, *args)
        numerical = []
        for i in range(len(u)):
            delta = np.zeros_like(u)
            delta[i] = 1e-6
            numerical.append((solver.objective(u+delta, *args)[0] -
                              solver.objective(u-delta, *args)[0]) / 2e-6)
        np.testing.assert_allclose(gradient, numerical, atol=1e-6, rtol=1e-5)

    def test_reference_uses_current_frame_and_rotating_offset_tangent(self):
        refs, controls = formation_reference((2, 3, math.pi/2), (.1, 0, .2),
                                            (-.8, .5), [0., .5], (0, 0, 0))
        np.testing.assert_allclose(refs[0, :2], (1.5, 2.2), atol=1e-12)
        # v - omega*dy = 0, omega*dx = -.16; tangent points along map +X.
        self.assertAlmostEqual(refs[0, 2], 0)
        np.testing.assert_allclose(controls[0], (.16, .2))
        predicted = advance((2, 3, math.pi/2), (.1, 0, .2), .5)
        np.testing.assert_allclose(refs[1, :2], predicted[:2] + [
            -.8*math.cos(predicted[2])-.5*math.sin(predicted[2]),
            -.8*math.sin(predicted[2])+.5*math.cos(predicted[2])])

    def test_limits_yaw_wrap_failure_and_deadline(self):
        solver = MPCSolver(solve_timeout=1)
        args = ((-.7, 0, math.pi-.01), (0, 0, -math.pi+.01), (.1, 0, .04),
                (-.8, .2), (.14, .2), (.15, .3), .05, .15, .8, .15, .8)
        result = solver.solve(*args)
        self.assertLessEqual(abs(result[0]-.14), .15*.05+1e-6)
        self.assertLessEqual(abs(result[1]-.2), .8*.05+1e-6)
        self.assertTrue(np.all(solver.plan[:, 0] <= .15+1e-6))
        lower_limit = list(args)
        lower_limit[7] = .03
        result = solver.solve(*lower_limit)
        self.assertLessEqual(result[0], .03+1e-6)
        solver.timeout = 1e-12
        with self.assertRaisesRegex(SolverFailure, 'MPC_TIMEOUT'):
            solver.solve(*args)
        self.assertIsNone(solver.plan)
        solver.timeout = 1
        with patch('five_ugv_mpc_formation_control.mpc.minimize',
                   return_value=SimpleNamespace(success=False, x=np.zeros(24), fun=0)):
            with self.assertRaisesRegex(SolverFailure, 'MPC_SOLVER_FAILED'):
                solver.solve(*args)
        bad = list(args)
        bad[0] = (float('nan'), 0, 0)
        with self.assertRaisesRegex(SolverFailure, 'MPC_INVALID_INPUT'):
            solver.solve(*bad)

    def controller(self, auto_align=False):
        params = {'~auto_align_yaw': auto_align, '~offset_x': 0., '~offset_y': 0.,
                  '~mpc_solve_timeout': 1.0}
        shared.rospy.get_param.side_effect = lambda name, default: params.get(name, default)
        c = MPCFollower()
        c.enabled = True
        return c

    def feed(self, c, t, leader=(0, 0, 0), follower=(-.4, 0, 0),
             velocity=(0, 0, 0), own=(0, 0, 0), age=0, frame='uwb_map'):
        for robot, p, v in ((c.leader, leader, velocity), (c.follower, follower, own)):
            stamp = Stamp(t-age)
            robot.pose, robot.pose_stamp, robot.frame_id = tuple(p), stamp, frame
            robot.valid, robot.valid_stamp = True, stamp
            robot.headings.append((stamp, p[2]))
            if robot.velocity_stamp is None or stamp > robot.velocity_stamp:
                robot.update_velocity(v, stamp, c.velocity_filter_tau, c.data_timeout)
        with patch.object(Stamp, 'now', return_value=Stamp(t)):
            c.control_step()
        return c.last_cmd

    def test_ros_contract_alignment_frames_debug_limits_and_stops(self):
        for frame in ('uwb_map', 'linktrack_map'):
            c = self.controller(auto_align=True)
            self.feed(c, 100, frame=frame)
            self.assertEqual(c.last_cmd, (0, 0))
            self.assertEqual(c.state_pub.last.data, 'WAIT_ALIGNMENT')
            for k in range(1, 14):
                self.feed(c, 100+k*.05, frame=frame)
            self.assertGreater(c.last_cmd[0], 0)
            self.assertEqual(c.target_pub.last.header.frame_id, frame)
            c.limit_cb(Scalar(data=.03))
            self.feed(c, 100.7, frame=frame)
            self.assertLessEqual(c.last_cmd[0], .03)
            self.assertEqual(c.limit_pub.last.data, .03)
            c.valid_cb(Scalar(data=False), c.leader)
            self.assertEqual(c.last_cmd, (0, 0))
            self.assertIsNone(c.solver.plan)
            self.feed(c, 102, age=1, frame=frame)
            self.assertEqual(c.state_pub.last.data, 'STALE_OR_MISSING_INPUT')
            c.enable_cb(Scalar(data=False))
            self.assertEqual(c.last_cmd, (0, 0))
            self.assertEqual(c.state_pub.last.data, 'DISABLED')
        c = self.controller()
        self.feed(c, 110)
        c.follower.frame_id = 'another_map'
        with patch.object(Stamp, 'now', return_value=Stamp(110.01)):
            c.control_step()
        self.assertEqual(c.last_cmd, (0, 0))
        self.assertEqual(c.state_pub.last.data, 'FRAME_MISMATCH')
        self.feed(c, 110.05, frame='linktrack_map')
        self.assertEqual(c.state_pub.last.data, 'FRAME_CHANGED_RESTART')
        c = self.controller()
        with patch.object(c.solver, 'solve', side_effect=SolverFailure('MPC_TIMEOUT')):
            self.feed(c, 111)
        self.assertEqual(c.last_cmd, (0, 0))
        self.assertEqual(c.state_pub.last.data, 'MPC_TIMEOUT')

    def test_existing_displacement_regressions(self):
        shared.self_test()

    def test_closed_loop(self):
        report = []
        # Lateral/behind capture, both turn directions, S turn, spinning leader,
        # non-zero global heading, 100 ms input delay, jitter and chassis lag.
        for profile, oy in [('static', 0), ('behind', 0), ('left', -.5),
                            ('right', .5), ('s_bend', .5), ('spin', -.5)]:
            c = self.controller()
            c.offset_x, c.offset_y = (-.8, oy) if profile not in ('static', 'behind') else (0., 0.)
            leader = np.array([2., 3., 2.9])
            if profile in ('static', 'behind'):
                follower = leader + np.array([.4, .3, 0 if profile == 'static' else math.pi])
            else:
                refs, _ = formation_reference(leader, (0, 0, 0), (-.8, oy), [0], leader)
                follower = np.array([refs[0, 0]+.1, refs[0, 1]-.1, leader[2]])
            own, last = (0, 0, 0), (0, 0)
            history, errors, seconds, failures = [], [], [], []
            for k in range(900):
                omega = (.06 if profile == 'left' else -.06 if profile == 'right' else
                         .07*math.sin(k*.05/6) if profile == 's_bend' else .06 if profile == 'spin' else 0)
                velocity = (0 if profile in ('static', 'behind', 'spin') else .06, 0, omega)
                history.append((leader.copy(), follower.copy(), velocity, own))
                dl, df, dv, do = history[max(0, len(history)-3)]
                observed = df + np.array([.008*math.sin(.7*k), .008*math.cos(.5*k), 0])
                command = self.feed(c, 200+k*.05, dl, observed, dv, do, age=min(k, 2)*.05)
                if c.state_pub.last.data.startswith('MPC_'):
                    failures.append(c.state_pub.last.data)
                if not c.holding:
                    self.assertLessEqual(abs(command[0]-last[0]), .15*.05+1e-6)
                if not c.holding:
                    self.assertLessEqual(abs(command[1]-last[1]), .8*.05+1e-6, (profile, k, c.state_pub.last.data))
                self.assertTrue(0 <= command[0] <= .15+1e-9)
                last = command
                own = (own[0]+.2*(command[0]-own[0]), 0, own[2]+.2*(command[1]-own[2]))
                leader = advance(leader, velocity, .05)
                follower = advance(follower, own, .05)
                if k >= 400:
                    refs, _ = formation_reference(leader, velocity, (c.offset_x, oy), [0], follower)
                    errors.append(np.linalg.norm(refs[0, :2]-follower[:2]))
                    seconds.append(c.solver.last_seconds)
            self.assertFalse(failures, (profile, failures[:5]))
            rms = float(np.sqrt(np.mean(np.square(errors))))
            p95 = float(np.percentile(errors, 95))
            self.assertLess(rms, .14, (profile, rms))
            self.assertLess(p95, .20, (profile, p95))
            if profile in ('static', 'behind'):
                self.assertTrue(c.holding, (profile, follower))
                self.assertLess(abs(shared.wrap_angle(follower[2]-leader[2])), .11)
            report.append('%s RMS=%.4f P95=%.4f solve_P95=%.4fs' %
                          (profile, rms, p95, np.percentile(seconds, 95)))
        print('\n'.join(report))


if __name__ == '__main__':
    unittest.main()
