"""MPC backend of the existing fleet control contract."""
from __future__ import division
import rospy
from std_msgs.msg import Float64, String
from five_ugv_formation_control.follower import DisplacementFollower, ControlUnavailable
from .mpc import MPCSolver, SolverFailure


class MPCFollower(DisplacementFollower):
    algorithm = 'mpc'

    def __init__(self):
        names = dict(horizon=12, dt=0.15, position_weight=20.0, heading_weight=0.3,
                     terminal_weight=4.0, linear_weight=0.5, angular_weight=0.05,
                     delta_linear_weight=1.0, delta_angular_weight=0.1,
                     max_iterations=40, tolerance=1e-5, solve_timeout=0.04)
        self.solver = MPCSolver(**{key: rospy.get_param('~mpc_' + key, value)
                                   for key, value in names.items()})
        self.control_frame = None
        self.solver_time_pub = rospy.Publisher('~mpc_solve_seconds', Float64, queue_size=1)
        self.solver_status_pub = rospy.Publisher('~mpc_solver_status', String, queue_size=1, latch=True)
        self.solver_cost_pub = rospy.Publisher('~mpc_cost', Float64, queue_size=1)
        self.solver_status_pub.publish(String(data='IDLE'))
        DisplacementFollower.__init__(self)

    def data_ready(self, now):
        frames = (self.leader.frame_id, self.follower.frame_id)
        if not all(frames) or frames[0] != frames[1]:
            return False, 'FRAME_MISMATCH'
        if self.control_frame is None:
            self.control_frame = frames[0]
        if frames[0] != self.control_frame:
            return False, 'FRAME_CHANGED_RESTART'
        for robot in (self.leader, self.follower):
            if robot.pose is not None and robot.pose[2] is None:
                return False, 'INVALID_ORIENTATION'
        return DisplacementFollower.data_ready(self, now)

    def refine_command(self, leader_pose, self_pose, leader_velocity, dt, seed):
        if self.holding or self.max_linear == 0:
            self.solver.reset()
            self.solver_status_pub.publish(String(data='HOLD' if self.holding else 'ZERO_LINEAR_LIMIT'))
            return seed
        if dt <= 0:
            raise ControlUnavailable('MPC_INVALID_PERIOD')
        try:
            command = self.solver.solve(
                self_pose, leader_pose, leader_velocity, (self.offset_x, self.offset_y),
                self.last_cmd, seed, dt, self.max_linear, self.max_angular,
                self.max_acceleration, self.max_angular_acceleration)
        except SolverFailure as error:
            self.solver_status_pub.publish(String(data=str(error)))
            rospy.logwarn_throttle(1.0, '%s MPC stopped: %s', self.robot_name, error)
            raise ControlUnavailable(str(error).split(':', 1)[0])
        except Exception as error:
            self.solver_status_pub.publish(String(data='MPC_SOLVER_EXCEPTION'))
            rospy.logwarn_throttle(1.0, '%s MPC exception: %s', self.robot_name, error)
            raise ControlUnavailable('MPC_SOLVER_EXCEPTION')
        finally:
            self.solver_time_pub.publish(Float64(data=self.solver.last_seconds))
        # The optimization consumed time; never emit a command from now-stale inputs.
        ready, reason = self.data_ready(rospy.Time.now())
        if not ready:
            raise ControlUnavailable(reason)
        self.solver_status_pub.publish(String(data='SOLVED'))
        self.solver_cost_pub.publish(Float64(data=self.solver.last_cost))
        return command

    def stop(self):
        self.solver.reset()
        DisplacementFollower.stop(self)
