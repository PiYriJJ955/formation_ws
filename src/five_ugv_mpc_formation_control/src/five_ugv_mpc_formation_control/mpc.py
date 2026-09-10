"""ROS-independent single-shooting NMPC with analytic derivatives (SciPy SLSQP)."""
from __future__ import division
import math
from timeit import default_timer as clock
import numpy as np
from scipy.optimize import minimize


class SolverFailure(RuntimeError):
    pass


def advance(pose, velocity, dt):
    """Exact constant body-twist integration, map X/Y, CCW radians."""
    x, y, yaw = pose
    vx, vy, omega = velocity
    angle = omega * dt
    if abs(angle) < 1e-8:
        dx, dy = vx * dt, vy * dt
    else:
        s, c = math.sin(angle) / omega, (1.0 - math.cos(angle)) / omega
        dx, dy = s * vx - c * vy, c * vx + s * vy
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([x + c * dx - s * dy, y + s * dx + c * dy, yaw + angle])


def formation_reference(leader, velocity, offset, times, follower):
    """Rotate offsets with predicted leader heading; track the offset-point tangent."""
    ox, oy = offset
    vx, vy, omega = velocity
    qx, qy = vx - omega * oy, vy + omega * ox
    speed = math.hypot(qx, qy)
    poses, controls = [], []
    for t in times:
        x, y, yaw = advance(leader, velocity, t)
        c, s = math.cos(yaw), math.sin(yaw)
        tx, ty = x + c * ox - s * oy, y + s * ox + c * oy
        # Final formation heading is handled by shared HOLD, after position capture.
        heading = (yaw + math.atan2(qy, qx) if speed > 1e-6 else
                   math.atan2(ty - follower[1], tx - follower[0]))
        poses.append((tx, ty, heading))
        controls.append((speed, omega if speed > 1e-6 else 0.0))
    return np.asarray(poses), np.asarray(controls)


class MPCSolver(object):
    def __init__(self, horizon=12, dt=0.15, position_weight=20.0,
                 heading_weight=0.3, terminal_weight=4.0, linear_weight=0.5,
                 angular_weight=0.05, delta_linear_weight=1.0,
                 delta_angular_weight=0.1, max_iterations=40,
                 tolerance=1e-5, solve_timeout=0.04):
        if isinstance(horizon, bool) or int(horizon) != horizon or not 3 <= horizon <= 40:
            raise ValueError('mpc_horizon must be an integer in [3, 40]')
        if (isinstance(max_iterations, bool) or int(max_iterations) != max_iterations or
                not 1 <= max_iterations <= 200):
            raise ValueError('mpc_max_iterations must be an integer in [1, 200]')
        positive = (dt, position_weight, heading_weight, terminal_weight,
                    linear_weight, angular_weight, delta_linear_weight,
                    delta_angular_weight, tolerance, solve_timeout)
        if not all(np.isfinite(v) and v > 0 for v in positive):
            raise ValueError('MPC time steps, budget and weights must be finite and positive')
        self.n, self.dt = int(horizon), float(dt)
        self.qp, self.qh, self.terminal = position_weight, heading_weight, terminal_weight
        self.r = np.array([linear_weight, angular_weight])
        self.rd = np.array([delta_linear_weight, delta_angular_weight])
        self.max_iterations, self.tolerance = int(max_iterations), tolerance
        self.timeout = solve_timeout
        self.plan = None
        self.last_seconds, self.last_cost, self.last_iterations = 0.0, 0.0, 0
        self.difference = np.eye(2 * self.n)
        self.difference[2:, :-2] -= np.eye(2 * self.n - 2)
        self.constraint_jac = np.vstack((self.difference, -self.difference))

    def reset(self):
        self.plan = None

    def objective(self, flat, pose, references, feedforward, steps, previous):
        """Midpoint unicycle model and reverse accumulation of exact cost gradients."""
        controls = np.asarray(flat).reshape(self.n, 2)
        states, matrices = [], []
        x = np.asarray(pose, dtype=float).copy()
        for u, dt in zip(controls, steps):
            v, w = u
            phi = x[2] + 0.5 * dt * w
            c, s = math.cos(phi), math.sin(phi)
            a = np.eye(3)
            a[0, 2], a[1, 2] = -dt * v * s, dt * v * c
            b = np.array([[dt * c, -0.5 * dt * dt * v * s],
                          [dt * s, 0.5 * dt * dt * v * c], [0.0, dt]])
            x = x + np.array([dt * v * c, dt * v * s, dt * w])
            states.append(x)
            matrices.append((a, b))
        error = np.asarray(states) - references
        weights = np.ones(self.n)
        weights[-1] = self.terminal
        du = controls - np.vstack((previous, controls[:-1]))
        eu = controls - feedforward
        # Periodic yaw cost has no +/-pi discontinuity.
        value = np.sum(weights * (self.qp * np.sum(error[:, :2] ** 2, axis=1) +
                                  2 * self.qh * (1 - np.cos(error[:, 2]))))
        value += np.sum(self.r * eu ** 2) + np.sum(self.rd * du ** 2)
        state_gradient = np.column_stack((2 * self.qp * error[:, :2],
                                         2 * self.qh * np.sin(error[:, 2])))
        state_gradient *= weights[:, None]
        gradient = 2 * self.r * eu + 2 * self.rd * du
        gradient[:-1] -= 2 * self.rd * du[1:]
        adjoint = np.zeros(3)
        for k in range(self.n - 1, -1, -1):
            adjoint += state_gradient[k]
            a, b = matrices[k]
            gradient[k] += b.T.dot(adjoint)
            adjoint = a.T.dot(adjoint)
        return float(value), gradient.ravel()

    def solve(self, pose, leader, velocity, offset, previous, seed, control_dt,
              max_linear, max_angular, max_acceleration, max_angular_acceleration):
        started = clock()
        self.last_seconds = 0.0
        values = list(pose) + list(leader) + list(velocity) + list(offset) + list(previous) + list(seed)
        values += [control_dt, max_linear, max_angular, max_acceleration, max_angular_acceleration]
        if (not np.all(np.isfinite(values)) or control_dt <= 0 or max_linear < 0 or
                min(max_angular, max_acceleration, max_angular_acceleration) <= 0):
            self.reset()
            raise SolverFailure('MPC_INVALID_INPUT')
        steps = np.full(self.n, self.dt)
        steps[0] = control_dt
        references, feedforward = formation_reference(
            leader, velocity, offset, np.cumsum(steps), pose)
        lo, hi = np.array([0.0, -max_angular]), np.array([max_linear, max_angular])
        # A new lower hard limit takes precedence over acceleration constraints.
        previous = np.clip(previous, lo, hi)
        change = steps[:, None] * np.array([max_acceleration, max_angular_acceleration])
        center = np.zeros((self.n, 2))
        center[0] = previous
        lower, upper = (center - change).ravel(), (center + change).ravel()

        def feasible_plan(plan):
            plan = np.asarray(plan).copy()
            last = previous
            for k in range(self.n):
                plan[k] = np.clip(plan[k], np.maximum(lo, last - change[k]),
                                  np.minimum(hi, last + change[k]))
                last = plan[k]
            return plan

        initial = feasible_plan(np.tile(seed, (self.n, 1)))
        objective_args = (pose, references, feedforward, steps, previous)
        if self.plan is not None:
            # Reuse at the control period, rather than shifting a whole prediction step.
            times = np.arange(self.n) * self.dt
            warm = np.column_stack([np.interp(times + control_dt, times, self.plan[:, i])
                                    for i in range(2)])
            warm = feasible_plan(warm)
            if self.objective(warm.ravel(), *objective_args)[0] < self.objective(initial.ravel(), *objective_args)[0]:
                initial = warm

        def check_time():
            if clock() - started > self.timeout:
                raise SolverFailure('MPC_TIMEOUT')

        def objective(flat):
            check_time()
            return self.objective(flat, *objective_args)

        def constraints(flat):
            delta = self.difference.dot(flat)
            return np.concatenate((delta - lower, upper - delta))

        try:
            result = minimize(objective, initial.ravel(), jac=True, method='SLSQP',
                              bounds=[(lo[i % 2], hi[i % 2]) for i in range(2 * self.n)],
                              constraints={'type': 'ineq', 'fun': constraints,
                                           'jac': lambda _: self.constraint_jac},
                              callback=lambda _: check_time(),
                              options={'maxiter': self.max_iterations, 'ftol': self.tolerance,
                                       'disp': False})
            check_time()
            if (not result.success or not np.all(np.isfinite(result.x)) or
                    not np.isfinite(result.fun) or np.min(constraints(result.x)) < -1e-6):
                raise SolverFailure('MPC_SOLVER_FAILED')
            plan = result.x.reshape(self.n, 2)
            if np.any(plan < lo - 1e-6) or np.any(plan > hi + 1e-6):
                raise SolverFailure('MPC_INFEASIBLE')
            self.plan = np.clip(plan, lo, hi)
            self.last_cost, self.last_iterations = float(result.fun), int(result.nit)
            return tuple(self.plan[0])
        except SolverFailure:
            self.reset()
            raise
        except Exception as error:
            self.reset()
            raise SolverFailure('MPC_SOLVER_EXCEPTION: ' + str(error))
        finally:
            self.last_seconds = clock() - started
