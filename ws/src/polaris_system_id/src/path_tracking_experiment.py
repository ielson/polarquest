#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unified ROS path-tracking experiment node.

Controllers:
  - greybox_mpc: proposed steering-only nonlinear MPC with identified model
  - kinematic_mpc: nominal kinematic-bicycle MPC with the same cost/constraints
  - stanley: classical Stanley path-tracking controller
  - pure_pursuit: optional classical geometric baseline

All controllers consume the same odometry, path, speed profile, steering limits,
control period, initial pose, and experiment logger. This is intended for a fair
closed-loop comparison for the PolarQuest paper revision.
"""

import csv
import json
import math
import os
import time
from datetime import datetime
from typing import Dict, Optional, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rospkg
import rospy
from ackermann_msgs.msg import AckermannDrive
from gekko import GEKKO
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def wrap_angle(angle: float) -> float:
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


class YawUnwrapper:
    def __init__(self) -> None:
        self.previous: Optional[float] = None
        self.offset = 0.0

    def update(self, yaw: float) -> float:
        if self.previous is None:
            self.previous = yaw
            return yaw
        delta = yaw - self.previous
        if delta > math.pi:
            self.offset -= 2.0 * math.pi
        elif delta < -math.pi:
            self.offset += 2.0 * math.pi
        self.previous = yaw
        return yaw + self.offset


class PathGeometry:
    """Reference path with local, monotonic nearest-point tracking.

    A local search is important for the infinity-shaped trajectory: a global
    nearest-point query may jump to the wrong branch at the self-intersection.
    """

    def __init__(self, x: np.ndarray, y: np.ndarray) -> None:
        if len(x) < 3:
            raise ValueError("The reference path must contain at least 3 points")
        self.x = np.asarray(x, dtype=float)
        self.y = np.asarray(y, dtype=float)
        dx = np.gradient(self.x)
        dy = np.gradient(self.y)
        self.psi = np.unwrap(np.arctan2(dy, dx))

        ds = np.hypot(np.diff(self.x), np.diff(self.y))
        self.s = np.concatenate(([0.0], np.cumsum(ds)))
        denom = np.maximum((dx * dx + dy * dy) ** 1.5, 1e-9)
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        self.kappa = (dx * ddy - dy * ddx) / denom

    @classmethod
    def from_ros_path(cls, msg: Path) -> "PathGeometry":
        x = np.array([pose.pose.position.x for pose in msg.poses], dtype=float)
        y = np.array([pose.pose.position.y for pose in msg.poses], dtype=float)
        return cls(x, y)

    @property
    def size(self) -> int:
        return len(self.x)

    def nearest_index(
        self,
        x: float,
        y: float,
        last_idx: Optional[int],
        search_back: int,
        search_forward: int,
    ) -> int:
        if last_idx is None:
            d2 = (self.x - x) ** 2 + (self.y - y) ** 2
            return int(np.argmin(d2))

        lo = max(0, int(last_idx) - max(0, search_back))
        hi = min(self.size, int(last_idx) + max(2, search_forward) + 1)
        d2 = (self.x[lo:hi] - x) ** 2 + (self.y[lo:hi] - y) ** 2
        candidate = lo + int(np.argmin(d2))
        # Keep progress monotonic to avoid switching branches at the crossing.
        return max(int(last_idx), candidate)

    def tracking_errors(
        self,
        x: float,
        y: float,
        yaw: float,
        last_idx: Optional[int],
        search_back: int,
        search_forward: int,
        axle_offset: float = 0.0,
    ) -> Tuple[int, float, float, float]:
        qx = x + axle_offset * math.cos(yaw)
        qy = y + axle_offset * math.sin(yaw)
        idx = self.nearest_index(qx, qy, last_idx, search_back, search_forward)
        seg_idx = min(idx, self.size - 2)

        p0 = np.array([self.x[seg_idx], self.y[seg_idx]], dtype=float)
        p1 = np.array([self.x[seg_idx + 1], self.y[seg_idx + 1]], dtype=float)
        seg = p1 - p0
        l2 = float(np.dot(seg, seg)) + 1e-12
        q = np.array([qx, qy], dtype=float)
        alpha = float(np.clip(np.dot(q - p0, seg) / l2, 0.0, 1.0))
        segment_length = float(
            self.s[seg_idx + 1] - self.s[seg_idx]
        )

        s_projected = float(
            self.s[seg_idx]
            + alpha * segment_length
        )
        projection = p0 + alpha * seg
        error_vec = q - projection
        distance = float(np.hypot(error_vec[0], error_vec[1]))
        cross_z = float(seg[0] * error_vec[1] - seg[1] * error_vec[0])
        cte = distance if cross_z >= 0.0 else -distance
        heading_error = wrap_angle(float(self.psi[seg_idx]) - yaw)
        return idx, s_projected, cte, heading_error

    def segment(self, start: int, count: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        indices = np.clip(np.arange(start, start + count), 0, self.size - 1)
        return self.x[indices], self.y[indices], self.psi[indices]

    def target_index_by_distance(self, start: int, distance: float) -> int:
        target_s = self.s[min(start, self.size - 1)] + max(distance, 0.0)
        return int(min(np.searchsorted(self.s, target_s), self.size - 1))
    
    def segment_by_distance(
        self,
        start: int,
        speed: float,
        Ts: float,
        count: int,
        preview_distance: float = 0.0,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:

        start = min(max(int(start), 0), self.size - 1)
        s0 = float(self.s[start]) + max(preview_distance, 0.0)

        offsets = (
            np.arange(count, dtype=float)
            * max(speed, 0.0)
            * Ts
        )

        target_s = np.clip(
            s0 + offsets,
            self.s[0],
            self.s[-1],
        )

        x_ref = np.interp(target_s, self.s, self.x)
        y_ref = np.interp(target_s, self.s, self.y)
        psi_ref = np.interp(target_s, self.s, self.psi)

        return x_ref, y_ref, psi_ref

    def horizon_from_s(
        self,
        start_s: float,
        speed_profile: np.ndarray,
        Ts: float,
        count: int,
    ) -> Tuple[
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
    ]:
        """Build a time-consistent reference horizon.

        The reference advances by v * Ts at each prediction step.
        """

        target_s = np.empty(count, dtype=float)
        speed_ref = np.empty(count, dtype=float)

        target_s[0] = float(
            np.clip(
                start_s,
                self.s[0],
                self.s[-1],
            )
        )

        for step in range(count):
            speed_ref[step] = float(
                np.interp(
                    target_s[step],
                    self.s,
                    speed_profile,
                )
            )

            if step + 1 < count:
                target_s[step + 1] = min(
                    target_s[step]
                    + max(speed_ref[step], 0.0) * Ts,
                    self.s[-1],
                )

        x_ref = np.interp(
            target_s,
            self.s,
            self.x,
        )

        y_ref = np.interp(
            target_s,
            self.s,
            self.y,
        )

        psi_ref = np.interp(
            target_s,
            self.s,
            self.psi,
        )

        return (
            x_ref,
            y_ref,
            psi_ref,
            speed_ref,
            target_s,
        )

# ---------------------------------------------------------------------------
# MPC builders
# ---------------------------------------------------------------------------

class GreyBoxMPC:
    def __init__(self, config: Dict[str, float]) -> None:
        self.Ts = config["Ts"]
        self.Np = int(config["Np"])
        self.steer_min = -config["steer_limit_rad"]
        self.steer_max = config["steer_limit_rad"]
        self.dmax = config["steer_dmax_rad_step"]
        self.pars = config["greybox"]

        horizon = np.linspace(0.0, self.Np * self.Ts, self.Np + 1)
        zeros = np.zeros(self.Np + 1)
        speed = np.full(self.Np + 1, config["v_fixed"])

        self.m = GEKKO(remote=False)
        self.m.time = horizon
        self.m.options.IMODE = 6
        self.m.options.NODES = int(config["gekko_nodes"])
        self.m.options.SOLVER = int(config["gekko_solver"])
        self.m.options.SCALING = 1
        self.m.options.MAX_ITER = int(config["max_iter"])
        self.m.solver_options = [
            "print_level 0",
            f"max_iter {int(config['max_iter'])}",
            f"tol {config['solver_tol']}",
            f"acceptable_tol {config['acceptable_tol']}",
        ]
        linear_solver = str(config.get("linear_solver", "")).strip()
        if linear_solver:
            self.m.solver_options.append(f"linear_solver {linear_solver}")

        p = self.pars
        tau_acc = self.m.Param(value=p["tau_acc"])
        tau_str = self.m.Param(value=p["tau_str"])
        tau_v = self.m.Param(value=p["tau_v"])
        L_eff = self.m.Param(value=p["L_eff"])
        k_del = self.m.Param(value=p["k_del"])
        k_b1 = self.m.Param(value=p["k_b1"])
        k_b3 = self.m.Param(value=p["k_b3"])
        psi_bias = self.m.Param(value=p["psi_bias"])

        self.x_sp = self.m.Param(value=zeros.copy())
        self.y_sp = self.m.Param(value=zeros.copy())
        self.psi_sp = self.m.Param(value=zeros.copy())
        self.speed_sp = self.m.Param(value=speed.copy())

        self.v = self.m.Var(value=config["v_fixed"], lb=-10.0, ub=10.0)
        self.psi = self.m.Var(value=0.0)
        self.xg = self.m.Var(value=0.0)
        self.yg = self.m.Var(value=0.0)
        self.uva = self.m.Var(value=config["v_fixed"], lb=-10.0, ub=10.0)
        self.da = self.m.Var(value=0.0, lb=self.steer_min, ub=self.steer_max)

        self.delta = self.m.MV(value=0.0, lb=self.steer_min, ub=self.steer_max)
        self.delta.STATUS = 1
        self.delta.DMAX = self.dmax
        self.delta.DCOST = config["mv_dcost"]

        self.m.Equation(self.uva.dt() == (self.speed_sp - self.uva) / tau_acc)
        self.m.Equation(self.da.dt() == (self.delta - self.da) / tau_str)
        self.m.Equation(self.v.dt() == (self.uva - self.v) / tau_v)
        self.m.Equation(self.psi.dt() == (self.v / L_eff) * self.m.tan(k_del * self.da))
        beta = self.m.Intermediate(k_b1 * self.da + k_b3 * self.da ** 3)
        self.m.Equation(self.xg.dt() == self.v * self.m.cos(self.psi + psi_bias + beta))
        self.m.Equation(self.yg.dt() == self.v * self.m.sin(self.psi + psi_bias + beta))

        ex = self.m.Intermediate(
            self.xg - self.x_sp
        )

        ey = self.m.Intermediate(
            self.yg - self.y_sp
        )

        contour_error = self.m.Intermediate(
            -self.m.sin(self.psi_sp) * ex
            + self.m.cos(self.psi_sp) * ey
        )

        lag_error = self.m.Intermediate(
            self.m.cos(self.psi_sp) * ex
            + self.m.sin(self.psi_sp) * ey
        )

        course_angle = self.m.Intermediate(
            self.psi
            + psi_bias
            + beta
        )

        heading_error = self.m.Intermediate(
            course_angle
            - self.psi_sp
        )
        self.m.Minimize(
            config["w_contour"]
            * contour_error**2
        )

        self.m.Minimize(
            config["w_lag"]
            * lag_error**2
        )

        self.m.Minimize(
            config["w_yaw"]
            * (
                1.0
                - self.m.cos(heading_error)
            )
        )

        self.m.Minimize(
            config["w_steer"]
            * self.delta**2
        )

        self.uva_est = config["v_fixed"]
        self.da_est = 0.0
        self.last_delta = 0.0


        self.m.options.TIME_SHIFT = 0
        self.m.options.WEB = 0

    def solve(
        self,
        x: float,
        y: float,
        yaw: float,
        speed_measured: float,
        _yaw_rate_measured: float,
        x_ref: np.ndarray,
        y_ref: np.ndarray,
        psi_ref: np.ndarray,
        speed_ref: np.ndarray,
    ) -> Tuple[float, bool]:
        psi_shift = round((yaw - float(psi_ref[0])) / (2.0 * math.pi)) * 2.0 * math.pi
        psi_aligned = psi_ref + psi_shift
        self.x_sp.value = x_ref.tolist()
        self.y_sp.value = y_ref.tolist()
        self.psi_sp.value = psi_aligned.tolist()
        self.speed_sp.value = speed_ref.tolist()

        self.v.value = float(speed_measured)
        self.psi.value = float(yaw)
        self.xg.value = float(x)
        self.yg.value = float(y)
        self.uva.value = float(self.uva_est)
        self.da.value = float(self.da_est)

        preview_idx = min(5, self.Np)
        guess = np.clip(wrap_angle(float(psi_aligned[preview_idx]) - yaw), self.steer_min, self.steer_max)
        self.delta.value = [float(guess)] * (self.Np + 1)

        try:
            self.m.solve(disp=False)
            success = int(self.m.options.APPSTATUS) == 1

            if success:
                delta = float(self.delta.NEWVAL)
            else:
                delta = self.last_delta
        except Exception as exc:  # GEKKO may raise several solver exception types
            rospy.logwarn_throttle(2.0, "Grey-box MPC solve failed: %s", exc)
            success = False
            delta = self.last_delta

        self.last_delta = float(np.clip(delta, self.steer_min, self.steer_max))
        return self.last_delta, success

    def update_actuator_estimates(self, speed_cmd: float, steering_cmd: float) -> None:
        p = self.pars
        self.uva_est += self.Ts * (speed_cmd - self.uva_est) / max(p["tau_acc"], 1e-6)
        self.da_est += self.Ts * (steering_cmd - self.da_est) / max(p["tau_str"], 1e-6)


class CurvatureGreyBoxMPC:
    """Grey-box MPC using an identified effective-curvature state.

    Identified lateral model:

        kappa_dot =
            (
                a1 * delta
                + a3 * delta**3
                - kappa
            ) / tau_kappa

        psi_dot = v * kappa

        x_dot = v * cos(psi)
        y_dot = v * sin(psi)

    The curvature dynamics represent the combined steering-actuator and
    vehicle response identified from steering command to measured yaw rate.

    The longitudinal model is intentionally kept equal to the successful
    legacy approximation for the first comparison. This isolates the effect
    of replacing the legacy da / tan(k_del * da) model with the directly
    identified curvature model.
    """

    def __init__(
        self,
        config: Dict[str, float],
    ) -> None:
        self.Ts = float(config["Ts"])
        self.Np = int(config["Np"])

        self.steer_min = -float(
            config["steer_limit_rad"]
        )
        self.steer_max = float(
            config["steer_limit_rad"]
        )
        self.dmax = float(
            config["steer_dmax_rad_step"]
        )

        self.pars = config[
            "curvature_greybox"
        ]

        self.kappa_limit = float(
            self.pars["kappa_limit"]
        )
        self.kappa_min_speed = float(
            self.pars["kappa_min_speed"]
        )

        horizon = np.linspace(
            0.0,
            self.Np * self.Ts,
            self.Np + 1,
        )

        zeros = np.zeros(
            self.Np + 1,
            dtype=float,
        )

        speed = np.full(
            self.Np + 1,
            float(config["v_fixed"]),
            dtype=float,
        )

        self.m = GEKKO(remote=False)
        self.m.time = horizon

        self.m.options.IMODE = 6
        self.m.options.NODES = int(
            config["gekko_nodes"]
        )
        self.m.options.SOLVER = int(
            config["gekko_solver"]
        )
        self.m.options.SCALING = 1
        self.m.options.MAX_ITER = int(
            config["max_iter"]
        )

        self.m.solver_options = [
            "print_level 0",
            (
                "max_iter "
                f"{int(config['max_iter'])}"
            ),
            f"tol {config['solver_tol']}",
            (
                "acceptable_tol "
                f"{config['acceptable_tol']}"
            ),
        ]

        linear_solver = str(
            config.get(
                "linear_solver",
                "",
            )
        ).strip()

        if linear_solver:
            self.m.solver_options.append(
                f"linear_solver {linear_solver}"
            )

        # ---------------------------------------------------------------
        # Identified parameters
        # ---------------------------------------------------------------

        tau_acc = self.m.Param(
            value=float(
                self.pars["tau_acc"]
            )
        )

        tau_v = self.m.Param(
            value=float(
                self.pars["tau_v"]
            )
        )

        tau_kappa = self.m.Param(
            value=float(
                self.pars["tau_kappa"]
            )
        )

        a1 = self.m.Param(
            value=float(
                self.pars["a1"]
            )
        )

        a3 = self.m.Param(
            value=float(
                self.pars["a3"]
            )
        )

        # ---------------------------------------------------------------
        # Time-varying references
        # ---------------------------------------------------------------

        self.x_sp = self.m.Param(
            value=zeros.copy()
        )

        self.y_sp = self.m.Param(
            value=zeros.copy()
        )

        self.psi_sp = self.m.Param(
            value=zeros.copy()
        )

        self.speed_sp = self.m.Param(
            value=speed.copy()
        )

        # ---------------------------------------------------------------
        # Predicted states
        # ---------------------------------------------------------------

        self.v = self.m.Var(
            value=float(config["v_fixed"]),
            lb=-10.0,
            ub=10.0,
        )

        self.uva = self.m.Var(
            value=float(config["v_fixed"]),
            lb=-10.0,
            ub=10.0,
        )

        self.kappa = self.m.Var(
            value=0.0,
            lb=-self.kappa_limit,
            ub=self.kappa_limit,
        )

        self.psi = self.m.Var(
            value=0.0
        )

        self.xg = self.m.Var(
            value=0.0
        )

        self.yg = self.m.Var(
            value=0.0
        )

        # ---------------------------------------------------------------
        # Manipulated variable: steering command
        # ---------------------------------------------------------------

        self.delta = self.m.MV(
            value=0.0,
            lb=self.steer_min,
            ub=self.steer_max,
        )

        self.delta.STATUS = 1
        self.delta.DMAX = self.dmax
        self.delta.DCOST = float(
            config["mv_dcost"]
        )

        # ---------------------------------------------------------------
        # Dynamic model
        # ---------------------------------------------------------------

        self.m.Equation(
            self.uva.dt()
            == (
                self.speed_sp
                - self.uva
            ) / tau_acc
        )

        self.m.Equation(
            self.v.dt()
            == (
                self.uva
                - self.v
            ) / tau_v
        )

        kappa_target = self.m.Intermediate(
            a1 * self.delta
            + a3 * self.delta**3
        )

        self.m.Equation(
            self.kappa.dt()
            == (
                kappa_target
                - self.kappa
            ) / tau_kappa
        )

        self.m.Equation(
            self.psi.dt()
            == self.v * self.kappa
        )

        self.m.Equation(
            self.xg.dt()
            == self.v
            * self.m.cos(
                self.psi
            )
        )

        self.m.Equation(
            self.yg.dt()
            == self.v
            * self.m.sin(
                self.psi
            )
        )

        # ---------------------------------------------------------------
        # Contouring cost
        # ---------------------------------------------------------------

        ex = self.m.Intermediate(
            self.xg
            - self.x_sp
        )

        ey = self.m.Intermediate(
            self.yg
            - self.y_sp
        )

        contour_error = self.m.Intermediate(
            -self.m.sin(
                self.psi_sp
            ) * ex
            + self.m.cos(
                self.psi_sp
            ) * ey
        )

        lag_error = self.m.Intermediate(
            self.m.cos(
                self.psi_sp
            ) * ex
            + self.m.sin(
                self.psi_sp
            ) * ey
        )

        heading_error = self.m.Intermediate(
            self.psi
            - self.psi_sp
        )

        self.m.Minimize(
            float(config["w_contour"])
            * contour_error**2
        )

        self.m.Minimize(
            float(config["w_lag"])
            * lag_error**2
        )

        self.m.Minimize(
            float(config["w_yaw"])
            * (
                1.0
                - self.m.cos(
                    heading_error
                )
            )
        )

        self.m.Minimize(
            float(config["w_steer"])
            * self.delta**2
        )

        # ---------------------------------------------------------------
        # State estimates and previous command
        # ---------------------------------------------------------------

        self.uva_est = float(
            config["v_fixed"]
        )

        self.last_kappa = 0.0
        self.last_delta = 0.0

        # Keep the same solver behavior as the successful legacy model.
        # delta.value is still explicitly initialized at every solve.
        self.m.options.TIME_SHIFT = 1
        self.m.options.WEB = 0

    def initial_curvature(
        self,
        speed_measured: float,
        yaw_rate_measured: float,
    ) -> float:
        """Estimate current curvature from measured yaw rate.

        For planar motion:

            kappa = yaw_rate / longitudinal_speed

        At very low speed this quotient is unreliable, so the most recent
        valid estimate is retained.
        """

        speed = float(
            speed_measured
        )

        yaw_rate = float(
            yaw_rate_measured
        )

        if (
            np.isfinite(speed)
            and np.isfinite(yaw_rate)
            and abs(speed)
            >= self.kappa_min_speed
        ):
            measured = (
                yaw_rate
                / speed
            )

            self.last_kappa = float(
                np.clip(
                    measured,
                    -self.kappa_limit,
                    self.kappa_limit,
                )
            )

        return self.last_kappa

    def solve(
        self,
        x: float,
        y: float,
        yaw: float,
        speed_measured: float,
        yaw_rate_measured: float,
        x_ref: np.ndarray,
        y_ref: np.ndarray,
        psi_ref: np.ndarray,
        speed_ref: np.ndarray,
    ) -> Tuple[float, bool]:
        psi_shift = (
            round(
                (
                    yaw
                    - float(
                        psi_ref[0]
                    )
                )
                / (
                    2.0
                    * math.pi
                )
            )
            * 2.0
            * math.pi
        )

        psi_aligned = (
            psi_ref
            + psi_shift
        )

        self.x_sp.value = (
            x_ref.tolist()
        )

        self.y_sp.value = (
            y_ref.tolist()
        )

        self.psi_sp.value = (
            psi_aligned.tolist()
        )

        self.speed_sp.value = (
            speed_ref.tolist()
        )

        # Measured initial conditions.
        self.v.value = float(
            speed_measured
        )

        self.psi.value = float(
            yaw
        )

        self.xg.value = float(
            x
        )

        self.yg.value = float(
            y
        )

        self.uva.value = float(
            self.uva_est
        )

        kappa_initial = (
            self.initial_curvature(
                speed_measured,
                yaw_rate_measured,
            )
        )

        self.kappa.value = float(
            kappa_initial
        )

        # Use the same explicit initial steering profile used by the other
        # MPCs. Do not wrap this in first_solve.
        preview_idx = min(
            5,
            self.Np,
        )

        guess = np.clip(
            wrap_angle(
                float(
                    psi_aligned[
                        preview_idx
                    ]
                )
                - yaw
            ),
            self.steer_min,
            self.steer_max,
        )

        self.delta.value = [
            float(guess)
        ] * (
            self.Np + 1
        )

        try:
            self.m.solve(
                disp=False
            )

            success = (
                int(
                    self.m.options.APPSTATUS
                )
                == 1
            )

            if success:
                delta = float(
                    self.delta.NEWVAL
                )
            else:
                delta = (
                    self.last_delta
                )

        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                (
                    "Curvature grey-box "
                    "MPC solve failed: %s"
                ),
                exc,
            )

            success = False
            delta = self.last_delta

        self.last_delta = float(
            np.clip(
                delta,
                self.steer_min,
                self.steer_max,
            )
        )

        return (
            self.last_delta,
            success,
        )

    def update_actuator_estimates(
        self,
        speed_cmd: float,
        _steering_cmd: float,
    ) -> None:
        """Update only the unmeasured longitudinal actuator state.

        Curvature itself is reinitialized from measured yaw rate before
        every optimization.
        """

        tau_acc = max(
            float(
                self.pars["tau_acc"]
            ),
            1e-6,
        )

        self.uva_est += (
            self.Ts
            * (
                float(speed_cmd)
                - self.uva_est
            )
            / tau_acc
        )

class KinematicMPC:
    """Nominal bicycle MPC without empirical slip, bias, gain, or actuator lags."""

    def __init__(self, config: Dict[str, float]) -> None:
        self.Ts = config["Ts"]
        self.Np = int(config["Np"])
        self.steer_min = -config["steer_limit_rad"]
        self.steer_max = config["steer_limit_rad"]

        horizon = np.linspace(0.0, self.Np * self.Ts, self.Np + 1)
        zeros = np.zeros(self.Np + 1)
        speed = np.full(self.Np + 1, config["v_fixed"])

        self.m = GEKKO(remote=False)
        self.m.time = horizon

        self.m.options.IMODE = 6
        self.m.options.NODES = int(config["gekko_nodes"])
        self.m.options.SOLVER = int(config["gekko_solver"])
        self.m.options.SCALING = 1
        self.m.options.MAX_ITER = int(config["max_iter"])
        self.m.options.TIME_SHIFT = 0
        self.m.options.WEB = 0
        self.m.options.NODES = int(config["gekko_nodes"])
        self.m.options.SOLVER = int(config["gekko_solver"])
        self.m.options.SCALING = 1
        self.m.options.MAX_ITER = int(config["max_iter"])
        self.m.solver_options = [
            "print_level 0",
            f"max_iter {int(config['max_iter'])}",
            f"tol {config['solver_tol']}",
            f"acceptable_tol {config['acceptable_tol']}",
        ]
        linear_solver = str(config.get("linear_solver", "")).strip()
        if linear_solver:
            self.m.solver_options.append(f"linear_solver {linear_solver}")

        self.x_sp = self.m.Param(value=zeros.copy())
        self.y_sp = self.m.Param(value=zeros.copy())
        self.psi_sp = self.m.Param(value=zeros.copy())
        self.speed_sp = self.m.Param(value=speed.copy())
        wheelbase = self.m.Param(value=config["wheelbase"])

        self.psi = self.m.Var(value=0.0)
        self.xg = self.m.Var(value=0.0)
        self.yg = self.m.Var(value=0.0)
        self.delta = self.m.MV(value=0.0, lb=self.steer_min, ub=self.steer_max)
        self.delta.STATUS = 1
        self.delta.DMAX = config["steer_dmax_rad_step"]
        self.delta.DCOST = config["mv_dcost"]

        self.m.Equation(self.psi.dt() == (self.speed_sp / wheelbase) * self.m.tan(self.delta))
        self.m.Equation(self.xg.dt() == self.speed_sp * self.m.cos(self.psi))
        self.m.Equation(self.yg.dt() == self.speed_sp * self.m.sin(self.psi))

        ex = self.m.Intermediate(
            self.xg - self.x_sp
        )

        ey = self.m.Intermediate(
            self.yg - self.y_sp
        )

        contour_error = self.m.Intermediate(
            -self.m.sin(self.psi_sp) * ex
            + self.m.cos(self.psi_sp) * ey
        )

        lag_error = self.m.Intermediate(
            self.m.cos(self.psi_sp) * ex
            + self.m.sin(self.psi_sp) * ey
        )

        heading_error = self.m.Intermediate(
            self.psi - self.psi_sp
        )

        self.m.Minimize(
            config["w_contour"]
            * contour_error**2
        )

        self.m.Minimize(
            config["w_lag"]
            * lag_error**2
        )

        self.m.Minimize(
            config["w_yaw"]
            * (
                1.0
                - self.m.cos(heading_error)
            )
        )

        self.m.Minimize(
            config["w_steer"]
            * self.delta**2
        )

        self.last_delta = 0.0

    def solve(
        self,
        x: float,
        y: float,
        yaw: float,
        _speed_measured: float,
        _yaw_rate_measured: float,
        x_ref: np.ndarray,
        y_ref: np.ndarray,
        psi_ref: np.ndarray,
        speed_ref: np.ndarray,
    ) -> Tuple[float, bool]:
        psi_shift = round((yaw - float(psi_ref[0])) / (2.0 * math.pi)) * 2.0 * math.pi
        psi_aligned = psi_ref + psi_shift
        self.x_sp.value = x_ref.tolist()
        self.y_sp.value = y_ref.tolist()
        self.psi_sp.value = psi_aligned.tolist()
        self.speed_sp.value = speed_ref.tolist()
        self.psi.value = float(yaw)
        self.xg.value = float(x)
        self.yg.value = float(y)

        preview_idx = min(5, self.Np)

        guess = np.clip(
            wrap_angle(
                float(psi_aligned[preview_idx])
                - yaw
            ),
            self.steer_min,
            self.steer_max,
        )

        self.delta.value = [
            float(guess)
        ] * (self.Np + 1)

        try:
            self.m.solve(disp=False)
            success = int(self.m.options.APPSTATUS) == 1

            if success:
                delta = float(self.delta.NEWVAL)
            else:
                delta = self.last_delta

        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Kinematic MPC solve failed: %s", exc)
            success = False
            delta = self.last_delta

        self.last_delta = float(np.clip(delta, self.steer_min, self.steer_max))
        return self.last_delta, success

    def update_actuator_estimates(self, _speed_cmd: float, _steering_cmd: float) -> None:
        return


# ---------------------------------------------------------------------------
# Experiment logging
# ---------------------------------------------------------------------------

class ExperimentLogger:
    def __init__(self, controller: str, run_id: str, root_rel: str, config: Dict) -> None:
        pkg_path = rospkg.RosPack().get_path("polaris_system_id")
        self.root = root_rel if os.path.isabs(root_rel) else os.path.join(pkg_path, root_rel)
        self.output_dir = os.path.join(self.root, controller, run_id)
        os.makedirs(self.output_dir, exist_ok=True)
        self.controller = controller
        self.run_id = run_id
        self.config = config
        self.rows = []
        self.finalized = False
        self.solve_failures = 0
        self.termination_reason = "external shutdown"
        with open(os.path.join(self.output_dir, "config.json"), "w") as stream:
            json.dump(config, stream, indent=2, sort_keys=True)
        rospy.on_shutdown(self.finalize)

    def set_termination_reason(self, reason: str) -> None:
        self.termination_reason = str(reason)

    def record(self, row: Dict[str, float]) -> None:
        self.rows.append(row)
        if row.get("did_compute", False) and not row.get("controller_success", True):
            self.solve_failures += 1

    @staticmethod
    def _rmse(values: np.ndarray) -> float:
        return float(np.sqrt(np.mean(values ** 2))) if len(values) else float("nan")

    def finalize(self) -> None:
        if self.finalized:
            return
        self.finalized = True
        if not self.rows:
            rospy.logwarn("No experiment data were recorded")
            return

        fieldnames = list(self.rows[0].keys())
        raw_path = os.path.join(self.output_dir, "timeseries.csv")
        with open(raw_path, "w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.rows)

        def array(name: str) -> np.ndarray:
            return np.asarray([float(row[name]) for row in self.rows], dtype=float)

        t = array("t")
        cte = array("cte")
        heading = array("heading_error")
        speed_error = array("speed_measured") - array("speed_ref")
        steering = array("steering_cmd")
        dt = np.diff(t)
        steering_rate = np.diff(steering) / np.maximum(dt, 1e-9)
        compute = np.asarray(
            [float(row["compute_time_ms"]) for row in self.rows if bool(row["did_compute"])],
            dtype=float,
        )
        successes = [bool(row["controller_success"]) for row in self.rows if bool(row["did_compute"])]
        deadline_ms = 1000.0 * float(self.config["Ts"]) * max(1, int(self.config["solve_every"]))
        transient_s = float(self.config.get("metrics_transient_s", 0.0))
        steady_mask = t >= transient_s
        steady_cte = cte[steady_mask]
        steady_heading = heading[steady_mask]

        summary = {
            "controller": self.controller,
            "run_id": self.run_id,
            "scenario": str(self.config.get("scenario", "nominal")),
            "repeat": int(self.config.get("repeat", 1)),
            "termination_reason": self.termination_reason,
            "duration_s": float(t[-1] - t[0]) if len(t) > 1 else 0.0,
            "samples": int(len(t)),
            "completion_percent": 100.0 * float(array("progress")[-1]),
            "cte_rmse_m": self._rmse(cte),
            "cte_mae_m": float(np.mean(np.abs(cte))),
            "cte_p95_abs_m": float(np.percentile(np.abs(cte), 95)),
            "cte_max_abs_m": float(np.max(np.abs(cte))),
            "cte_rmse_after_transient_m": self._rmse(steady_cte),
            "heading_rmse_deg": math.degrees(self._rmse(heading)),
            "heading_rmse_after_transient_deg": math.degrees(self._rmse(steady_heading)),
            "speed_rmse_mps": self._rmse(speed_error),
            "steering_rms_deg": math.degrees(self._rmse(steering)),
            "steering_rate_rms_deg_s": math.degrees(self._rmse(steering_rate)),
            "steering_total_variation_deg": math.degrees(float(np.sum(np.abs(np.diff(steering))))),
            "compute_mean_ms": float(np.mean(compute)) if len(compute) else 0.0,
            "compute_p95_ms": float(np.percentile(compute, 95)) if len(compute) else 0.0,
            "compute_max_ms": float(np.max(compute)) if len(compute) else 0.0,
            "compute_budget_ms": deadline_ms,
            "deadline_miss_percent": 100.0 * float(np.mean(compute > deadline_ms)) if len(compute) else 0.0,
            "controller_failures": int(sum(not value for value in successes)),
        }
        with open(os.path.join(self.output_dir, "summary.json"), "w") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)

        self._save_plots(t, cte, steering, array("x"), array("y"), array("x_ref"), array("y_ref"))
        rospy.loginfo("Experiment summary saved to %s", self.output_dir)
        rospy.loginfo(
            "[%s] CTE RMSE=%.3f m | p95=%.3f m | compute p95=%.2f ms | completion=%.1f%%",
            self.controller,
            summary["cte_rmse_m"],
            summary["cte_p95_abs_m"],
            summary["compute_p95_ms"],
            summary["completion_percent"],
        )

    def _save_plots(
        self,
        t: np.ndarray,
        cte: np.ndarray,
        steering: np.ndarray,
        x: np.ndarray,
        y: np.ndarray,
        x_ref: np.ndarray,
        y_ref: np.ndarray,
    ) -> None:
        plt.figure(figsize=(9, 4))
        plt.plot(t, cte)
        plt.axhline(0.0, linestyle="--", linewidth=0.8)
        plt.xlabel("Time [s]")
        plt.ylabel("CTE [m]")
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, "cte_over_time.png"), dpi=160)
        plt.close()

        plt.figure(figsize=(9, 4))
        plt.plot(t, np.degrees(steering))
        plt.xlabel("Time [s]")
        plt.ylabel("Steering command [deg]")
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, "steering_over_time.png"), dpi=160)
        plt.close()

        plt.figure(figsize=(7, 7))
        plt.plot(x_ref, y_ref, label="reference")
        plt.plot(x, y, label=self.controller)
        plt.axis("equal")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, "trajectory.png"), dpi=160)
        plt.close()


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class PathTrackingExperimentNode:
    def __init__(self) -> None:
        self.controller_name = str(rospy.get_param("~controller", "greybox_mpc"))
        allowed = {"greybox_mpc", "curvature_greybox_mpc", "kinematic_mpc", "stanley", "pure_pursuit"}
        if self.controller_name not in allowed:
            raise ValueError(f"Unsupported controller '{self.controller_name}'. Choose one of {sorted(allowed)}")

        self.Ts = float(rospy.get_param("~Ts", 0.05))
        self.Np = int(rospy.get_param("~Np", 20))
        self.solve_every = max(1, int(rospy.get_param("~solve_every", 1)))
        self.v_fixed = float(rospy.get_param("~v_fixed", 5.0))
        self.use_curv_speed = bool(rospy.get_param("~use_curv_speed", False))
        self.curv_gain = float(rospy.get_param("~curv_gain", 6.0))
        self.spd_min = float(rospy.get_param("~spd_min", 0.5))
        self.spd_max = float(rospy.get_param("~spd_max", 8.0))
        self.wheelbase = float(rospy.get_param("~wheelbase", 1.75))
        self.steer_limit_rad = math.radians(float(rospy.get_param("~steer_limit_deg", 35.0)))
        self.steer_dmax_rad_step = math.radians(float(rospy.get_param("~steer_slew_deg_step", 12.0)))
        self.preview_points = max(0, int(rospy.get_param("~preview_points", 3)))
        self.search_back = max(0, int(rospy.get_param("~search_back", 30)))
        self.search_forward = max(20, int(rospy.get_param("~search_forward", 500)))
        self.max_run_time = float(rospy.get_param("~max_run_time", 240.0))
        self.max_abs_cte = float(rospy.get_param("~max_abs_cte", 20.0))
        self.metrics_transient_s = float(rospy.get_param("~metrics_transient_s", 5.0))
        self.stop_at_path_end = bool(rospy.get_param("~stop_at_path_end", True))
        self.end_margin_points = max(1, int(rospy.get_param("~end_margin_points", 10)))
        self.odom_twist_frame = str(rospy.get_param("~odom_twist_frame", "body"))
        self.scenario = str(rospy.get_param("~scenario", "nominal"))
        self.repeat = int(rospy.get_param("~repeat", 1))

        self.w_pos = float(rospy.get_param("~w_pos", 2.0))
        self.w_yaw = float(rospy.get_param("~w_yaw", 0.5))
        self.w_steer = float(rospy.get_param("~w_steer", 1e-6))
        self.mv_dcost = float(rospy.get_param("~mv_dcost", 1.0))
        self.w_contour = float(
            rospy.get_param("~w_contour", 2.0)
        )

        self.w_lag = float(
            rospy.get_param("~w_lag", 0.1)
        )

        self.greybox = {
            "tau_acc": float(rospy.get_param("~tau_acc", 0.85716492375)),
            "tau_str": float(rospy.get_param("~tau_str", 0.4119392892)),
            "tau_v": float(rospy.get_param("~tau_v", 0.43809159234)),
            "L_eff": float(rospy.get_param("~L_eff", 2.8168931892)),
            "k_del": float(rospy.get_param("~k_del", 1.666545649)),
            "k_b1": float(rospy.get_param("~k_b1", 0.6)),
            "k_b3": float(rospy.get_param("~k_b3", 1.0)),
            "psi_bias": float(rospy.get_param("~psi_bias", -0.020810914222)),
        }

        self.curvature_greybox = {
            "tau_acc": float(
                rospy.get_param(
                    "~tau_acc",
                    0.05,
                )
            ),

            "tau_v": float(
                rospy.get_param(
                    "~tau_v",
                    1.5580303592291165,
                )
            ),

            "tau_kappa": float(
                rospy.get_param(
                    "~tau_kappa",
                    0.6012253028518952,
                )
            ),

            "a1": float(
                rospy.get_param(
                    "~kappa_a1",
                    0.6007225373,
                )
            ),

            "a3": float(
                rospy.get_param(
                    "~kappa_a3",
                    0.0,
                )
            ),

            "kappa_limit": float(
                rospy.get_param(
                    "~kappa_limit",
                    0.75,
                )
            ),

            "kappa_min_speed": float(
                rospy.get_param(
                    "~kappa_min_speed",
                    0.5,
                )
            ),
        }

        self.stanley_k = float(rospy.get_param("~stanley_k", 1.0))
        self.stanley_softening = float(rospy.get_param("~stanley_softening", 1.0))
        self.stanley_cte_sign = float(rospy.get_param("~stanley_cte_sign", -1.0))
        self.pp_lookahead_base = float(rospy.get_param("~pp_lookahead_base", 3.0))
        self.pp_lookahead_gain = float(rospy.get_param("~pp_lookahead_gain", 0.5))

        self.config = {
            "controller": self.controller_name,
            "Ts": self.Ts,
            "Np": self.Np,
            "solve_every": self.solve_every,
            "v_fixed": self.v_fixed,
            "use_curv_speed": self.use_curv_speed,
            "curv_gain": self.curv_gain,
            "spd_min": self.spd_min,
            "spd_max": self.spd_max,
            "wheelbase": self.wheelbase,
            "steer_limit_rad": self.steer_limit_rad,
            "steer_dmax_rad_step": self.steer_dmax_rad_step,
            "w_pos": self.w_pos,
            "w_yaw": self.w_yaw,
            "w_steer": self.w_steer,
            "mv_dcost": self.mv_dcost,
            "gekko_nodes": int(rospy.get_param("~gekko_nodes", 2)),
            "gekko_solver": int(rospy.get_param("~gekko_solver", 3)),
            "max_iter": int(rospy.get_param("~max_iter", 120)),
            "solver_tol": float(rospy.get_param("~solver_tol", 1e-6)),
            "acceptable_tol": float(rospy.get_param("~acceptable_tol", 1e-4)),
            "linear_solver": str(rospy.get_param("~linear_solver", "ma27")),
            "greybox": self.greybox,
            "curvature_greybox": self.curvature_greybox,
            "stanley_k": self.stanley_k,
            "stanley_softening": self.stanley_softening,
            "pp_lookahead_base": self.pp_lookahead_base,
            "pp_lookahead_gain": self.pp_lookahead_gain,
            "odom_twist_frame": self.odom_twist_frame,
            "scenario": self.scenario,
            "repeat": self.repeat,
            "max_run_time": self.max_run_time,
            "max_abs_cte": self.max_abs_cte,
            "metrics_transient_s": self.metrics_transient_s,
            "stanley_cte_sign": self.stanley_cte_sign,
            "w_contour": self.w_contour,
            "w_lag": self.w_lag,
        }

        run_id = str(rospy.get_param("~run_id", "")).strip()
        if not run_id:
            run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        metrics_root = str(rospy.get_param("~metrics_root", "src/analysis/data/controller_comparison"))
        self.logger = ExperimentLogger(self.controller_name, run_id, metrics_root, self.config)

        self.path: Optional[PathGeometry] = None
        self.speed_profile: Optional[np.ndarray] = None
        self.last_control_idx: Optional[int] = None
        self.last_metric_idx: Optional[int] = None
        self.have_odom = False
        self.yaw_unwrapper = YawUnwrapper()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed_measured = 0.0
        self.yaw_rate_measured = 0.0
        self.run_start_sim: Optional[float] = None
        self.last_steering = 0.0
        self.last_speed = self.v_fixed
        self.step = 0

        self.initial_path_index = max(
            0,
            int(rospy.get_param("~initial_path_index", 0)),
        )

        self.end_margin_m = max(
            0.0,
            float(rospy.get_param("~end_margin_m", 1.0)),
        )

        if self.controller_name == "greybox_mpc":
            self.controller = GreyBoxMPC(self.config)
        elif self.controller_name == "curvature_greybox_mpc":
            self.controller = CurvatureGreyBoxMPC(self.config)
        elif self.controller_name == "kinematic_mpc":
            self.controller = KinematicMPC(self.config)
        else:
            self.controller = None

        self.publisher = rospy.Publisher("/gem/ackermann_cmd", AckermannDrive, queue_size=1)
        rospy.Subscriber("/gem/base_footprint/odom", Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber("/path_xy", Path, self.path_callback, queue_size=1)

        rospy.loginfo(
            "Path-tracking experiment ready: controller=%s, Ts=%.3f s, Np=%d, speed=%.2f m/s",
            self.controller_name,
            self.Ts,
            self.Np,
            self.v_fixed,
        )

    def path_callback(self, msg: Path) -> None:
        if len(msg.poses) < 3:
            rospy.logwarn("Received a path with fewer than three points")
            return
        self.path = PathGeometry.from_ros_path(msg)
        if self.use_curv_speed:
            speed = self.spd_max / (1.0 + self.curv_gain * np.abs(self.path.kappa))
            self.speed_profile = np.clip(speed, self.spd_min, self.spd_max)
        else:
            self.speed_profile = np.full(self.path.size, self.v_fixed, dtype=float)
        self.initial_path_index = int(
            np.clip(
                self.initial_path_index,
                0,
                self.path.size - 2,
            )
        )

        self.last_control_idx = self.initial_path_index
        self.last_metric_idx = self.initial_path_index

        self.path_start_s = float(
            self.path.s[self.initial_path_index]
        )

        self.path_length_from_start = max(
            float(self.path.s[-1] - self.path_start_s),
            1e-9,
        )
        rospy.loginfo("Reference path loaded with %d points", self.path.size)

    def odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw_raw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )[2]
        yaw = self.yaw_unwrapper.update(yaw_raw)
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        yaw_rate = float(msg.twist.twist.angular.z)
        if self.odom_twist_frame == "world":
            speed = vx * math.cos(yaw) + vy * math.sin(yaw)
        elif self.odom_twist_frame == "magnitude":
            speed = math.copysign(math.hypot(vx, vy), vx)
        else:
            speed = vx

        self.x = float(position.x)
        self.y = float(position.y)
        self.yaw = float(yaw)
        self.speed_measured = float(speed)
        self.have_odom = True
        self.speed_measured = float(speed)
        self.yaw_rate_measured = float(yaw_rate)
        self.have_odom = True

    def speed_at(self, index: int) -> float:
        assert self.speed_profile is not None
        return float(self.speed_profile[min(max(index, 0), len(self.speed_profile) - 1)])

    def rate_limit_steering(self, desired: float) -> float:
        desired = float(np.clip(desired, -self.steer_limit_rad, self.steer_limit_rad))
        delta = float(np.clip(desired - self.last_steering,
                              -self.steer_dmax_rad_step,
                              self.steer_dmax_rad_step))
        return float(np.clip(self.last_steering + delta,
                             -self.steer_limit_rad,
                             self.steer_limit_rad))

    def compute_classical(self, idx: int) -> Tuple[float, bool]:
        assert self.path is not None
        if self.controller_name == "stanley":
            idx_front, _s_front, cte_front, heading_error = self.path.tracking_errors(
                self.x,
                self.y,
                self.yaw,
                self.last_control_idx,
                self.search_back,
                self.search_forward,
                axle_offset=self.wheelbase,
            )
            self.last_control_idx = idx_front
            speed = max(abs(self.speed_measured), 0.0)
            correction = math.atan2(
                self.stanley_k * self.stanley_cte_sign * cte_front,
                speed + self.stanley_softening,
            )
            desired = heading_error + correction
            return self.rate_limit_steering(desired), True

        lookahead = self.pp_lookahead_base + self.pp_lookahead_gain * abs(self.speed_measured)
        target_idx = self.path.target_index_by_distance(idx, lookahead)
        dx = float(self.path.x[target_idx] - self.x)
        dy = float(self.path.y[target_idx] - self.y)
        alpha = wrap_angle(math.atan2(dy, dx) - self.yaw)
        actual_lookahead = max(math.hypot(dx, dy), 1e-3)
        desired = math.atan2(2.0 * self.wheelbase * math.sin(alpha), actual_lookahead)
        return self.rate_limit_steering(desired), True

    def publish(self, steering: float, speed: float) -> None:
        msg = AckermannDrive()
        msg.steering_angle = float(steering)
        msg.speed = float(speed)
        self.publisher.publish(msg)

    def publish_stop(self) -> None:
        self.publish(0.0, 0.0)

    def spin(self) -> None:
        rate = rospy.Rate(max(1, int(round(1.0 / self.Ts))))
        while not rospy.is_shutdown():
            if self.path is None or self.speed_profile is None or not self.have_odom:
                rate.sleep()
                continue

            if self.run_start_sim is None:
                self.run_start_sim = rospy.Time.now().to_sec()

            idx, s_projected, cte, heading_error = self.path.tracking_errors(
                self.x,
                self.y,
                self.yaw,
                self.last_metric_idx,
                self.search_back,
                self.search_forward,
                axle_offset=0.0,
            )
            self.last_metric_idx = idx
            if self.last_control_idx is None:
                self.last_control_idx = idx

            start_idx = min(idx + self.preview_points, self.path.size - 1)
            speed_cmd = self.speed_at(start_idx)
            did_compute = False
            success = True
            compute_time_ms = 0.0

            should_compute = self.controller_name in {"stanley", "pure_pursuit"} or self.step % self.solve_every == 0
            if should_compute:
                did_compute = True
                wall_start = time.perf_counter()
                if self.controller_name in {"greybox_mpc", "curvature_greybox_mpc", "kinematic_mpc"}:
                    assert self.controller is not None
                    (
                        x_ref,
                        y_ref,
                        psi_ref,
                        speed_ref,
                        target_s,
                    ) = self.path.horizon_from_s(
                        start_s=s_projected,
                        speed_profile=self.speed_profile,
                        Ts=self.Ts,
                        count=self.Np + 1,
                    )
                    steering, success = self.controller.solve(
                        self.x,
                        self.y,
                        self.yaw,
                        self.speed_measured,
                        self.yaw_rate_measured,
                        x_ref,
                        y_ref,
                        psi_ref,
                        speed_ref,
                    )
                else:
                    steering, success = self.compute_classical(idx)
                compute_time_ms = 1000.0 * (time.perf_counter() - wall_start)
                self.last_steering = float(steering)
                self.last_speed = float(speed_cmd)

            self.publish(self.last_steering, self.last_speed)
            if self.controller is not None:
                self.controller.update_actuator_estimates(self.last_speed, self.last_steering)

            nearest_idx = min(idx, self.path.size - 1)
            elapsed = rospy.Time.now().to_sec() - self.run_start_sim
            covered_distance = max(
                0.0,
                float(
                    s_projected
                    - self.path_start_s
                ),
            )

            progress = float(
                np.clip(
                    covered_distance
                    / self.path_length_from_start,
                    0.0,
                    1.0,
                )
            )

            remaining_distance = max(
                0.0,
                float(
                    self.path.s[-1]
                    - s_projected
                ),
            )

            reached_end = (
                remaining_distance
                <= self.end_margin_m
            )

            if abs(self.speed_measured) >= 0.5:
                kappa_measured = (
                    self.yaw_rate_measured
                    / self.speed_measured
                )
            else:
                kappa_measured = 0.0
            self.logger.record({
                "t": float(elapsed),
                "x": self.x,
                "y": self.y,
                "yaw": self.yaw,
                "speed_measured": self.speed_measured,
                "x_ref": float(self.path.x[nearest_idx]),
                "y_ref": float(self.path.y[nearest_idx]),
                "speed_ref": self.speed_at(nearest_idx),
                "path_index": int(nearest_idx),
                "progress": progress,
                "path_dist_m": covered_distance,
                "path_remaining_m": remaining_distance,
                "cte": cte,
                "heading_error": heading_error,
                "steering_cmd": self.last_steering,
                "speed_cmd": self.last_speed,
                "compute_time_ms": compute_time_ms,
                "did_compute": bool(did_compute),
                "controller_success": bool(success),
                "yaw_rate_measured": (self.yaw_rate_measured),
                "kappa_measured": float(kappa_measured),
            })

            timed_out = elapsed >= self.max_run_time
            diverged = elapsed > self.metrics_transient_s and abs(cte) >= self.max_abs_cte
            if (self.stop_at_path_end and reached_end and elapsed > 5.0) or timed_out or diverged:
                if reached_end:
                    reason = "path completed"
                elif diverged:
                    reason = f"diverged: |CTE| >= {self.max_abs_cte:.2f} m"
                else:
                    reason = "maximum run time reached"
                rospy.loginfo("Stopping experiment: %s", reason)
                self.logger.set_termination_reason(reason)
                self.publish_stop()
                rospy.signal_shutdown(reason)
                break

            self.step += 1
            rate.sleep()


def main() -> None:
    rospy.init_node("path_tracking_experiment")
    try:
        PathTrackingExperimentNode().spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
