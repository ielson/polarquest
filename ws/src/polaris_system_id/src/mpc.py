#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Steering-only MPC ROS node.
- Tracks (x,y,psi) using steering; speed is fixed (v_fixed) and published.
- Waypoints from from /path_xy (nav_msgs/Path).
- Publishes AckermannDrive on /gem/ackermann_cmd.
- Uses parameters identified before at system_identification.ipynb

"""

import math
import numpy as np

import rospy
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDrive
from tf.transformations import euler_from_quaternion

from gekko import GEKKO

# ------------------------------ Utils ------------------------------ #

def unwrap(yaw, yaw_prev=[None,0.0]):
    """Simple yaw unwrapping."""
    if yaw_prev[0] is None:
        yaw_prev[0] = yaw
        return yaw
    dy = yaw - yaw_prev[0]
    if   dy >  math.pi: yaw_prev[1] -= 2*math.pi
    elif dy < -math.pi: yaw_prev[1] += 2*math.pi
    yaw_prev[0] = yaw
    return yaw + yaw_prev[1]

def nearest_front_axle_idx(x_ref, y_ref, x, y, psi, L):
    """Closest path index to the front-axle point (reduces CTE bias)."""
    fx = x + L*np.cos(psi)
    fy = y + L*np.sin(psi)
    d2 = (x_ref-fx)**2 + (y_ref-fy)**2
    return int(np.argmin(d2))

def psi_from_xy(x, y):
    dx = np.gradient(x); dy = np.gradient(y)
    return np.unwrap(np.arctan2(dy, dx))

def pad_slice(a, i0, n):
    """Return a[i0:i0+n], clamped to [0,len(a)-1] and padded by repeating the last value."""
    a = np.asarray(a, dtype=float)
    if a.size == 0 or n <= 0:
        return np.empty((0,), dtype=float)
    i0 = int(np.clip(i0, 0, len(a)-1))
    idx = np.arange(i0, i0+n, dtype=int)
    idx = np.clip(idx, 0, len(a)-1)
    return a[idx]


# ---------- (B) GEKKO MPC builder: steering-only, tracks x,y,psi ---------- #

def mpc_xypsi_steer_only(t, pars, x_ref, y_ref, psi_ref, v_cmd,
                         Ts=None, Np=20,
                         steer_bounds=(-0.6,0.6), dmax_steer=0.20,
                         w_pos=1.0, w_yaw=0.3, w_u=1e-6):
    """
    Steering-only MPC that tracks (x, y, psi). Speed is a known input v_cmd(t).
    'pars' are fixed model params (actuators + slip). If None, sensible defaults.
    """
    if Ts is None:
        Ts = float(np.median(np.diff(t)))
    H = np.linspace(0.0, Np*Ts, Np+1)

    if pars is None:
        pars = dict(tau_acc=0.20, tau_str=0.15, tau_v=0.5, L_eff=1.75,
                    k_del=1.0, k_b1=0.0, k_b3=0.0, psi_bias=0.0)

    m = GEKKO(remote=False)
    m.time = H
    m.options.IMODE   = 6
    m.options.NODES   = 2
    m.options.SOLVER  = 3           # IPOPT
    m.options.SCALING = 1
    m.options.MAX_ITER = 120
    m.solver_options  = ['print_level 0','max_iter 120',
                         'tol 1e-6','acceptable_tol 1e-4','linear_solver ma27']

    # fixed params
    tau_acc = m.Param(value=pars['tau_acc'])
    tau_str = m.Param(value=pars['tau_str'])
    tau_v   = m.Param(value=pars['tau_v'])
    L_eff   = m.Param(value=pars['L_eff'])
    k_del   = m.Param(value=pars['k_del'])
    k_b1    = m.Param(value=pars['k_b1'])
    k_b3    = m.Param(value=pars['k_b3'])
    psi_b   = m.Param(value=pars['psi_bias'])

    # refs + known speed over horizon
    x_sp   = m.Param(value=x_ref[:Np+1].copy())
    y_sp   = m.Param(value=y_ref[:Np+1].copy())
    psi_sp = m.Param(value=psi_ref[:Np+1].copy())
    u_vp   = m.Param(value=v_cmd[:Np+1].copy())

    # states
    v   = m.Var(value=float(v_cmd[0]), lb=-10, ub=10)
    psi = m.Var(value=float(psi_ref[0]))
    xg  = m.Var(value=float(x_ref[0]))
    yg  = m.Var(value=float(y_ref[0]))
    uva = m.Var(value=float(v_cmd[0]), lb=-10, ub=10)
    da  = m.Var(value=0.0, lb=steer_bounds[0], ub=steer_bounds[1])

    # MV: steering command
    delt_u = m.MV(value=0.0, lb=steer_bounds[0], ub=steer_bounds[1])
    delt_u.STATUS = 1
    delt_u.DMAX   = dmax_steer
    delt_u.DCOST  = 1.0

    # dynamics
    m.Equation(uva.dt() == (u_vp - uva)/tau_acc)
    m.Equation(da.dt()  == (delt_u - da)/tau_str)
    m.Equation(v.dt()   == (uva - v)/tau_v)
    m.Equation(psi.dt() == (v/L_eff) * m.tan(k_del*da))
    beta = m.Intermediate(k_b1*da + k_b3*da**3)
    m.Equation(xg.dt()  == v*m.cos(psi + psi_b + beta))
    m.Equation(yg.dt()  == v*m.sin(psi + psi_b + beta))

    # objective
    ex = m.Intermediate(xg - x_sp)
    ey = m.Intermediate(yg - y_sp)
    dpsi = m.Intermediate(psi - psi_sp)
    m.Minimize(w_pos*(ex**2 + ey**2))
    m.Minimize(w_yaw*(1 - m.cos(dpsi)))
    m.Minimize(w_u*(delt_u**2))

    # APIs
    def set_refs_and_speed(xseg, yseg, psiseg, vseg):
        x_sp.value   = xseg[:Np+1]
        y_sp.value   = yseg[:Np+1]
        psi_sp.value = psiseg[:Np+1]
        u_vp.value   = vseg[:Np+1]

    def solve_first_move(v0, psi0, x0, y0, uva0, da0):
        # ICs
        v.value, psi.value, xg.value, yg.value = v0, psi0, x0, y0
        uva.value, da.value = uva0, da0
        # warm-start: steer in the direction of yaw error K steps ahead
        head_err = float((psi_sp.value[min(5,Np)] - psi0 + np.pi)%(2*np.pi) - np.pi)
        d_guess  = np.clip(head_err, steer_bounds[0], steer_bounds[1])
        delt_u.value = [d_guess]*(Np+1)
        m.solve(disp=False)
        return float(delt_u.NEWVAL)

    return set_refs_and_speed, solve_first_move, dict(Ts=Ts, Np=Np)

# ------------------------------ ROS Node ------------------------------ #

class SteerOnlyMPCNode:
    def __init__(self):
        # --- params ---
        self.frame = rospy.get_param('~fixed_frame', 'world')
        self.Ts    = float(rospy.get_param('~Ts', 0.05))  # 20 Hz default
        self.Np    = int(rospy.get_param('~Np', 20))
        self.solve_every = int(rospy.get_param('~solve_every', 3))
        self.v_fixed = float(rospy.get_param('~v_fixed', 5.0))
        # self.L = float(rospy.get_param('~L', 1.75))
        self.L = float(rospy.get_param('~L', 2.8168931892))
        self.steer_deg = float(rospy.get_param('~steer_limit_deg', 35.0))
        self.dmax_deg  = float(rospy.get_param('~steer_slew_deg', 12.0))  # per control step
        self.n_points  = int(rospy.get_param('~csv_points', 0))  # 0 -> load all

        self.pars = dict(
            tau_acc  = float(rospy.get_param('~tau_acc', 0.85716492375)),
            tau_str  = float(rospy.get_param('~tau_str', 0.4119392892)),
            tau_v    = float(rospy.get_param('~tau_v', .43809159234)),
            L_eff    = float(rospy.get_param('~L_eff', self.L)),
            k_del    = float(rospy.get_param('~k_del', 1.666545649)),
            k_b1     = float(rospy.get_param('~k_b1', 0.6)),
            k_b3     = float(rospy.get_param('~k_b3', 1.0)),
            psi_bias = float(rospy.get_param('~psi_bias', -0.020810914222)),
        )

        steer_bounds = (-math.radians(self.steer_deg), math.radians(self.steer_deg))
        dmax_steer   = math.radians(self.dmax_deg)

        # --- path / references ---
        self.have_path = False
        self.x_ref = None; self.y_ref = None; self.psi_ref = None
        self.v_cmd = None

        # --- build MPC ---
        t_stub = np.arange(0.0, (self.Np)*self.Ts + 1e-9, self.Ts)
        if not self.have_path:
            xr = np.zeros_like(t_stub); yr = np.zeros_like(t_stub); pr = np.zeros_like(t_stub)
            vcmd = np.full_like(t_stub, self.v_fixed, float)
        else:
            xr = self.x_ref[:self.Np+1]; yr = self.y_ref[:self.Np+1]; pr = self.psi_ref[:self.Np+1]
            vcmd = self.v_cmd[:self.Np+1]

        self.set_hor, self.solve_move, self.info = mpc_xypsi_steer_only(
            t=t_stub, pars=self.pars,
            x_ref=xr, y_ref=yr, psi_ref=pr, v_cmd=vcmd,
            Ts=self.Ts, Np=self.Np,
            steer_bounds=steer_bounds, dmax_steer=dmax_steer,
            w_pos=2.0, w_yaw=0.5, w_u=1e-6
        )

        # --- state / actuator memories ---
        self.x = np.zeros(4)  # [x,y,psi_unwrapped,v_body]  (v_body will evolve internally)
        self.yaw_unwrap_state = [None, 0.0]
        self.uva = self.v_fixed    # speed actuator state
        self.da  = 0.0             # steering actuator state
        self.u_del_hold = 0.0

        # --- ROS I/O ---
        self.pub = rospy.Publisher('/gem/ackermann_cmd', AckermannDrive, queue_size=1)
        rospy.Subscriber('/gem/base_footprint/odom', Odometry, self.cb_odom, queue_size=1)
        rospy.Subscriber('/path_xy', Path, self.cb_path, queue_size=1)

        rospy.loginfo("[MPC] Node ready (steering-only). Publishing /gem/ackermann_cmd")

    # --------------- Callbacks --------------- #
    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        yaw = unwrap(yaw, self.yaw_unwrap_state)
        self.x[0] = p.x
        self.x[1] = p.y
        self.x[2] = yaw
        # leaving x[3] (v) to evolve by actuator model; not using odom v here

    def cb_path(self, path: Path):
        M = len(path.poses)
        if M < 3:
            return
        xr = np.zeros(M); yr = np.zeros(M)
        for i, ps in enumerate(path.poses):
            xr[i] = ps.pose.position.x
            yr[i] = ps.pose.position.y
        pr = psi_from_xy(xr, yr)
        self.x_ref, self.y_ref, self.psi_ref = xr, yr, pr
        total_len = np.sum(np.hypot(np.diff(xr), np.diff(yr)))
        total_time = total_len / max(self.v_fixed,1e-6)
        self.t = np.arange(0.0, total_time + 0.5*self.Ts, self.Ts)
        self.v_cmd = np.full_like(self.t, self.v_fixed, float)
        self.have_path = True

    # --------------- Control Loop --------------- #
    def spin(self):
        r = rospy.Rate(int(round(1.0/self.Ts)))
        step = 0
        while not rospy.is_shutdown():
            if not self.have_path or self.x_ref is None:
                r.sleep(); continue

            # choose along-track index near front axle; add a small preview offset
            idx0 = nearest_front_axle_idx(self.x_ref, self.y_ref, self.x[0], self.x[1], self.x[2], self.L)
            off  = 3
            i0   = min(idx0 + off, len(self.x_ref) - 1)  # clamp
            xseg   = pad_slice(self.x_ref,   i0, self.Np+1)
            yseg   = pad_slice(self.y_ref,   i0, self.Np+1)
            psiseg = pad_slice(self.psi_ref, i0, self.Np+1)
            vseg   = np.full((self.Np+1,), self.v_fixed, float)

            if step % self.solve_every == 0:
                self.set_hor(xseg, yseg, psiseg, vseg)
                u_del = self.solve_move(self.x[3], self.x[2], self.x[0], self.x[1],
                                        self.uva, self.da)
                self.u_del_hold = u_del

            self.publish_cmd(self.u_del_hold, self.v_fixed)

            # advance internal actuator + vehicle states by Ts (Euler)
            Ts = self.Ts
            self.uva = self.uva + Ts*(self.v_fixed - self.uva)/max(self.pars['tau_acc'],1e-6)
            self.da  = self.da  + Ts*(self.u_del_hold - self.da)/max(self.pars['tau_str'],1e-6)
            self.x[3]= self.x[3] + Ts*(self.uva - self.x[3])/max(self.pars['tau_v'],1e-6)  # v_body
            beta = self.pars['k_b1']*self.da + self.pars['k_b3']*(self.da**3)
            self.x[0]= self.x[0] + Ts*(self.x[3]*np.cos(self.x[2] + self.pars['psi_bias'] + beta))
            self.x[1]= self.x[1] + Ts*(self.x[3]*np.sin(self.x[2] + self.pars['psi_bias'] + beta))
            self.x[2]= self.x[2] + Ts*(self.x[3]/self.pars['L_eff'])*np.tan(self.pars['k_del']*self.da)

            step += 1
            r.sleep()

    def publish_cmd(self, delta, speed):
        msg = AckermannDrive()
        msg.steering_angle = float(np.clip(delta,
                                -math.radians(self.steer_deg),
                                 math.radians(self.steer_deg)))
        msg.speed = float(speed)
        self.pub.publish(msg)

# ------------------------------ main ------------------------------ #

def main():
    rospy.init_node('gekko_mpc_steer_only')
    try:
        node = SteerOnlyMPCNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()