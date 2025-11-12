#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MPC ROS node.
- Mode 'steer_only': controla apenas direção; velocidade é passada como perfil conhecido (constante por padrão).
- Mode 'speed_steer': controla direção e velocidade simultaneamente.
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


# ----------  MPC builder:  ---------- #

def mpc_xypsi(t, pars, x_ref, y_ref, psi_ref, v_ref,
              Ts=None, Np=20, mode='steer_only',
              steer_bounds=(-0.6,0.6), dmax_steer=0.20,
              spd_bounds=(0.0,8.0), dmax_spd=0.6,
              w_pos=1.0, w_yaw=0.3, w_v=0.2,
              w_u_steer=1e-6, w_u_spd=1e-6):
    """
    GEKKO MPC that tracks (x, y, psi) and optionally v.
      mode='steer_only'  -> optimize steering only; v_ref is a known profile
      mode='speed_steer' -> optimize steering + speed
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
    m.options.SOLVER  = 3
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

    # horizon references
    x_sp   = m.Param(value=x_ref[:Np+1].copy())
    y_sp   = m.Param(value=y_ref[:Np+1].copy())
    psi_sp = m.Param(value=psi_ref[:Np+1].copy())
    v_sp   = m.Param(value=v_ref[:Np+1].copy())

    # states
    v   = m.Var(value=float(v_ref[0]), lb=-10, ub=10)
    psi = m.Var(value=float(psi_ref[0]))
    xg  = m.Var(value=float(x_ref[0]))
    yg  = m.Var(value=float(y_ref[0]))
    uva = m.Var(value=float(v_ref[0]), lb=-10, ub=10)  # speed actuator state
    da  = m.Var(value=0.0, lb=steer_bounds[0], ub=steer_bounds[1])

    # MV: steering
    delt_u = m.MV(value=0.0, lb=steer_bounds[0], ub=steer_bounds[1])
    delt_u.STATUS = 1
    delt_u.DMAX   = dmax_steer
    delt_u.DCOST  = 1.0

    # speed: MV (speed_steer) or Param (steer_only)
    if mode == 'speed_steer':
        spd_u = m.MV(value=float(v_ref[0]), lb=spd_bounds[0], ub=spd_bounds[1])
        spd_u.STATUS = 1
        spd_u.DMAX   = dmax_spd
        spd_u.DCOST  = 1.0
        spd_src = spd_u
    else:
        spd_param = m.Param(value=v_ref[:Np+1].copy())
        spd_src   = spd_param

    # dynamics
    m.Equation(uva.dt() == (spd_src - uva)/tau_acc)
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
    m.Minimize(w_u_steer*(delt_u**2))
    if mode == 'speed_steer':
        m.Minimize(w_v*(v - v_sp)**2)
        m.Minimize(w_u_spd*(spd_u**2))

    # APIs to update horizon and solve
    def set_refs_and_speed(xseg, yseg, psiseg, vseg):
        x_sp.value   = xseg[:Np+1]
        y_sp.value   = yseg[:Np+1]
        psi_sp.value = psiseg[:Np+1]
        v_sp.value   = vseg[:Np+1]
        if mode == 'steer_only':
            spd_param.value = vseg[:Np+1]

    def solve_first_move(v0, psi0, x0, y0, uva0, da0):
        v.value, psi.value, xg.value, yg.value = v0, psi0, x0, y0
        uva.value, da.value = uva0, da0
        # warm-start steering toward yaw error
        head_err = float((psi_sp.value[min(5,Np)] - psi0 + np.pi)%(2*np.pi) - np.pi)
        d_guess  = np.clip(head_err, steer_bounds[0], steer_bounds[1])
        delt_u.value = [d_guess]*(Np+1)
        if mode == 'speed_steer':
            spd_u.value = list(np.clip(v_sp.value, spd_bounds[0], spd_bounds[1]))
        m.solve(disp=False)
        if mode == 'speed_steer':
            return float(spd_u.NEWVAL), float(delt_u.NEWVAL)
        else:
            return float(v_sp.value[0]), float(delt_u.NEWVAL)

    return set_refs_and_speed, solve_first_move, dict(Ts=Ts, Np=Np, mode=mode)


# ------------------------------ ROS Node ------------------------------ #

class MPCNode:
    def __init__(self):
        # --- params ---
        self.frame = rospy.get_param('~fixed_frame', 'world')
        self.Ts    = float(rospy.get_param('~Ts', 0.05))  # 20 Hz default
        self.Np    = int(rospy.get_param('~Np', 20))
        self.solve_every = int(rospy.get_param('~solve_every', 3))
        self.v_fixed = float(rospy.get_param('~v_fixed', 5.0))
        self.mode    = str(rospy.get_param('~mode', 'steer_only'))  # 'steer_only' | 'speed_steer'

        # self.L = float(rospy.get_param('~L', 1.75))
        self.L = float(rospy.get_param('~L', 2.8168931892))
        self.steer_deg = float(rospy.get_param('~steer_limit_deg', 35.0))
        self.dmax_deg  = float(rospy.get_param('~steer_slew_deg', 12.0))  # per control step
        self.n_points  = int(rospy.get_param('~csv_points', 0))  # 0 -> load all
        self.spd_min   = float(rospy.get_param('~spd_min', 0.0))
        self.spd_max   = float(rospy.get_param('~spd_max', 8.0))
        self.dmax_spd  = float(rospy.get_param('~spd_slew', 0.6))  # m/s per control step

        # opcional: v_ref por curvatura
        self.use_curv_speed = bool(rospy.get_param('~use_curv_speed', False))
        self.curv_gain      = float(rospy.get_param('~curv_gain', 6.0))

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
        self.v_ref = None

        # --- build MPC ---
        t_stub = np.arange(0.0, (self.Np)*self.Ts + 1e-9, self.Ts)
        if not self.have_path:
            xr = np.zeros_like(t_stub); yr = np.zeros_like(t_stub); pr = np.zeros_like(t_stub)
            vcmd = np.full_like(t_stub, self.v_fixed, float)
        else:
            xr = self.x_ref[:self.Np+1]; yr = self.y_ref[:self.Np+1]; pr = self.psi_ref[:self.Np+1]
            vcmd = self.v_cmd[:self.Np+1]

        self.set_hor, self.solve_move, self.info = mpc_xypsi(
            t=t_stub, pars=self.pars,
            x_ref=xr, y_ref=yr, psi_ref=pr, v_ref=vcmd,
            Ts=self.Ts, Np=self.Np, mode=self.mode,
            steer_bounds=steer_bounds, dmax_steer=dmax_steer,
            spd_bounds=(self.spd_min, self.spd_max), dmax_spd=self.dmax_spd,
            w_pos=2.0, w_yaw=0.5, w_v=0.6,
            w_u_steer=1e-6, w_u_spd=1e-6
        )

        # --- state / actuator memories ---
        self.x = np.zeros(4)  # [x,y,psi_unwrapped,v_body]  (v_body will evolve internally)
        self.yaw_unwrap_state = [None, 0.0]
        self.uva = self.v_fixed    # speed actuator state
        self.da  = 0.0             # steering actuator state
        self.u_del_hold = 0.0
        self.u_spd_hold = self.v_fixed

        # --- ROS I/O ---
        self.pub = rospy.Publisher('/gem/ackermann_cmd', AckermannDrive, queue_size=1)
        rospy.Subscriber('/gem/base_footprint/odom', Odometry, self.cb_odom, queue_size=1)
        rospy.Subscriber('/path_xy', Path, self.cb_path, queue_size=1)

        self.metrics = CTEMetrics(
            save_rel=rospy.get_param("~metrics_rel", "src/analysis/data/cte_metrics"),
            signed=True
        )

        rospy.loginfo(f"[MPC] Node ready (mode={self.mode}). Publishing /gem/ackermann_cmd")

    # --------------- Callbacks --------------- #
    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        yaw = unwrap(yaw, self.yaw_unwrap_state)
        self.x[0] = p.x
        self.x[1] = p.y
        self.x[2] = yaw
        if hasattr(self, "metrics"):
            self.metrics.update_odom(msg)

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
        # perfil de velocidade (constante ou por curvatura)
        if self.use_curv_speed:
            dx  = np.gradient(xr);  dy  = np.gradient(yr)
            ddx = np.gradient(dx);  ddy = np.gradient(dy)
            kappa = np.abs(dx*ddy - dy*ddx) / np.maximum((dx*dx + dy*dy)**1.5, 1e-6)
            vref = self.spd_max / (1.0 + self.curv_gain * kappa)
            vref = np.clip(vref, self.spd_min, self.spd_max)
            self.v_ref = vref.astype(float)
        else:
            self.v_ref = np.full(M, self.v_fixed, float)
        self.have_path = True
        if hasattr(self, "metrics"):
            self.metrics.update_path(path)

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
            vseg   = pad_slice(self.v_ref,   i0, self.Np+1)

            if step % self.solve_every == 0:
                self.set_hor(xseg, yseg, psiseg, vseg)
                u_spd, u_del = self.solve_move(self.x[3], self.x[2], self.x[0], self.x[1],
                                               self.uva, self.da)
                self.u_spd_hold = float(u_spd)   # pode ser igual ao vseg[0] no modo steer_only
                self.u_del_hold = float(u_del)

            self.publish_cmd(self.u_del_hold, self.u_spd_hold)

            # advance internal actuator + vehicle states by Ts (Euler)
            Ts = self.Ts
            self.uva = self.uva + Ts*(self.u_spd_hold - self.uva)/max(self.pars['tau_acc'],1e-6)
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



# ====  CTE Metrics ====
import os, csv, numpy as np
import matplotlib
matplotlib.use('Agg')  # seguro em Docker/headless
import matplotlib.pyplot as plt
import rospkg

class CTEMetrics:
    def __init__(self, save_rel="src/analysis/data/cte_metrics", signed=True):
        pkg_path = rospkg.RosPack().get_path("polaris_system_id")
        self.save_dir = os.path.join(pkg_path, save_rel)
        os.makedirs(self.save_dir, exist_ok=True)
        self.signed = signed

        self.P = None
        self.seg_p0 = None
        self.seg_v  = None
        self.seg_l2 = None
        self.ts, self.cte = [], []

        rospy.on_shutdown(self.finalize)

    def update_path(self, msg: Path):
        if len(msg.poses) < 2:
            return
        P = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=float)
        self.P = P
        self.seg_p0 = P[:-1]
        self.seg_v  = P[1:] - P[:-1]
        self.seg_l2 = np.einsum('ij,ij->i', self.seg_v, self.seg_v) + 1e-12
        rospy.loginfo("[CTE] Path received with %d points", len(P))

    def update_odom(self, msg: Odometry):
        if self.seg_v is None:
            return
        qx = msg.pose.pose.position.x
        qy = msg.pose.pose.position.y
        t  = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()

        q  = np.array([qx, qy])
        w  = q - self.seg_p0
        s  = np.clip(np.einsum('ij,ij->i', w, self.seg_v) / self.seg_l2, 0.0, 1.0)
        proj = self.seg_p0 + (self.seg_v.T * s).T
        dvec = q - proj
        d    = np.hypot(dvec[:,0], dvec[:,1])
        k    = int(np.argmin(d))

        cte = float(d[k])
        if self.signed:
            crossz = self.seg_v[k,0]*dvec[k,1] - self.seg_v[k,1]*dvec[k,0]
            cte *= 1.0 if crossz >= 0.0 else -1.0

        self.ts.append(t)
        self.cte.append(cte)
        

    def finalize(self):
        if not self.cte:
            rospy.logwarn("[CTE] No data to save")
            return
        t0   = self.ts[0]
        t    = np.array(self.ts) - t0
        cte  = np.array(self.cte)
        rmse = float(np.sqrt(np.mean(cte**2)))

        # CSV
        csv_path = os.path.join(self.save_dir, "cte_log.csv")
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f); w.writerow(["t","cte"])
            for ti, ei in zip(t, cte): w.writerow([f"{ti:.6f}", f"{ei:.6f}"])

        # TXT
        with open(os.path.join(self.save_dir, "rmse_cte.txt"), "w") as f:
            f.write(f"{rmse:.6f}\n")

        # Figura
        plt.figure(figsize=(9,4))
        plt.plot(t, cte, lw=1.2)
        plt.axhline(0, ls="--", lw=0.8)
        plt.grid(True, alpha=0.3)
        plt.xlabel("tempo (s)")
        plt.ylabel("CTE (m)")
        plt.title(f"Cross-Track Error — RMSE = {rmse:.3f} m")
        plt.tight_layout()
        fig_path = os.path.join(self.save_dir, "cte_over_time.png")
        plt.savefig(fig_path, dpi=150); plt.close()

        rospy.loginfo("[CTE] RMSE=%.3f m | CSV: %s | FIG: %s", rmse, csv_path, fig_path)


# ------------------------------ main ------------------------------ #

def main():
    rospy.init_node('mpc_node')
    try:
        node = MPCNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()