#!/usr/bin/env python3
"""Compare the identified grey-box and nominal bicycle models open loop.

Expected CSV columns (same names used in the notebook):
  t, speed_cmd, steering_cmd, v_measured, yaw, x, y

Use a validation run that was NOT used to identify the grey-box parameters.
"""

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Dict, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


def read_csv(path: Path) -> Dict[str, np.ndarray]:
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"t", "speed_cmd", "steering_cmd", "v_measured", "yaw", "x", "y"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"Missing CSV columns: {sorted(missing)}")
        rows = list(reader)
    if len(rows) < 3:
        raise ValueError("Validation CSV must contain at least three samples")
    return {
        name: np.asarray([float(row[name]) for row in rows], dtype=float)
        for name in required
    }


def load_yaml(path: Path) -> Dict[str, float]:
    with path.open() as stream:
        data = yaml.safe_load(stream)
    return {key: float(value) for key, value in data.items()}


def rk4_step(state: np.ndarray, command: Tuple[float, float], dt: float, dynamics) -> np.ndarray:
    k1 = dynamics(state, command)
    k2 = dynamics(state + 0.5 * dt * k1, command)
    k3 = dynamics(state + 0.5 * dt * k2, command)
    k4 = dynamics(state + dt * k3, command)
    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def simulate_greybox(data: Dict[str, np.ndarray], p: Dict[str, float], v0: float) -> np.ndarray:
    t = data["t"] - data["t"][0]
    state = np.array([v0, data["yaw"][0], data["x"][0], data["y"][0],
                      data["speed_cmd"][0], data["steering_cmd"][0]], dtype=float)
    out = np.zeros((len(t), 6), dtype=float)
    out[0] = state

    def f(x: np.ndarray, u: Tuple[float, float]) -> np.ndarray:
        v, psi, xg, yg, uva, da = x
        speed_cmd, steer_cmd = u
        beta = p["k_b1"] * da + p["k_b3"] * da ** 3
        return np.array([
            (uva - v) / max(p["tau_v"], 1e-9),
            (v / p["L_eff"]) * math.tan(p["k_del"] * da),
            v * math.cos(psi + p["psi_bias"] + beta),
            v * math.sin(psi + p["psi_bias"] + beta),
            (speed_cmd - uva) / max(p["tau_acc"], 1e-9),
            (steer_cmd - da) / max(p["tau_str"], 1e-9),
        ], dtype=float)

    for k in range(len(t) - 1):
        dt = max(float(t[k + 1] - t[k]), 1e-6)
        command = (float(data["speed_cmd"][k]), float(data["steering_cmd"][k]))
        state = rk4_step(state, command, dt, f)
        out[k + 1] = state
    return out


def simulate_nominal(data: Dict[str, np.ndarray], wheelbase: float, tau_v: float, v0: float) -> np.ndarray:
    t = data["t"] - data["t"][0]
    state = np.array([v0, data["yaw"][0], data["x"][0], data["y"][0]], dtype=float)
    out = np.zeros((len(t), 4), dtype=float)
    out[0] = state

    def f(x: np.ndarray, u: Tuple[float, float]) -> np.ndarray:
        v, psi, _xg, _yg = x
        speed_cmd, steer_cmd = u
        return np.array([
            (speed_cmd - v) / max(tau_v, 1e-9),
            (v / wheelbase) * math.tan(steer_cmd),
            v * math.cos(psi),
            v * math.sin(psi),
        ], dtype=float)

    for k in range(len(t) - 1):
        dt = max(float(t[k + 1] - t[k]), 1e-6)
        command = (float(data["speed_cmd"][k]), float(data["steering_cmd"][k]))
        state = rk4_step(state, command, dt, f)
        out[k + 1] = state
    return out


def rmse(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.sqrt(np.mean((np.asarray(a) - np.asarray(b)) ** 2)))


def metrics(data: Dict[str, np.ndarray], state: np.ndarray, greybox: bool, p: Dict[str, float]) -> Dict[str, float]:
    if greybox:
        v, psi, xg, yg, _uva, da = state.T
        beta = p["k_b1"] * da + p["k_b3"] * da ** 3
        vx_world = v * np.cos(psi + p["psi_bias"] + beta)
    else:
        v, psi, xg, yg = state.T
        vx_world = v * np.cos(psi)
    yaw_error = np.unwrap(psi) - np.unwrap(data["yaw"])
    return {
        "vx_world_rmse_mps": rmse(vx_world, data["v_measured"]),
        "yaw_rmse_rad": float(np.sqrt(np.mean(yaw_error ** 2))),
        "x_rmse_m": rmse(xg, data["x"]),
        "y_rmse_m": rmse(yg, data["y"]),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--greybox-config", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--wheelbase", type=float, default=1.75)
    parser.add_argument("--nominal-tau-v", type=float, default=0.5)
    args = parser.parse_args()

    data = read_csv(args.csv)
    order = np.argsort(data["t"])
    for key in data:
        data[key] = data[key][order]
    data["yaw"] = np.unwrap(data["yaw"])

    # Estimate initial body-frame speed from position derivatives. This avoids
    # assuming that v_measured is already expressed in the body frame.
    vx = np.gradient(data["x"], data["t"])
    vy = np.gradient(data["y"], data["t"])
    v_body = vx * np.cos(data["yaw"]) + vy * np.sin(data["yaw"])
    v0 = float(v_body[0])

    p = load_yaml(args.greybox_config)
    grey = simulate_greybox(data, p, v0)
    nominal = simulate_nominal(data, args.wheelbase, args.nominal_tau_v, v0)

    result = {
        "validation_csv": str(args.csv.resolve()),
        "greybox": metrics(data, grey, True, p),
        "nominal_kinematic": metrics(data, nominal, False, p),
    }
    args.out.mkdir(parents=True, exist_ok=True)
    with (args.out / "open_loop_metrics.json").open("w") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)

    t = data["t"] - data["t"][0]
    plt.figure(figsize=(7, 7))
    plt.plot(data["x"], data["y"], label="measured")
    plt.plot(nominal[:, 2], nominal[:, 3], label="nominal bicycle")
    plt.plot(grey[:, 2], grey[:, 3], label="grey-box")
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(args.out / "open_loop_trajectory.png", dpi=180)
    plt.close()

    plt.figure(figsize=(9, 4))
    plt.plot(t, data["yaw"], label="measured")
    plt.plot(t, nominal[:, 1], label="nominal bicycle")
    plt.plot(t, grey[:, 1], label="grey-box")
    plt.xlabel("Time [s]")
    plt.ylabel("Yaw [rad]")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(args.out / "open_loop_yaw.png", dpi=180)
    plt.close()

    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
