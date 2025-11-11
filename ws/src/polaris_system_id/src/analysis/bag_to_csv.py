#!/usr/bin/env python3
import argparse
import os
import rosbag
import numpy as np
import pandas as pd
from tf.transformations import euler_from_quaternion

ODOM_TOPIC = "/gem/base_footprint/odom"
CMD_TOPIC  = "/gem/ackermann_cmd"

def main():
    ap = argparse.ArgumentParser(description="Reads a bag recorded with rosbag record -O bagname.bag /gem/ackermann_cmd /gem/base_footprint/odom and saves two csvs with its important info")
    ap.add_argument("--bag", required=True, help="Bag file relative path")
    ap.add_argument("--out", default=".", help="Desired output path for the csv files")
    args = ap.parse_args()

    bag_path = args.bag
    out_dir  = args.out
    base     = os.path.splitext(os.path.basename(bag_path))[0]

    odom_rows = []
    cmd_rows  = []

    bag = rosbag.Bag(bag_path)

    # ODOM
    for topic, msg, t in bag.read_messages(topics=[ODOM_TOPIC]):
        ts = t.to_sec()
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q  = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        odom_rows.append(dict(t=ts, x=px, y=py, yaw=yaw, v=vx, vy=vy, yaw_rate=wz))

    # CMD
    for topic, msg, t in bag.read_messages(topics=[CMD_TOPIC]):
        ts = t.to_sec()
        delta = float(msg.steering_angle)
        speed = float(msg.speed) if not np.isnan(getattr(msg, "speed", np.nan)) else 0.0
        # Just leaves fields that the robot needs
        cmd_rows.append(dict(t=ts, steering_angle=delta, speed=speed))

    bag.close()

    os.makedirs(out_dir, exist_ok=True)
    odom_csv = os.path.join(out_dir, f"{base}_odom_raw.csv")
    cmd_csv  = os.path.join(out_dir, f"{base}_cmd_raw.csv")

    if odom_rows:
        pd.DataFrame(odom_rows).to_csv(odom_csv, index=False)
        print("saved:", odom_csv)
    else:
        print(f"[WARNING] No messages in {ODOM_TOPIC}")

    if cmd_rows:
        pd.DataFrame(cmd_rows).to_csv(cmd_csv, index=False)
        print("saved:", cmd_csv)
    else:
        print(f"[WARNING] No messages in {CMD_TOPIC}")

if __name__ == "__main__":
    main()
