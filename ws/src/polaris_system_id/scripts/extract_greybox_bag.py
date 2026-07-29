#!/usr/bin/env python3
"""Extract synchronized command, odometry and steering joints from a ROS1 bag.

Run inside a sourced ROS Noetic environment where `rosbag` and the message
packages are installed.
"""
import argparse
import math

import numpy as np
import pandas as pd
import rosbag
from tf.transformations import euler_from_quaternion


def zoh(times, values, grid):
    idx = np.searchsorted(times, grid, side='right') - 1
    idx = np.clip(idx, 0, len(values)-1)
    return np.asarray(values)[idx]


def linear(times, values, grid, unwrap=False):
    values = np.asarray(values, dtype=float)
    if unwrap:
        values = np.unwrap(values)
    return np.interp(grid, times, values)


def equivalent_center_steering(left, right):
    left = np.asarray(left, dtype=float)
    right = np.asarray(right, dtype=float)
    result = np.zeros_like(left)
    small = (np.abs(left) < 1e-6) & (np.abs(right) < 1e-6)
    cot_sum = np.divide(1.0, np.tan(left), out=np.zeros_like(left), where=np.abs(left)>1e-6) + \
              np.divide(1.0, np.tan(right), out=np.zeros_like(right), where=np.abs(right)>1e-6)
    result[~small] = np.arctan2(2.0, cot_sum[~small])
    # atan2 may choose the opposite branch for negative turns.
    sign = np.sign(left + right)
    result = np.abs(result) * sign
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('bag')
    p.add_argument('output_csv')
    p.add_argument('--cmd-topic', default='/gem/ackermann_cmd')
    p.add_argument('--odom-topic', default='/gem/base_footprint/odom')
    p.add_argument('--joint-topic', default='/gem/joint_states')
    p.add_argument('--left-joint', default='left_steering_hinge_joint')
    p.add_argument('--right-joint', default='right_steering_hinge_joint')
    p.add_argument('--rate', type=float, default=50.0)
    args = p.parse_args()

    cmd_t=[]; speed=[]; steer=[]
    odom_t=[]; x=[]; y=[]; yaw=[]; vx=[]; vy=[]; wz=[]
    joint_t=[]; left=[]; right=[]

    with rosbag.Bag(args.bag) as bag:
        for topic, msg, stamp in bag.read_messages(topics=[args.cmd_topic,args.odom_topic,args.joint_topic]):
            ts = stamp.to_sec()
            if topic == args.cmd_topic:
                cmd_t.append(ts); speed.append(msg.speed); steer.append(msg.steering_angle)
            elif topic == args.odom_topic:
                odom_t.append(ts)
                x.append(msg.pose.pose.position.x); y.append(msg.pose.pose.position.y)
                q=msg.pose.pose.orientation
                yaw.append(euler_from_quaternion([q.x,q.y,q.z,q.w])[2])
                vx.append(msg.twist.twist.linear.x); vy.append(msg.twist.twist.linear.y)
                wz.append(msg.twist.twist.angular.z)
            elif topic == args.joint_topic:
                names=list(msg.name)
                if args.left_joint in names and args.right_joint in names:
                    joint_t.append(ts)
                    left.append(msg.position[names.index(args.left_joint)])
                    right.append(msg.position[names.index(args.right_joint)])

    if not cmd_t or not odom_t:
        raise RuntimeError('Command or odometry topic had no messages')

    start=max(min(cmd_t),min(odom_t)); stop=min(max(cmd_t),max(odom_t))
    if joint_t:
        start=max(start,min(joint_t)); stop=min(stop,max(joint_t))
    grid=np.arange(start,stop,1.0/args.rate)

    data={
        't':grid-grid[0],
        'x':linear(odom_t,x,grid), 'y':linear(odom_t,y,grid),
        'yaw':linear(odom_t,yaw,grid,unwrap=True),
        'vx_world':linear(odom_t,vx,grid), 'vy_world':linear(odom_t,vy,grid),
        'yaw_rate':linear(odom_t,wz,grid),
        'speed_cmd':zoh(cmd_t,speed,grid),
        'steering_cmd':zoh(cmd_t,steer,grid),
    }
    if joint_t:
        left_i=linear(joint_t,left,grid); right_i=linear(joint_t,right,grid)
        data['left_steering']=left_i; data['right_steering']=right_i
        data['steering_measured']=equivalent_center_steering(left_i,right_i)

    pd.DataFrame(data).to_csv(args.output_csv,index=False)
    print('Wrote',args.output_csv,'with',len(grid),'samples')


if __name__ == '__main__':
    main()
