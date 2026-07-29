#!/usr/bin/env python3
"""Publish repeatable Ackermann excitation sequences for Gazebo identification.

Start rosbag recording separately. This script is intended for simulation or a
controlled test area. It publishes to /gem/ackermann_cmd by default.
"""
import argparse
import math
import random

import rospy
from ackermann_msgs.msg import AckermannDrive


def publish_for(pub, rate, duration, speed, steering):
    end = rospy.Time.now() + rospy.Duration(duration)
    while not rospy.is_shutdown() and rospy.Time.now() < end:
        msg = AckermannDrive()
        msg.speed = float(speed)
        msg.steering_angle = float(steering)
        pub.publish(msg)
        rate.sleep()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', required=True,
                        choices=['speed_steps','steering_steps','multisine','prbs','validation'])
    parser.add_argument('--topic', default='/gem/ackermann_cmd')
    parser.add_argument('--rate', type=float, default=30.0)
    parser.add_argument('--speed', type=float, default=5.0)
    parser.add_argument('--seed', type=int, default=42)
    args, _ = parser.parse_known_args()

    rospy.init_node('greybox_excitation_publisher')
    pub = rospy.Publisher(args.topic, AckermannDrive, queue_size=10)
    rate = rospy.Rate(args.rate)
    rospy.sleep(1.0)

    try:
        if args.mode == 'speed_steps':
            for _ in range(3):
                for speed in [0, 2, 4, 6, 4, 2, 0]:
                    publish_for(pub, rate, 8.0, speed, 0.0)

        elif args.mode == 'steering_steps':
            publish_for(pub, rate, 10.0, args.speed, 0.0)
            sequence_deg = [0,2,0,-2,0,5,0,-5,0,8,0,-8,0,12,0,-12,0]
            for angle_deg in sequence_deg:
                publish_for(pub, rate, 6.0, args.speed, math.radians(angle_deg))

        elif args.mode == 'multisine':
            publish_for(pub, rate, 10.0, args.speed, 0.0)
            amplitude = math.radians(5.0)
            for frequency in [0.10, 0.20, 0.40, 0.70]:
                start = rospy.Time.now().to_sec()
                while not rospy.is_shutdown() and rospy.Time.now().to_sec() - start < 25.0:
                    elapsed = rospy.Time.now().to_sec() - start
                    steering = amplitude * math.sin(2.0 * math.pi * frequency * elapsed)
                    publish_for(pub, rate, 1.0 / args.rate, args.speed, steering)

        elif args.mode == 'prbs':
            rng = random.Random(args.seed)
            publish_for(pub, rate, 10.0, args.speed, 0.0)
            elapsed = 0.0
            amplitude = math.radians(6.0)
            sign = 1.0
            while elapsed < 120.0 and not rospy.is_shutdown():
                hold = rng.uniform(0.5, 1.5)
                sign *= -1.0 if rng.random() < 0.7 else 1.0
                publish_for(pub, rate, min(hold, 120.0-elapsed), args.speed, sign*amplitude)
                elapsed += hold

        elif args.mode == 'validation':
            publish_for(pub, rate, 10.0, args.speed, 0.0)
            for angle_deg in [3,-7,10,-3,7,-10,0]:
                publish_for(pub, rate, 6.0, args.speed, math.radians(angle_deg))
            amplitude = math.radians(6.0)
            start = rospy.Time.now().to_sec()
            while not rospy.is_shutdown() and rospy.Time.now().to_sec() - start < 60.0:
                elapsed = rospy.Time.now().to_sec() - start
                steering = amplitude * math.sin(2.0 * math.pi * 0.30 * elapsed)
                publish_for(pub, rate, 1.0 / args.rate, args.speed, steering)
    finally:
        publish_for(pub, rate, 2.0, 0.0, 0.0)


if __name__ == '__main__':
    main()
