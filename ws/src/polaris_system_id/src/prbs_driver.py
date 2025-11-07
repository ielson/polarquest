#!/usr/bin/env python3
import rospy, numpy as np, random, math
from ackermann_msgs.msg import AckermannDrive

def clamp(x,a,b): return max(a,min(b,x))

# this function will evolve to a prbs (if I have time)
def prbs_value(ampl):
    return random.gauss(0, ampl/2)

def main():
    rospy.init_node("driver")
    pub = rospy.Publisher("/gem/ackermann_cmd", AckermannDrive, queue_size=1)
    rate_hz = rospy.get_param("~rate_hz", 50)
    mode    = rospy.get_param("~mode", "speed_steer")   # speed_only or speed_steer
    A_delta = rospy.get_param("~A_delta", 0.10)   # rad
    A_speed   = rospy.get_param("~A_speed",   5.00)   # m/s^2
    dwell = 500

    Ts  = 1.0/float(rate_hz)
    k   = 0
    delta_cmd = 0.0
    speed_cmd   = 0.0

    # initial seeds
    delta_cmd = prbs_value(A_delta) if mode == "speed_steer" else 0.0
    speed_cmd   = prbs_value(A_speed)   if mode in ("speed_only","speed_steer") else 0.0

    msg = AckermannDrive()
    r   = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        # change values at each dwell
        if k % dwell == 0:
            if mode == "speed_steer":
                delta_cmd = prbs_value(A_delta)
            if mode in ("speed_only","speed_steer"):
                speed_cmd   = prbs_value(A_speed)
            t  = rospy.Time.now().to_sec()
            rospy.loginfo_throttle(0.2, "t=%.2fs | speed=%.3f m/s | delta=%.3f rad | mode=%s", t, speed_cmd, delta_cmd, mode)

        # safe clamps 
        delta_cmd = clamp(delta_cmd, -10.0, 10.0)
        speed_cmd   = clamp(speed_cmd,  -20.0, 20.0)

        # from tests sending ackermannDrive msgs to /gem/ackermann_cmd we can see that 
        # the only iputs the vehicle reacts to are acceleration and steering_angle
        msg.steering_angle = float(delta_cmd)
        msg.speed = speed_cmd

        pub.publish(msg)
        k += 1
        r.sleep()

if __name__ == "__main__":
    main()
