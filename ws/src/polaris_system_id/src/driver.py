#!/usr/bin/env python3
import rospy, random
from ackermann_msgs.msg import AckermannDrive

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def main():
    rospy.init_node("sys_id_driver")
    pub = rospy.Publisher("/gem/ackermann_cmd", AckermannDrive, queue_size=1)

    # === parâmetros ===
    rate_hz     = rospy.get_param("~rate_hz", 30)          
    mode        = rospy.get_param("~mode", "speed_step")   # "speed_step" or "steer_step"
    pre_time    = float(rospy.get_param("~pre_time", 5.0))  # how long each segment takes
    step_time   = float(rospy.get_param("~step_time", 15.0)) 
    speed_step  = float(rospy.get_param("~speed_step", 4.0)) # m/s
    delta_step  = float(rospy.get_param("~delta_step", 0.10)) # first rotation done at steer step
    fixed_speed = float(rospy.get_param("~fixed_speed", 4.0))

    segments    = int(rospy.get_param("~segments", 60))     # number of segments after the first step
    seed        = int(rospy.get_param("~seed", 42))
    min_speed   = float(rospy.get_param("~min_speed", 0.5)) 
    speed_max_p = float(rospy.get_param("~speed_max", 5.0)) 
    steer_max_p = float(rospy.get_param("~steer_max", 0.10)) # absolute max value (radians)

    SPEED_MAX  = 5.0
    STEER_MAX  = 0.17
    speed_max  = min(speed_max_p, SPEED_MAX)
    steer_max  = min(abs(steer_max_p), STEER_MAX)

    random.seed(seed)

    msg = AckermannDrive()
    rate = rospy.Rate(rate_hz)
    Ts = 1.0 / rate_hz

    if mode == "steer_step":
        total_segments = 2 + max(0, segments)
        total_time     = total_segments * pre_time
        rospy.loginfo("Step test started: mode=%s, rate=%.1f Hz, segments=%d, total=%.1fs",
                      mode, rate_hz, total_segments, total_time)

        t = 0.0
        last_seg = -1
        while not rospy.is_shutdown() and t < total_time:
            seg = int(t // pre_time) 

            if seg != last_seg:
                if seg == 0:
                    # always starts sending a speed with 0 heading
                    msg.speed = clamp(fixed_speed, min_speed, speed_max)
                    msg.steering_angle = 0.0
                    rospy.loginfo("[seg=%d] PRE  | speed=%.2f | delta=%.3f",
                                  seg, msg.speed, msg.steering_angle)
                elif seg == 1:
                    # direction jstep, maintaining fixed speed
                    msg.speed = clamp(fixed_speed, min_speed, speed_max)
                    msg.steering_angle = clamp(delta_step, -steer_max, steer_max)
                    rospy.loginfo("[seg=%d] STEP | speed=%.2f | delta=%.3f",
                                  seg, msg.speed, msg.steering_angle)
                else:
                    new_speed = random.uniform(min_speed, speed_max)
                    new_steer = random.uniform(-steer_max, steer_max)
                    msg.speed = clamp(new_speed, min_speed, speed_max)
                    msg.steering_angle = clamp(new_steer, -steer_max, steer_max)
                    rospy.loginfo("[seg=%d] VAR  | speed=%.2f | delta=%.3f",
                                  seg, msg.speed, msg.steering_angle)
                last_seg = seg

            pub.publish(msg)

            if int(t * rate_hz) % rate_hz == 0:
                rospy.loginfo("t=%.1fs | speed=%.2f | delta=%.3f", t, msg.speed, msg.steering_angle)

            t += Ts
            rate.sleep()

    else:
        total_time = pre_time + step_time
        rospy.loginfo("Step test started: mode=%s, rate=%.1f Hz, total=%.1fs",
                      mode, rate_hz, total_time)
        t = 0.0
        while not rospy.is_shutdown() and t < total_time:
            if t < pre_time:
                msg.speed = 0.0
                msg.steering_angle = 0.0
            else:
                msg.speed = speed_step
                msg.steering_angle = 0.0

            pub.publish(msg)
            if int(t * rate_hz) % rate_hz == 0:
                rospy.loginfo("t=%.1fs | speed=%.2f | delta=%.3f", t, msg.speed, msg.steering_angle)

            t += Ts
            rate.sleep()

    msg.speed = 0.0
    msg.steering_angle = 0.0
    pub.publish(msg)
    rospy.loginfo("Finished sending messages for sysId")

if __name__ == "__main__":
    main()
