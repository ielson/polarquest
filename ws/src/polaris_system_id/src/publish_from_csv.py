#!/usr/bin/env python3
import rospy, math, os
import pandas as pd
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from tf.transformations import quaternion_from_euler

def compute_headings(xs, ys):
    yaws = []
    n = len(xs)
    for i in range(n):
        if i < n-1:
            dx = xs[i+1] - xs[i]
            dy = ys[i+1] - ys[i]
        else:
            dx = xs[i] - xs[i-1] if n > 1 else 1.0
            dy = ys[i] - ys[i-1] if n > 1 else 0.0
        yaws.append(math.atan2(dy, dx))
    return yaws

def load_wps(csv_path):
    df = pd.read_csv(csv_path)
    cols = {c.lower().strip(): c for c in df.columns}
    xcol = cols.get('x') or list(df.columns)[0]
    ycol = cols.get('y') or list(df.columns)[1]
    yawcol = cols.get('yaw') or cols.get('psi') or cols.get('theta') or None

    xs = df[xcol].to_numpy().astype(float)
    ys = df[ycol].to_numpy().astype(float)
    yaws = df[yawcol].to_numpy().astype(float) if yawcol else compute_headings(xs, ys)
    return xs, ys, yaws

def make_path(xs, ys, yaws, frame):
    path = Path()
    path.header.frame_id = frame
    now = rospy.Time.now()
    path.header.stamp = now
    for x, y, yaw in zip(xs, ys, yaws):
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.header.stamp = now
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
        path.poses.append(ps)
    return path

def make_pose_array(xs, ys, yaws, frame):
    pa = PoseArray()
    pa.header.frame_id = frame
    pa.header.stamp = rospy.Time.now()
    for x, y, yaw in zip(xs, ys, yaws):
        p = Pose()
        p.position.x = float(x)
        p.position.y = float(y)
        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        pa.poses.append(p)
    return pa

if __name__ == "__main__":
    rospy.init_node("publish_from_csv")

    frame    = rospy.get_param("~frame_id", "world")
    topic_path = rospy.get_param("~topic_path", "/path_xy")                 # nav_msgs/Path
    repeat   = rospy.get_param("~repeat", True)
    rate_hz  = rospy.get_param("~rate_hz", 1.0)
    csv_path = rospy.get_param("~csv", "analysis/data/wps.csv")

    # if the csv path is relative, resolves relative to this file
    if not os.path.isabs(csv_path):
        here = os.path.dirname(os.path.realpath(__file__))
        csv_path = os.path.join(here, csv_path)

    if not os.path.exists(csv_path):
        rospy.logfatal("CSV file not found at '%s'. ",
                       csv_path)
        raise SystemExit(1)

    xs, ys, yaws = load_wps(csv_path)
    rospy.loginfo("Loaded %d points from '%s' (publishing Path at %s, frame_id='%s').",
                  len(xs), csv_path, topic_path, frame)

    pub_path = rospy.Publisher(topic_path, Path, queue_size=1, latch=True)

    path_msg = make_path(xs, ys, yaws, frame)
    pub_path.publish(path_msg)

    if not repeat:
        rospy.loginfo("Feito (latch=on).")
        rospy.spin()  
    else:
        r = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            path_msg.header.stamp = rospy.Time.now()
            for ps in path_msg.poses:
                ps.header.stamp = path_msg.header.stamp
            pub_path.publish(path_msg)
            r.sleep()
