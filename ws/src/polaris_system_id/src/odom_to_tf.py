#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomToTF:
    def __init__(self):
        odom_topic = rospy.get_param("~odom_topic", "/gem/base_footprint/odom")
        self.parent_override = rospy.get_param("~parent_frame", "")  
        self.child_override  = rospy.get_param("~child_frame", "")    
        self.br = tf2_ros.TransformBroadcaster()
        rospy.Subscriber(odom_topic, Odometry, self.cb, queue_size=50)

    def cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        t.header.frame_id = self.parent_override or msg.header.frame_id      
        t.child_frame_id  = self.child_override  or msg.child_frame_id       
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = q
        self.br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("odom_to_tf")
    OdomToTF()
    rospy.spin()
