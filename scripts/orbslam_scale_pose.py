#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

scale = 1.0
pub = None

def cb(msg: PoseStamped):
    out = PoseStamped()
    out.header = msg.header
    out.pose = msg.pose
    out.pose.position.x *= scale
    out.pose.position.y *= scale
    out.pose.position.z *= scale
    pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("orbslam_scale_pose")

    scale = float(rospy.get_param("~scale", 1.0))
    in_topic  = rospy.get_param("~in",  "/orb_slam3/camera_pose")
    out_topic = rospy.get_param("~out", "/orb_slam3/camera_pose_scaled")

    pub = rospy.Publisher(out_topic, PoseStamped, queue_size=10)
    rospy.Subscriber(in_topic, PoseStamped, cb, queue_size=10)

    rospy.loginfo("Scaling %s by %.9f -> %s", in_topic, scale, out_topic)
    rospy.spin()
