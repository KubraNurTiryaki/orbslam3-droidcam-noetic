#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import PointStamped

last = None

def cb(msg: PointStamped):
    global last
    p = msg.point
    if last is None:
        last = p
        rospy.loginfo("Point A: (%.4f, %.4f, %.4f). Now click Point B...", p.x, p.y, p.z)
        return

    dx = p.x - last.x
    dy = p.y - last.y
    dz = p.z - last.z
    d = math.sqrt(dx*dx + dy*dy + dz*dz)

    rospy.loginfo("Point B: (%.4f, %.4f, %.4f)", p.x, p.y, p.z)
    rospy.loginfo("Distance (SLAM units): %.6f", d)

    # reset for next measurement
    last = None
    rospy.loginfo("Click a new Point A to measure again.")

def main():
    rospy.init_node("measure_clicked_distance")
    rospy.Subscriber("/clicked_point", PointStamped, cb, queue_size=1)
    rospy.loginfo("Listening /clicked_point. In RViz select 'Publish Point' and click two points.")
    rospy.spin()

if __name__ == "__main__":
    main()
