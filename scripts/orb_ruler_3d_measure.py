#!/usr/bin/env python3
import math
import numpy as np
import rospy
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped

def quat_to_R(qx, qy, qz, qw):
    # (x,y,z,w) quaternion -> 3x3 rotation matrix
    x, y, z, w = qx, qy, qz, qw
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    R = np.array([
        [1 - 2*(yy+zz),     2*(xy-wz),       2*(xz+wy)],
        [2*(xy+wz),         1 - 2*(xx+zz),   2*(yz-wx)],
        [2*(xz-wy),         2*(yz+wx),       1 - 2*(xx+yy)]
    ], dtype=np.float64)
    return R

class OrbRuler3D:
    def __init__(self):
        self.bridge = CvBridge()

        # Params
        self.image_topic = rospy.get_param("~image", "/camera/image_rect_color")
        self.info_topic  = rospy.get_param("~camera_info", "/camera/camera_info")
        self.pose_topic  = rospy.get_param("~pose", "/orb_slam3/pose")
        self.cloud_topic = rospy.get_param("~cloud", "/orb_slam3/map_points")

        self.max_points = int(rospy.get_param("~max_points", 15000))
        self.pick_radius_px = float(rospy.get_param("~pick_radius_px", 12.0))

        # Intrinsics
        self.fx = self.fy = self.cx = self.cy = None

        # Latest data
        self.last_img = None
        self.last_Twc = None  # 4x4
        self.last_pts_w = None  # Nx3 (world/map frame)

        # Picks
        self.picked = []  # list of (Pw, (u,v))

        rospy.Subscriber(self.info_topic, CameraInfo, self.cb_info, queue_size=1)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.cb_pose, queue_size=1)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cb_cloud, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.cb_img, queue_size=1)

        cv2.namedWindow("orb_ruler_3d", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("orb_ruler_3d", self.on_mouse)

        rospy.loginfo("orb_ruler_3d_measure started.")
        rospy.loginfo("image=%s", self.image_topic)
        rospy.loginfo("camera_info=%s", self.info_topic)
        rospy.loginfo("pose=%s", self.pose_topic)
        rospy.loginfo("cloud=%s", self.cloud_topic)

    def cb_info(self, msg: CameraInfo):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def cb_pose(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        t = np.array([[p.x],[p.y],[p.z]], dtype=np.float64)
        Twc = np.eye(4, dtype=np.float64)
        Twc[:3,:3] = R
        Twc[:3, 3:] = t
        self.last_Twc = Twc

    def cb_cloud(self, msg: PointCloud2):
        # Read xyz points
        pts = []
        for p in pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True):
            pts.append([p[0], p[1], p[2]])
        if not pts:
            return
        pts = np.asarray(pts, dtype=np.float64)

        # Downsample if too large
        if pts.shape[0] > self.max_points:
            idx = np.random.choice(pts.shape[0], self.max_points, replace=False)
            pts = pts[idx]

        self.last_pts_w = pts

    def cb_img(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if len(img.shape) == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        self.last_img = img

        vis = img.copy()
        for i, (Pw, uv) in enumerate(self.picked):
            u,v = uv
            cv2.circle(vis, (u,v), 6, (0,255,0), -1)
            cv2.putText(vis, f"P{i+1}", (u+8, v-8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        if len(self.picked) == 2:
            P1 = self.picked[0][0]
            P2 = self.picked[1][0]
            d = float(np.linalg.norm(P2 - P1))
            txt = f"ORB 3D distance = {d:.4f} (ORB units) | scale = (true_m / {d:.4f})"
            cv2.putText(vis, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,255), 2)

        cv2.putText(vis, "Left click: pick | Right click: reset | ESC: quit",
                    (10, vis.shape[0]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        cv2.imshow("orb_ruler_3d", vis)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            rospy.signal_shutdown("user quit")

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_RBUTTONDOWN:
            self.picked = []
            return

        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.last_img is None or self.last_Twc is None or self.last_pts_w is None:
            rospy.logwarn("No data yet (need image+pose+cloud).")
            return
        if None in (self.fx, self.fy, self.cx, self.cy):
            rospy.logwarn("No camera intrinsics yet.")
            return

        # Twc -> Tcw (inverse)
        Twc = self.last_Twc
        Rwc = Twc[:3,:3]
        twc = Twc[:3,3]
        Rcw = Rwc.T
        tcw = -Rcw @ twc

        pts_w = self.last_pts_w
        # world -> camera
        pts_c = (Rcw @ pts_w.T).T + tcw.reshape(1,3)

        Z = pts_c[:,2]
        valid = Z > 0.05
        pts_c = pts_c[valid]
        pts_wv = pts_w[valid]
        if pts_c.shape[0] == 0:
            rospy.logwarn("No valid points in front of camera.")
            return

        u = (self.fx * (pts_c[:,0] / pts_c[:,2]) + self.cx)
        v = (self.fy * (pts_c[:,1] / pts_c[:,2]) + self.cy)

        du = u - float(x)
        dv = v - float(y)
        dist2 = du*du + dv*dv

        min_i = int(np.argmin(dist2))
        min_d = math.sqrt(float(dist2[min_i]))

        if min_d > self.pick_radius_px:
            rospy.logwarn("No close map point (min %.1f px). Try clicking on a very 'featurey' endpoint.", min_d)
            return

        Pw = pts_wv[min_i]
        rospy.loginfo("Picked point: Pw=[%.4f %.4f %.4f], pixel=(%d,%d), reproj_err=%.1fpx",
                      Pw[0], Pw[1], Pw[2], x, y, min_d)

        if len(self.picked) >= 2:
            self.picked = []
        self.picked.append((Pw, (int(x), int(y))))

        if len(self.picked) == 2:
            P1 = self.picked[0][0]
            P2 = self.picked[1][0]
            d = float(np.linalg.norm(P2 - P1))
            rospy.loginfo("ORB 3D distance = %.6f (ORB units). Compute scale = true_m / %.6f", d, d)

def main():
    rospy.init_node("orb_ruler_3d_measure")
    OrbRuler3D()
    rospy.spin()

if __name__ == "__main__":
    main()
