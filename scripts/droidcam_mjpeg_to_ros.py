#!/usr/bin/env python3
import time
import urllib.request

import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from cv_bridge import CvBridge

try:
    import yaml
    HAVE_YAML = True
except Exception:
    HAVE_YAML = False


def open_stream(url: str):
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    return urllib.request.urlopen(req, timeout=10)


def caminfo_to_yaml_dict(ci: CameraInfo, camera_name: str = "droidcam"):
    return {
        "image_width": int(ci.width),
        "image_height": int(ci.height),
        "camera_name": camera_name,
        "camera_matrix": {"rows": 3, "cols": 3, "data": list(ci.K)},
        "distortion_model": ci.distortion_model if ci.distortion_model else "plumb_bob",
        "distortion_coefficients": {"rows": 1, "cols": len(ci.D), "data": list(ci.D)},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": list(ci.R)},
        "projection_matrix": {"rows": 3, "cols": 4, "data": list(ci.P)},
    }


def main():
    rospy.init_node("droidcam_mjpeg_to_ros")

    url = rospy.get_param("~url", "http://192.168.3.87:4747/video")
    frame_id = rospy.get_param("~frame_id", "camera")
    encoding = rospy.get_param("~encoding", "bgr8")

    publish_camera_info = rospy.get_param("~publish_camera_info", True)
    camera_name = rospy.get_param("~camera_name", "droidcam")
    save_camera_info = rospy.get_param("~save_camera_info", True)
    camera_info_file = rospy.get_param("~camera_info_file", "/tmp/droidcam_camera_info.yaml")

    max_fps = float(rospy.get_param("~max_fps", 30.0))
    min_period = 1.0 / max_fps if max_fps > 0 else 0.0
    last_pub_t = 0.0

    pub_img = rospy.Publisher("image_raw", Image, queue_size=1)
    pub_info = rospy.Publisher("camera_info", CameraInfo, queue_size=1, latch=True) if publish_camera_info else None
    bridge = CvBridge()

    cam_info = CameraInfo()
    cam_info.header.frame_id = frame_id

    def set_camera_info_cb(req):
        nonlocal cam_info
        cam_info = req.camera_info
        cam_info.header.frame_id = frame_id

        rospy.loginfo("[droidcam_mjpeg_to_ros] set_camera_info: %dx%d model=%s",
                      cam_info.width, cam_info.height, cam_info.distortion_model)

        ok = True
        msg = "ok"
        if save_camera_info and HAVE_YAML:
            try:
                data = caminfo_to_yaml_dict(cam_info, camera_name=camera_name)
                with open(camera_info_file, "w") as f:
                    yaml.safe_dump(data, f, sort_keys=False)
                rospy.loginfo("[droidcam_mjpeg_to_ros] Saved camera_info to: %s", camera_info_file)
            except Exception as e:
                ok = False
                msg = str(e)
                rospy.logwarn("[droidcam_mjpeg_to_ros] Save camera_info failed: %s", e)

        return SetCameraInfoResponse(success=ok, status_message=msg)

    rospy.Service("set_camera_info", SetCameraInfo, set_camera_info_cb)

    rospy.loginfo("[droidcam_mjpeg_to_ros] Connecting to: %s", url)
    rospy.loginfo("[droidcam_mjpeg_to_ros] Service: %s", rospy.resolve_name("set_camera_info"))
    if publish_camera_info:
        rospy.loginfo("[droidcam_mjpeg_to_ros] Publishing: %s and %s", pub_img.resolved_name, pub_info.resolved_name)

    buf = b""
    stream = None

    while not rospy.is_shutdown():
        try:
            if stream is None:
                stream = open_stream(url)
                rospy.loginfo("[droidcam_mjpeg_to_ros] Stream opened.")

            chunk = stream.read(4096)
            if not chunk:
                raise RuntimeError("empty chunk")
            buf += chunk

            # JPEG boundaries
            a = buf.find(b"\xff\xd8")
            b = buf.find(b"\xff\xd9")
            if a != -1 and b != -1 and b > a:
                jpg = buf[a:b + 2]
                buf = buf[b + 2:]

                now = time.time()
                if min_period > 0 and (now - last_pub_t) < min_period:
                    continue
                last_pub_t = now

                arr = np.frombuffer(jpg, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                h, w = frame.shape[:2]
                if cam_info.width == 0 or cam_info.height == 0:
                    cam_info.width = w
                    cam_info.height = h
                    cam_info.distortion_model = "plumb_bob"
                    cam_info.K = [600, 0, w / 2.0,
                                  0, 600, h / 2.0,
                                  0, 0, 1]
                    cam_info.D = [0, 0, 0, 0, 0]
                    cam_info.R = [1, 0, 0,
                                  0, 1, 0,
                                  0, 0, 1]
                    cam_info.P = [cam_info.K[0], 0, cam_info.K[2], 0,
                                  0, cam_info.K[4], cam_info.K[5], 0,
                                  0, 0, 1, 0]

                stamp = rospy.Time.now()
                img_msg = bridge.cv2_to_imgmsg(frame, encoding=encoding)
                img_msg.header.stamp = stamp
                img_msg.header.frame_id = frame_id
                pub_img.publish(img_msg)

                if publish_camera_info and pub_info is not None:
                    cam_info.header.stamp = stamp
                    cam_info.header.frame_id = frame_id
                    pub_info.publish(cam_info)

        except Exception as e:
            rospy.logwarn("[droidcam_mjpeg_to_ros] Error: %s", e)
            try:
                if stream is not None:
                    stream.close()
            except Exception:
                pass
            stream = None
            buf = b""
            time.sleep(1.0)


if __name__ == "__main__":
    main()
