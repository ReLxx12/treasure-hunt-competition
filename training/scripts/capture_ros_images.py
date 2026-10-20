#!/usr/bin/env python3
import os
import time
from pathlib import Path

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class RosImageCapture:
    def __init__(self):
        self.out_dir = Path(os.path.expanduser(rospy.get_param("~out_dir", "~/yolo_capture/raw")))
        self.prefix = rospy.get_param("~prefix", "frame")
        self.interval = float(rospy.get_param("~interval", 0.5))
        self.max_images = int(rospy.get_param("~max_images", 0))
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 95))

        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.last_save_time = 0.0
        self.count = 0

        self.sub = rospy.Subscriber("image", Image, self.image_cb, queue_size=1, buff_size=2 ** 24)
        rospy.loginfo("Saving ROS images to: %s", self.out_dir)
        rospy.loginfo("prefix=%s interval=%.2fs max_images=%d", self.prefix, self.interval, self.max_images)

    def image_cb(self, msg):
        now = time.time()
        if now - self.last_save_time < self.interval:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            rospy.logerr("cv_bridge error: %s", exc)
            return

        stamp = time.strftime("%Y%m%d_%H%M%S")
        filename = "{}_{}_{:06d}.jpg".format(self.prefix, stamp, self.count)
        path = self.out_dir / filename
        ok = cv2.imwrite(str(path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            rospy.logerr("Failed to write image: %s", path)
            return

        self.last_save_time = now
        self.count += 1
        if self.count % 10 == 0:
            rospy.loginfo("Saved %d images, latest: %s", self.count, path)

        if self.max_images > 0 and self.count >= self.max_images:
            rospy.loginfo("Reached max_images=%d, shutting down.", self.max_images)
            rospy.signal_shutdown("capture complete")


def main():
    rospy.init_node("capture_ros_images")
    RosImageCapture()
    rospy.spin()


if __name__ == "__main__":
    main()

