#!/usr/bin/env python3
import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

try:
    from ultralytics import YOLO
except ImportError:
    rospy.logerr("Cannot import ultralytics. Install it on robot or use an ONNX/OpenVINO runtime.")
    raise


class YoloDetectorNode:
    def __init__(self):
        self.model_path = rospy.get_param("~model_path", "")
        self.conf = float(rospy.get_param("~conf", 0.4))
        self.imgsz = int(rospy.get_param("~imgsz", 512))
        self.device = rospy.get_param("~device", "cpu")
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))
        self.log_detections = bool(rospy.get_param("~log_detections", True))
        self.log_period_sec = float(rospy.get_param("~log_period_sec", 2.0))
        self.last_log_time = rospy.Time(0)
        self.logged_image_info = False

        if not self.model_path:
            raise RuntimeError("~model_path is required")

        self.bridge = CvBridge()
        self.model = YOLO(self.model_path)

        self.objects_pub = rospy.Publisher("objects", Detection2DArray, queue_size=1)
        self.debug_pub = rospy.Publisher("debug_image", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("image", Image, self.image_cb, queue_size=1, buff_size=2 ** 24)

        rospy.loginfo("YOLO detector loaded: %s", self.model_path)

    def cv2_to_bgr8_msg(self, frame, header):
        frame = np.ascontiguousarray(frame, dtype=np.uint8)
        image_msg = Image()
        image_msg.header = header
        image_msg.height = frame.shape[0]
        image_msg.width = frame.shape[1]
        image_msg.encoding = "bgr8"
        image_msg.is_bigendian = 0
        image_msg.step = frame.shape[1] * 3
        image_msg.data = frame.tobytes()
        return image_msg

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            rospy.logerr("cv_bridge error: %s", exc)
            return
        if not self.logged_image_info:
            rospy.logwarn(
                "YOLO input image: encoding=%s, width=%d, height=%d, cv_shape=%s",
                msg.encoding,
                msg.width,
                msg.height,
                frame.shape,
            )
            self.logged_image_info = True

        results = self.model.predict(
            source=frame,
            imgsz=self.imgsz,
            conf=self.conf,
            device=self.device,
            verbose=False,
        )

        det_array = Detection2DArray()
        det_array.header = msg.header

        debug_frame = frame.copy()
        if results:
            result = results[0]
            boxes = result.boxes
            names = result.names
            class_counts = {}

            for box in boxes:
                cls_id = int(box.cls[0].item())
                score = float(box.conf[0].item())
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                class_counts[cls_id] = class_counts.get(cls_id, 0) + 1

                det = Detection2D()
                det.header = msg.header
                hyp = ObjectHypothesisWithPose()
                hyp.id = cls_id
                hyp.score = score
                det.results.append(hyp)
                det.bbox.center.x = int((x1 + x2) / 2.0)
                det.bbox.center.y = int((y1 + y2) / 2.0)
                det.bbox.size_x = int(x2 - x1)
                det.bbox.size_y = int(y2 - y1)
                det_array.detections.append(det)

                if self.publish_debug:
                    label = "{} {:.2f}".format(names.get(cls_id, str(cls_id)), score)
                    cv2.rectangle(debug_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 180, 0), 2)
                    cv2.putText(debug_frame, label, (int(x1), max(20, int(y1) - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 180, 0), 2)

            now = rospy.Time.now()
            if self.log_detections and (now - self.last_log_time).to_sec() >= self.log_period_sec:
                readable = {
                    names.get(class_id, str(class_id)): count
                    for class_id, count in sorted(class_counts.items())
                }
                rospy.logwarn("YOLO detections conf>=%.2f: %s", self.conf, readable)
                self.last_log_time = now

        self.objects_pub.publish(det_array)

        if self.publish_debug:
            self.debug_pub.publish(self.cv2_to_bgr8_msg(debug_frame, msg.header))


def main(argv):
    rospy.init_node("yolo_detect_ros")
    YoloDetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main(sys.argv)
