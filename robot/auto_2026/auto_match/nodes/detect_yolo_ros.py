#!/usr/bin/env python3

import os
import sys

import cv2
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

try:
    from ultralytics import YOLO
except ImportError:
    rospy.logerr("unable to import ultralytics. Install torch and ultralytics first.")
    sys.exit(1)


class YoloDetector:
    def __init__(self):
        rospack = rospkg.RosPack()
        default_model = os.path.join(
            rospack.get_path("auto_match"), "models", "best.pt")

        self.model_path = rospy.get_param("~model_path", default_model)
        self.conf = float(rospy.get_param("~conf", 0.50))
        self.imgsz = int(rospy.get_param("~imgsz", 640))
        self.device = rospy.get_param("~device", "cpu")
        self.line_thickness = int(rospy.get_param("~line_thickness", 2))

        if not os.path.exists(self.model_path):
            rospy.logerr("YOLO model not found: %s", self.model_path)
            sys.exit(1)

        rospy.logwarn(
            "loading YOLO model: %s, conf=%.2f, imgsz=%d, device=%s",
            self.model_path,
            self.conf,
            self.imgsz,
            self.device,
        )
        self.model = YOLO(self.model_path)

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("debug_image", Image, queue_size=1)
        self.object_pub = rospy.Publisher("objects", Detection2DArray, queue_size=1)
        self.image_sub = rospy.Subscriber(
            "image", Image, self.image_cb, queue_size=1, buff_size=2**24)

    def image_cb(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("cv_bridge error: %s", e)
            return

        obj_array = Detection2DArray()
        obj_array.header = data.header

        try:
            results = self.model.predict(
                source=cv_image,
                imgsz=self.imgsz,
                conf=self.conf,
                device=self.device,
                verbose=False,
            )
        except Exception as e:
            rospy.logerr("YOLO predict failed: %s", e)
            return

        debug_image = cv_image.copy()
        if len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes
            xyxy = boxes.xyxy.cpu().numpy()
            cls = boxes.cls.cpu().numpy()
            conf = boxes.conf.cpu().numpy()

            for box, class_id, score in zip(xyxy, cls, conf):
                obj_array.detections.append(
                    self.object_predict(int(class_id), float(score), box, data.header)
                )
                self.draw_debug_box(debug_image, int(class_id), float(score), box)

        self.object_pub.publish(obj_array)

        try:
            image_out = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            image_out.header = data.header
            self.image_pub.publish(image_out)
        except CvBridgeError as e:
            rospy.logerr("cv_bridge publish error: %s", e)

    def object_predict(self, class_id, score, box, header):
        x1, y1, x2, y2 = box
        obj = Detection2D()
        obj_hypothesis = ObjectHypothesisWithPose()

        obj.header = header
        obj_hypothesis.id = int(class_id)
        obj_hypothesis.score = float(score)
        obj.results.append(obj_hypothesis)
        obj.bbox.size_x = int(x2 - x1)
        obj.bbox.size_y = int(y2 - y1)
        obj.bbox.center.x = int((x1 + x2) / 2)
        obj.bbox.center.y = int((y1 + y2) / 2)
        return obj

    def draw_debug_box(self, image, class_id, score, box):
        x1, y1, x2, y2 = [int(v) for v in box]
        name = self.model.names.get(class_id, str(class_id))
        label = "{} {:.2f}".format(name, score)
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), self.line_thickness)
        cv2.putText(
            image,
            label,
            (x1, max(15, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )


def main():
    rospy.init_node("detect_yolo_ros")
    YoloDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
