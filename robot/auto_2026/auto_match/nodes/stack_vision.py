#!/usr/bin/env python3

import math
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image


class StackVision:
    """Find a stable top-plane center from color-aligned D435 depth frames."""

    def __init__(self, config):
        self.config = config or {}
        self.enabled = bool(self.config.get("enabled", False))
        self.depth_topic = str(rospy.get_param(
            "~stack_depth_topic",
            self.config.get(
                "depth_topic", "/camera/aligned_depth_to_color/image_raw")))
        self.camera_info_topic = str(rospy.get_param(
            "~stack_camera_info_topic",
            self.config.get(
                "camera_info_topic", "/camera/color/camera_info")))
        self.bridge = CvBridge()

    @staticmethod
    def _failure(reason, **details):
        result = {"ok": False, "reason": str(reason)}
        result.update(details)
        return result

    @staticmethod
    def depth_to_millimetres(depth_image, encoding):
        depth = np.asarray(depth_image, dtype=np.float32)
        if str(encoding).upper() == "32FC1":
            depth = depth * 1000.0
        return depth

    @staticmethod
    def _validated_roi(roi, image_shape):
        if not isinstance(roi, dict) or len(image_shape) < 2:
            return None
        height, width = image_shape[:2]
        try:
            x_min = max(0, int(roi["x_min"]))
            x_max = min(width, int(roi["x_max"]))
            y_min = max(0, int(roi["y_min"]))
            y_max = min(height, int(roi["y_max"]))
        except (KeyError, TypeError, ValueError):
            return None
        if x_max <= x_min or y_max <= y_min:
            return None
        return x_min, x_max, y_min, y_max

    @classmethod
    def extract_top_observation(cls, depth_mm, roi, config):
        roi_bounds = cls._validated_roi(roi, depth_mm.shape)
        if roi_bounds is None:
            return cls._failure("invalid_roi")
        x_min, x_max, y_min, y_max = roi_bounds
        crop = depth_mm[y_min:y_max, x_min:x_max]

        min_depth = float(config.get("min_depth_mm", 150.0))
        max_depth = float(config.get("max_depth_mm", 2000.0))
        valid = np.isfinite(crop) & (crop >= min_depth) & (crop <= max_depth)
        valid_count = int(np.count_nonzero(valid))
        min_valid_pixels = max(1, int(config.get("min_valid_pixels", 200)))
        if valid_count < min_valid_pixels:
            return cls._failure(
                "insufficient_valid_depth", valid_pixels=valid_count)

        percentile = float(config.get("foreground_percentile", 5.0))
        percentile = min(50.0, max(1.0, percentile))
        near_depth = float(np.percentile(crop[valid], percentile))
        plane_band = max(1.0, float(config.get("plane_band_mm", 20.0)))
        foreground = valid & (crop <= near_depth + plane_band)

        kernel_size = max(1, int(config.get("morphology_kernel", 3)))
        if kernel_size % 2 == 0:
            kernel_size += 1
        if kernel_size > 1:
            kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
            foreground = cv2.morphologyEx(
                foreground.astype(np.uint8), cv2.MORPH_OPEN, kernel)
            foreground = cv2.morphologyEx(
                foreground, cv2.MORPH_CLOSE, kernel)
        else:
            foreground = foreground.astype(np.uint8)

        component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
            foreground, connectivity=8)
        min_area = max(1, int(config.get("min_component_area", 100)))
        candidates = []
        for label in range(1, component_count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < min_area:
                continue
            ys, xs = np.nonzero(labels == label)
            if len(xs) == 0:
                continue
            component_depth = crop[ys, xs]
            candidates.append({
                "area": area,
                "u": float(x_min + np.median(xs)),
                "v": float(y_min + np.median(ys)),
                "depth_mm": float(np.median(component_depth)),
            })

        if not candidates:
            return cls._failure(
                "no_top_plane_component",
                valid_pixels=valid_count,
                near_depth_mm=near_depth)

        best = max(candidates, key=lambda item: item["area"])
        best.update({
            "ok": True,
            "reason": "ok",
            "valid_pixels": valid_count,
            "near_depth_mm": near_depth,
        })
        return best

    @classmethod
    def stable_observation(cls, observations, config):
        valid = [item for item in observations if item.get("ok")]
        min_stable = max(1, int(config.get("min_stable_frames", 5)))
        if len(valid) < min_stable:
            return cls._failure(
                "insufficient_observations",
                valid_frames=len(valid),
                required_frames=min_stable)

        median_u = float(np.median([item["u"] for item in valid]))
        median_v = float(np.median([item["v"] for item in valid]))
        median_depth = float(np.median([item["depth_mm"] for item in valid]))
        center_tolerance = max(
            0.0, float(config.get("center_tolerance_px", 8.0)))
        depth_tolerance = max(
            0.0, float(config.get("depth_tolerance_mm", 15.0)))

        stable = [
            item for item in valid
            if (
                math.hypot(item["u"] - median_u, item["v"] - median_v)
                <= center_tolerance and
                abs(item["depth_mm"] - median_depth) <= depth_tolerance
            )
        ]
        if len(stable) < min_stable:
            return cls._failure(
                "unstable_top_plane",
                valid_frames=len(valid),
                stable_frames=len(stable),
                required_frames=min_stable)

        return {
            "ok": True,
            "reason": "ok",
            "u": float(np.median([item["u"] for item in stable])),
            "v": float(np.median([item["v"] for item in stable])),
            "depth_mm": float(np.median([
                item["depth_mm"] for item in stable])),
            "area": float(np.median([item["area"] for item in stable])),
            "valid_frames": len(valid),
            "stable_frames": len(stable),
        }

    @classmethod
    def apply_local_calibration(cls, observation, calibration, config):
        try:
            reference = np.asarray(
                calibration["reference_pixel"], dtype=np.float64)
            matrix = np.asarray(
                calibration["pixel_to_arm_mm"], dtype=np.float64)
        except (KeyError, TypeError, ValueError):
            return cls._failure("invalid_local_calibration")
        if reference.shape != (2,) or matrix.shape != (2, 2):
            return cls._failure("invalid_local_calibration")
        if (
                not np.all(np.isfinite(reference)) or
                not np.all(np.isfinite(matrix)) or
                np.linalg.norm(matrix) < 1e-9):
            return cls._failure("invalid_local_calibration")

        pixel_delta = np.asarray([
            observation["u"] - reference[0],
            observation["v"] - reference[1],
        ])
        correction = matrix.dot(pixel_delta)
        correction_x = float(correction[0])
        correction_y = float(correction[1])
        max_correction = max(
            0.0, float(config.get("max_correction_mm", 40.0)))
        if max(abs(correction_x), abs(correction_y)) > max_correction:
            return cls._failure(
                "correction_out_of_range",
                correction_x=correction_x,
                correction_y=correction_y,
                max_correction_mm=max_correction)

        result = dict(observation)
        result.update({
            "ok": True,
            "reason": "ok",
            "correction_x": correction_x,
            "correction_y": correction_y,
        })
        return result

    def locate_main_stack(self, collection, should_stop=None):
        if not self.enabled:
            return self._failure("stack_vision_disabled")
        calibrations = self.config.get("collections", {})
        calibration = calibrations.get(collection, {})
        if not bool(calibration.get("calibrated", False)):
            return self._failure("collection_not_calibrated")

        frame_count = max(1, int(self.config.get("frame_count", 7)))
        frame_timeout = max(
            0.05, float(self.config.get("frame_timeout_sec", 1.0)))
        observation_timeout = max(
            frame_timeout,
            float(self.config.get("observation_timeout_sec", 10.0)))
        deadline = time.monotonic() + observation_timeout

        try:
            camera_info = rospy.wait_for_message(
                self.camera_info_topic,
                CameraInfo,
                timeout=min(frame_timeout, observation_timeout))
        except Exception:
            return self._failure("camera_info_timeout")
        if self._validated_roi(
                calibration.get("roi"),
                (camera_info.height, camera_info.width)) is None:
            return self._failure("invalid_roi")

        observations = []
        last_stamp = None
        max_frame_age = max(
            0.0, float(self.config.get("max_frame_age_sec", 0.5)))
        while len(observations) < frame_count and time.monotonic() < deadline:
            if should_stop is not None and should_stop():
                return self._failure("stopped")
            remaining = deadline - time.monotonic()
            try:
                message = rospy.wait_for_message(
                    self.depth_topic,
                    Image,
                    timeout=min(frame_timeout, max(0.05, remaining)))
            except Exception:
                observations.append(self._failure("depth_frame_timeout"))
                continue
            stamp = message.header.stamp
            if not stamp.is_zero():
                if last_stamp is not None and stamp <= last_stamp:
                    observations.append(self._failure("duplicate_depth_frame"))
                    continue
                last_stamp = stamp
                if (
                        max_frame_age > 0.0 and
                        (rospy.Time.now() - stamp).to_sec() > max_frame_age):
                    observations.append(self._failure("stale_depth_frame"))
                    continue
            try:
                image = self.bridge.imgmsg_to_cv2(
                    message, desired_encoding="passthrough")
            except CvBridgeError:
                observations.append(self._failure("depth_decode_failed"))
                continue
            depth_mm = self.depth_to_millimetres(image, message.encoding)
            if depth_mm.shape[:2] != (camera_info.height, camera_info.width):
                observations.append(self._failure("depth_size_mismatch"))
                continue
            observations.append(self.extract_top_observation(
                depth_mm, calibration.get("roi"), self.config))

        stable = self.stable_observation(observations, self.config)
        if not stable.get("ok"):
            return stable
        calibrated = self.apply_local_calibration(
            stable, calibration, self.config)
        calibrated["confidence"] = (
            float(stable.get("stable_frames", 0)) / float(frame_count))
        return calibrated
