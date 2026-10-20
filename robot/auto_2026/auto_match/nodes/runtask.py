#!/usr/bin/python3

import sys
import os
import math
import yaml
import threading
import pickle
import traceback

import rospy
import rospkg
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
import tf.transformations as tf_transformations
import common.msg
import common.srv
from common.msg import MoveStraightDistanceAction, TurnBodyDegreeAction
# import common.action
import swiftpro.msg
from std_msgs.msg import String
from swiftpro.msg import position
from vision_msgs.msg import Detection2DArray

class SwiftProInterface:
    def __init__(self, motion_config=None):
        motion_config = motion_config or {}
        # 创建控制机械臂的topic发布者
        self.arm_position_pub = rospy.Publisher(
            "position_write_topic", swiftpro.msg.position, queue_size=1)   # 机械臂运动位置发布者
        self.arm_pump_pub = rospy.Publisher(
            "pump_topic", swiftpro.msg.status, queue_size=1)               # 机械臂气泵状态发布者
        self.arm_status_pub = rospy.Publisher(
            "swiftpro_status_topic", swiftpro.msg.status, queue_size=1)    # 机械臂开关状态发布者

        # The stock driver only echoes commanded positions, so motion timing is
        # estimated from the commanded Cartesian distance. Keep conservative
        # bounds and make every value field-tunable in items_config.yaml.
        self.dynamic_wait_enabled = bool(motion_config.get(
            "dynamic_wait_enabled", True))
        self.estimated_speed_mm_s = max(
            1.0, float(motion_config.get("estimated_speed_mm_s", 120.0)))
        self.motion_settle_margin_sec = max(
            0.0, float(motion_config.get("settle_margin_sec", 0.25)))
        self.min_motion_wait_sec = max(
            0.0, float(motion_config.get("min_wait_sec", 0.45)))
        self.max_motion_wait_sec = max(
            self.min_motion_wait_sec,
            float(motion_config.get("max_wait_sec", 2.4)))
        self.first_motion_wait_sec = max(
            self.min_motion_wait_sec,
            float(motion_config.get("first_move_wait_sec", 2.0)))
        self.legacy_motion_wait_sec = max(
            0.0, float(motion_config.get("legacy_wait_sec", 3.0)))
        self.last_commanded_pose = None
        self.motion_deadline = None
        self.stop_checker = None

    def set_stop_checker(self, stop_checker):
        self.stop_checker = stop_checker

    def should_stop(self):
        return self.stop_checker is not None and self.stop_checker()

    def wait_duration(self, seconds):
        seconds = max(0.0, float(seconds))
        deadline = rospy.Time.now() + rospy.Duration(seconds)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.should_stop():
                return False
            rate.sleep()
        return not rospy.is_shutdown()

    def estimated_motion_wait(self, x, y, z):
        if not self.dynamic_wait_enabled:
            return self.legacy_motion_wait_sec
        if self.last_commanded_pose is None:
            return min(self.first_motion_wait_sec, self.max_motion_wait_sec)

        last_x, last_y, last_z = self.last_commanded_pose
        distance_mm = math.sqrt(
            (float(x) - last_x) ** 2 +
            (float(y) - last_y) ** 2 +
            (float(z) - last_z) ** 2)
        wait_sec = (
            distance_mm / self.estimated_speed_mm_s +
            self.motion_settle_margin_sec)
        return min(
            self.max_motion_wait_sec,
            max(self.min_motion_wait_sec, wait_sec))

    def wait_for_motion(self):
        if self.motion_deadline is None:
            return True
        remaining = (self.motion_deadline - rospy.Time.now()).to_sec()
        if remaining <= 0.0:
            self.motion_deadline = None
            return True
        if not self.wait_duration(remaining):
            return False
        self.motion_deadline = None
        return True

    def command_pose(self, x, y, z):
        wait_sec = self.estimated_motion_wait(x, y, z)
        self.publish_pose(x, y, z)
        self.last_commanded_pose = (float(x), float(y), float(z))
        self.motion_deadline = rospy.Time.now() + rospy.Duration(wait_sec)
        rospy.loginfo(
            "arm motion estimate target=(%.1f, %.1f, %.1f) wait=%.2fs",
            x, y, z, wait_sec)
        return wait_sec


    def publish_pose(self, x, y, z):
        '''
        发布机械臂运动位置
        '''
        pos = position()
        pos.x = x
        pos.y = y
        pos.z = z
        # rospy.loginfo(f"set pose {x},{y},{z}")
        self.arm_position_pub.publish(pos)

    def set_pose(self, x, y, z):
        if not self.wait_for_motion():
            return False
        self.command_pose(x, y, z)
        return self.wait_for_motion()

    def set_pose_async(self, x, y, z):
        return self.command_pose(x, y, z)

    def set_pump(self, enable:bool):
        '''
        吸取或释放，设定机械臂气泵状态
        '''
        rospy.loginfo(f" 设定机械臂气泵状态为：{enable}")
        if enable:
            self.arm_pump_pub.publish(swiftpro.msg.status(1))
        else:
            self.arm_pump_pub.publish(swiftpro.msg.status(0))

    def set_status(self, lock:bool):
        '''
        设定机械臂开关状态
        '''
        rospy.loginfo(f"set arm status {lock}")
        if lock:
            self.arm_status_pub.publish(swiftpro.msg.status(1))
        else:
            self.arm_status_pub.publish(swiftpro.msg.status(0))



class CamAction:
    def __init__(self):
        # 获取标定文件相关信息
        rospack = rospkg.RosPack()
        package_path = os.path.join(rospack.get_path('auto_match'))          # 获取功能包路径
        items_path = os.path.join(package_path, 'config', 'items_config.yaml')  # 获取物体标签路径
        try:
            with open(items_path, "r", encoding="utf8") as f:
                items_content = yaml.load(f.read(), Loader=yaml.FullLoader)
        except Exception:
            rospy.logerr("can't not open file")
            sys.exit(1)
        if isinstance(items_content, type(None)):
            rospy.logerr("items file empty")
            sys.exit(1)

        self.config = items_content
        self.detector_type = rospy.get_param(
            "/detector_type", items_content.get("detector_type", "coco"))
        detector_profiles = items_content.get("detector_profiles", {})
        self.items = detector_profiles.get(self.detector_type, items_content["items"])
        rospy.logwarn("detector profile: %s, items: %s", self.detector_type, self.items)
        self.item_names_by_id = {int(v): k for k, v in self.items.items()}

        # 保留旧的手动抓取默认目标。
        self.search_id = self.ids_for_names([
            items_content["objects"]["objects_a"]
        ])

    def ids_for_names(self, names):
        if isinstance(names, str):
            names = [names]
        ids = []
        for name in names:
            if name not in self.items:
                rospy.logwarn("unknown item name in config: %s", name)
                continue
            ids.append(int(self.items[name]))
        return ids

    def item_name(self, item_id):
        return self.item_names_by_id.get(int(item_id), str(item_id))

    def detector(self, search_ids=None, search_names=None, timeout=5):
        '''
        获取需要抓取的物品在显示屏上的坐标位置
        @return: 需要抓取的物品列表cube_list

        cube_list[i]:代表第几个物体
        cube_list[i][0]:代表第i个物体的ID信息;               cube_list[i][1]:代表第i个物体的位置信息
        cube_list[i][1][0]:代表第i个物体的x方向上的位置;     cube_list[i][1][1]:代表第i个物体的y方向上的位置
        cube_list[i][1][2]:代表第i个物体的置信度
        cube_list[i][1][3]:检测框宽度;                       cube_list[i][1][4]:检测框高度
        '''
        cube_list = []
        obj_array = None
        if search_names is not None:
            search_ids = self.ids_for_names(search_names)
        if search_ids is None:
            search_ids = self.search_id
        search_ids = set([int(item_id) for item_id in search_ids])

        try:
            obj_array = rospy.wait_for_message(
                "/objects", Detection2DArray, timeout=timeout)
        except Exception:
            return cube_list
        
        for obj in obj_array.detections:
            if len(obj.results) == 0:
                continue
            item_id = int(obj.results[0].id)
            if item_id in search_ids:
                cube_list.append([
                    item_id,
                    [
                        obj.bbox.center.x,
                        obj.bbox.center.y,
                        obj.results[0].score,
                        obj.bbox.size_x,
                        obj.bbox.size_y,
                    ]
                ])

        return cube_list


class ArmAction:
    def __init__(self):

        self.cam = CamAction()
        self.default_grasp_profile = {
            "u_offset": 0.0,
            "v_offset": 0.0,
            "x_offset": 0.0,
            "y_offset": -30.0,
            "z": -60.0,
            "approach_z": 20.0,
            "lift_z": 120.0,
        }
        self.grasp_profiles = self.cam.config.get("grasp_profiles", {})
        full_rule = self.cam.config.get("full_rule", {})
        grasp_search = full_rule.get("grasp_search", {})
        self.target_preference = grasp_search.get("target_preference", {})
        self.item_target_preferences = grasp_search.get(
            "item_target_preferences", {})
        self.verify_after_lift = bool(grasp_search.get("verify_after_lift", False))
        self.verify_pixel_tolerance = float(grasp_search.get("verify_pixel_tolerance", 60.0))
        self.verify_timeout = float(grasp_search.get("verify_timeout", 1.0))
        self.verify_settle_sec = float(grasp_search.get("verify_settle_sec", 0.3))
        self.verify_frames = max(1, int(grasp_search.get("verify_frames", 2)))
        self.verify_iou_threshold = float(
            grasp_search.get("verify_iou_threshold", 0.25))
        self.verify_frame_interval_sec = max(
            0.0, float(grasp_search.get("verify_frame_interval_sec", 0.15)))
        placement = full_rule.get("placement", {})
        self.arm_motion_config = full_rule.get("arm_motion", {})
        self.approach_settle_sec = max(
            0.0, float(self.arm_motion_config.get("approach_settle_sec", 0.15)))
        self.contact_settle_sec = max(
            0.0, float(self.arm_motion_config.get("contact_settle_sec", 0.20)))
        self.pump_grip_wait_sec = max(
            0.0, float(self.arm_motion_config.get("pump_grip_wait_sec", 0.70)))
        self.lift_settle_sec = max(
            0.0, float(self.arm_motion_config.get("lift_settle_sec", 0.10)))
        self.drop_settle_sec = max(
            0.0, float(self.arm_motion_config.get("drop_settle_sec", 0.15)))
        self.retract_clearance_wait_sec = max(
            0.0, float(self.arm_motion_config.get(
                "retract_clearance_wait_sec", 0.70)))
        self.async_retract_enabled = bool(self.arm_motion_config.get(
            "async_retract_enabled", True))
        self.drop_x = float(placement.get("x", 300.0))
        self.drop_y = float(placement.get("y", 0.0))
        self.drop_base_z = float(placement.get("base_z", 120.0))
        self.drop_stack_step_z = float(placement.get("stack_step_z", 30.0))
        self.drop_max_z = float(placement.get("max_z", 180.0))
        self.drop_hover_margin_z = float(placement.get("hover_margin_z", 25.0))
        max_hover_z = placement.get("max_hover_z")
        self.drop_max_hover_z = (
            None if max_hover_z is None else float(max_hover_z))
        self.drop_release_wait_sec = float(placement.get("release_wait_sec", 0.8))
        self.last_grasp_failure_reason = None
        self.last_target_rejection_reasons = set()

        # 获取标定文件数据
        filename = os.environ['HOME'] + "/thefile.txt"
        with open(filename, 'r') as f:
            s = f.read()
        arr = s.split()
        self.x_kb = [float(arr[0]), float(arr[1])]
        self.y_kb = [float(arr[2]), float(arr[3])]        

        # 创建机械臂控制接口的对象
        self.interface = SwiftProInterface(self.arm_motion_config)

        self.grasp_status_pub = rospy.Publisher("/grasp_status", String, queue_size=1)

    def set_stop_checker(self, stop_checker):
        self.interface.set_stop_checker(stop_checker)

    def wait_for_motion(self):
        return self.interface.wait_for_motion()

    def get_grasp_profile(self, item_name):
        profile = self.default_grasp_profile.copy()
        profile.update(self.grasp_profiles.get("default", {}))
        profile.update(self.grasp_profiles.get(item_name, {}))
        return profile

    def swiftpro_ik_ok(self, x, y, z):
        math_trans = 57.2958
        math_l1 = 106.6
        math_l2 = 13.2
        math_lower_arm = 142.07
        math_upper_arm = 158.81
        math_upper_lower = math_upper_arm / math_lower_arm

        z = z + 74.55
        z_in = (z - math_l1) / math_lower_arm

        if x < 0.1:
            x = 0.1

        if y == 0:
            angle_rot = 90.0
        elif y < 0:
            angle_rot = -math.atan(x / y) * math_trans
        else:
            angle_rot = 180.0 - math.atan(x / y) * math_trans

        try:
            sin_rot = math.sin(angle_rot / math_trans)
            if abs(sin_rot) < 1e-6:
                return False
            x_in = (x / sin_rot - math_l2 - 56.55) / math_lower_arm
            if abs(x_in) < 1e-6:
                phi = 90.0 if z_in >= 0.0 else -90.0
            else:
                phi = math.atan(z_in / x_in) * math_trans
            sqrt_zx = math.sqrt(z_in * z_in + x_in * x_in)
            if sqrt_zx < 1e-6:
                return False

            right_all = (
                sqrt_zx * sqrt_zx + math_upper_lower * math_upper_lower - 1.0
            ) / (2.0 * math_upper_lower * sqrt_zx)
            angle_right = math.acos(right_all) * math_trans

            right_all = (
                sqrt_zx * sqrt_zx + 1.0 - math_upper_lower * math_upper_lower
            ) / (2.0 * sqrt_zx)
            angle_left = math.acos(right_all) * math_trans
            angle_left = angle_left + phi
            angle_right = angle_right - phi
        except (ValueError, ZeroDivisionError):
            return False

        return not (
            math.isnan(angle_rot) or math.isnan(angle_left) or math.isnan(angle_right)
        )

    def detection_to_arm_target(self, cube):
        item_id = cube[0]
        item_name = self.cam.item_name(item_id)
        u = cube[1][0]
        v = cube[1][1]
        score = cube[1][2]
        width = cube[1][3] if len(cube[1]) > 3 else 0.0
        height = cube[1][4] if len(cube[1]) > 4 else 0.0
        profile = self.get_grasp_profile(item_name)

        target_u = u + float(profile.get("u_offset", 0.0))
        target_v = v + float(profile.get("v_offset", 0.0))
        x = self.x_kb[0] * target_v + self.x_kb[1] + float(profile.get("x_offset", 0.0))
        y = self.y_kb[0] * target_u + self.y_kb[1] + float(profile.get("y_offset", -30.0))
        z = float(profile.get("z", -60.0))
        return {
            "item_id": item_id,
            "item_name": item_name,
            "u": u,
            "v": v,
            "target_u": target_u,
            "target_v": target_v,
            "score": score,
            "width": width,
            "height": height,
            "x": x,
            "y": y,
            "z": z,
            "approach_z": float(profile.get("approach_z", 20.0)),
            "lift_z": float(profile.get("lift_z", 120.0)),
        }

    def target_poses_reachable(self, target):
        x = target["x"]
        y = target["y"]
        z = target["z"]
        approach_z = z + target["approach_z"]
        lift_z = z + target["lift_z"]
        checks = [
            ("approach", approach_z),
            ("touch", z),
            ("lift", lift_z),
        ]
        bad_poses = []
        for pose_name, pose_z in checks:
            if not self.swiftpro_ik_ok(x, y, pose_z):
                bad_poses.append("%s_z=%.1f" % (pose_name, pose_z))
        if len(bad_poses) > 0:
            rospy.logwarn(
                "grasp target %s has unreachable poses: %s",
                target["item_name"],
                ", ".join(bad_poses))
            return False
        return True

    def target_preference_for(self, target):
        preference = self.target_preference.copy()
        item_name = target.get("item_name")
        if item_name is not None:
            preference.update(self.item_target_preferences.get(item_name, {}))
        return preference

    def target_workspace_reason(self, target):
        pref = self.target_preference_for(target)
        x = target["x"]
        y = target["y"]
        x_min = float(pref.get("x_min", 230.0))
        x_max = float(pref.get("x_max", 350.0))
        y_abs_max = float(pref.get("y_abs_max", 120.0))
        if x < x_min:
            return "x_low"
        if x > x_max:
            return "x_high"
        if abs(y) > y_abs_max:
            return "y_far"
        return None

    def target_preference_score(self, target):
        pref = self.target_preference_for(target)
        x = target["x"]
        y = target["y"]
        confidence = target["score"]
        workspace_reason = self.target_workspace_reason(target)
        if workspace_reason is not None:
            return None, workspace_reason

        x_ideal = float(pref.get("x_ideal", 300.0))
        y_ideal = float(pref.get("y_ideal", 0.0))
        x_soft_min = float(pref.get("x_soft_min", 260.0))
        x_soft_max = float(pref.get("x_soft_max", 340.0))
        x_weight = float(pref.get("x_weight", 1.0))
        y_weight = float(pref.get("y_weight", 0.6))
        edge_weight = float(pref.get("edge_weight", 2.0))
        confidence_weight = float(pref.get("confidence_weight", 100.0))

        edge_penalty = 0.0
        if x < x_soft_min:
            edge_penalty = x_soft_min - x
        elif x > x_soft_max:
            edge_penalty = x - x_soft_max

        score = (
            confidence * confidence_weight
            - abs(x - x_ideal) * x_weight
            - abs(y - y_ideal) * y_weight
            - edge_penalty * edge_weight
        )
        return score, "ok"

    def target_preference_tier(self, target):
        pref = self.target_preference_for(target)
        if not bool(pref.get("comfort_first", True)):
            return "comfort"

        x = target["x"]
        y = target["y"]
        comfort_x_min = float(pref.get("comfort_x_min", 180.0))
        comfort_x_max = float(pref.get("comfort_x_max", 330.0))
        comfort_y_abs_max = float(pref.get("comfort_y_abs_max", 100.0))
        if (
                x < comfort_x_min or
                x > comfort_x_max or
                abs(y) > comfort_y_abs_max):
            return "fallback"
        return "comfort"

    def score_grasp_target(self, target):
        workspace_reason = self.target_workspace_reason(target)
        ok = False
        score = None
        reason = workspace_reason
        tier = None
        if workspace_reason is None:
            ok = self.target_poses_reachable(target)
            reason = "ik"
            pref = self.target_preference_for(target)
            advance_x_min = float(
                pref.get(
                    "advance_x_min",
                    pref.get("comfort_x_max", 330.0)))
            if not ok and target["x"] > advance_x_min:
                reason = "x_high"
        if ok:
            score, reason = self.target_preference_score(target)
            if score is not None:
                tier = self.target_preference_tier(target)
        target["ik_ok"] = ok
        target["preference_score"] = score
        target["preference_reason"] = reason
        target["preference_tier"] = tier
        return score is not None

    def select_best_target(self, cube_list):
        best_comfort_target = None
        best_fallback_target = None
        self.last_target_rejection_reasons = set()
        for cube in cube_list:
            target = self.detection_to_arm_target(cube)
            accepted = self.score_grasp_target(target)
            rospy.logwarn(
                "grasp candidate %s score=%.2f pixel=(%.1f, %.1f) target_pixel=(%.1f, %.1f) arm=(%.1f, %.1f, %.1f) approach_z=%.1f lift_z=%.1f ik=%s preference=%s tier=%s accepted=%s comfort_score=%s",
                target["item_name"],
                target["score"],
                target["u"],
                target["v"],
                target["target_u"],
                target["target_v"],
                target["x"],
                target["y"],
                target["z"],
                target["z"] + target["approach_z"],
                target["z"] + target["lift_z"],
                target["ik_ok"],
                target["preference_reason"],
                target["preference_tier"],
                accepted,
                "%.1f" % target["preference_score"] if target["preference_score"] is not None else "None",
            )
            if not accepted:
                self.last_target_rejection_reasons.add(
                    target["preference_reason"])
                continue
            if target["preference_tier"] == "comfort":
                if (
                        best_comfort_target is None or
                        target["preference_score"] > best_comfort_target["preference_score"]):
                    best_comfort_target = target
            elif (
                    best_fallback_target is None or
                    target["preference_score"] > best_fallback_target["preference_score"]):
                best_fallback_target = target

        if best_comfort_target is not None:
            return best_comfort_target
        if best_fallback_target is not None:
            rospy.logwarn(
                "no comfort grasp target found, using fallback target %s at arm=(%.1f, %.1f, %.1f)",
                best_fallback_target["item_name"],
                best_fallback_target["x"],
                best_fallback_target["y"],
                best_fallback_target["z"])
        return best_fallback_target

    def find_best_grasp_target(self, search_ids=None, search_names=None):
        cube_list = self.cam.detector(search_ids=search_ids, search_names=search_names)
        if len(cube_list) == 0:
            rospy.logwarn("没有找到物品啊。。。去下一个地方")
            self.last_grasp_failure_reason = "no_detection"
            return None
        target = self.select_best_target(cube_list)
        if target is None:
            if self.last_target_rejection_reasons == set(["x_high"]):
                self.last_grasp_failure_reason = "target_too_far"
            else:
                self.last_grasp_failure_reason = "no_accepted_target"
        return target

    def bbox_iou(self, center_a, size_a, center_b, size_b):
        if min(size_a[0], size_a[1], size_b[0], size_b[1]) <= 0.0:
            return 0.0
        a_left = center_a[0] - size_a[0] / 2.0
        a_right = center_a[0] + size_a[0] / 2.0
        a_top = center_a[1] - size_a[1] / 2.0
        a_bottom = center_a[1] + size_a[1] / 2.0
        b_left = center_b[0] - size_b[0] / 2.0
        b_right = center_b[0] + size_b[0] / 2.0
        b_top = center_b[1] - size_b[1] / 2.0
        b_bottom = center_b[1] + size_b[1] / 2.0

        intersection_width = max(0.0, min(a_right, b_right) - max(a_left, b_left))
        intersection_height = max(0.0, min(a_bottom, b_bottom) - max(a_top, b_top))
        intersection = intersection_width * intersection_height
        union = size_a[0] * size_a[1] + size_b[0] * size_b[1] - intersection
        if union <= 0.0:
            return 0.0
        return intersection / union

    def target_still_visible_after_lift(self, target):
        if not self.verify_after_lift:
            return False
        if self.verify_settle_sec > 0.0:
            rospy.sleep(self.verify_settle_sec)

        source_u = target["u"]
        source_v = target["v"]
        source_size = (target["width"], target["height"])
        consecutive_missing_frames = 0
        for frame_index in range(self.verify_frames):
            cube_list = self.cam.detector(
                search_ids=[target["item_id"]],
                timeout=self.verify_timeout)
            source_match = False
            for cube in cube_list:
                u = cube[1][0]
                v = cube[1][1]
                width = cube[1][3] if len(cube[1]) > 3 else 0.0
                height = cube[1][4] if len(cube[1]) > 4 else 0.0
                distance = math.hypot(u - source_u, v - source_v)
                iou = self.bbox_iou(
                    (source_u, source_v),
                    source_size,
                    (u, v),
                    (width, height))
                rospy.logwarn(
                    "post-lift verification frame=%d/%d %s pixel=(%.1f, %.1f), source=(%.1f, %.1f), dist=%.1f iou=%.2f",
                    frame_index + 1,
                    self.verify_frames,
                    target["item_name"],
                    u,
                    v,
                    source_u,
                    source_v,
                    distance,
                    iou)
                if source_size[0] > 0.0 and source_size[1] > 0.0:
                    source_match = (
                        distance <= self.verify_pixel_tolerance and
                        iou >= self.verify_iou_threshold)
                else:
                    source_match = distance <= self.verify_pixel_tolerance
                if source_match:
                    break

            if source_match:
                consecutive_missing_frames = 0
            else:
                consecutive_missing_frames += 1
                rospy.logwarn(
                    "post-lift verification %s missing frame=%d/%d consecutive=%d",
                    target["item_name"],
                    frame_index + 1,
                    self.verify_frames,
                    consecutive_missing_frames)
            if consecutive_missing_frames >= self.verify_frames:
                rospy.loginfo(
                    "grasp verification passed for %s: source missing for %d consecutive frames",
                    target["item_name"],
                    consecutive_missing_frames)
                return False
            if (
                    frame_index < self.verify_frames - 1 and
                    self.verify_frame_interval_sec > 0.0):
                rospy.sleep(self.verify_frame_interval_sec)
        return True

    def execute_grasp_target(self, target):
        self.last_grasp_failure_reason = None
        x = target["x"]
        y = target["y"]
        z = target["z"]
        approach_z = target["approach_z"]
        lift_z = target["lift_z"]

        rospy.logwarn(
            "selected grasp target %s: %.1f, %.1f, %.1f comfort_score=%s",
            target["item_name"], x, y, z,
            "%.1f" % target["preference_score"] if target["preference_score"] is not None else "None")

        # 机械臂移动到目标位置上方
        if not self.interface.set_pose(x, y, z + approach_z):
            return None
        if not self.interface.wait_duration(self.approach_settle_sec):
            return None

        # 先下降到接触位置，再开气泵吸取。提前开泵容易在空中漏吸。
        if not self.interface.set_pose(x, y, z):
            return None
        if not self.interface.wait_duration(self.contact_settle_sec):
            return None

        self.interface.set_pump(True)
        if not self.interface.wait_duration(self.pump_grip_wait_sec):
            self.interface.set_pump(False)
            return None

        rospy.loginfo("lift grasped target %s to z=%.1f", target["item_name"], z + lift_z)
        if not self.interface.set_pose(x, y, z + lift_z):
            self.interface.set_pump(False)
            return None
        if not self.interface.wait_duration(self.lift_settle_sec):
            self.interface.set_pump(False)
            return None

        if self.verify_after_lift:
            rospy.loginfo("move arm aside before post-lift verification")
            if not self.arm_grasp_ready():
                self.interface.set_pump(False)
                return None

        if self.target_still_visible_after_lift(target):
            rospy.logwarn(
                "grasp verification failed for %s: target still visible at source",
                target["item_name"])
            self.interface.set_pump(False)
            self.interface.wait_duration(0.5)
            self.arm_grasp_ready()
            self.grasp_status_pub.publish(String("1"))
            self.last_grasp_failure_reason = "verify_failed"
            return None

        self.arm_carry_ready_async()
        self.last_grasp_failure_reason = None
        self.grasp_status_pub.publish(String("0"))

        return target["item_id"]
    
    def grasp(self, search_ids=None, search_names=None):
        '''
        使用深度学习找到所需物品在图像上的位置, 估算物品实际位置, 让机械臂抓取
        @return: 抓取到物品的id, None为未识别到需要的物品
        '''
        r1 = rospy.Rate(0.25)
        r2 = rospy.Rate(10)
        x = 0
        y = 0
        z = 0

        target = self.find_best_grasp_target(
            search_ids=search_ids, search_names=search_names)
        if target is None:
            rospy.logwarn("detected targets are outside preferred grasp workspace")
            self.grasp_status_pub.publish(String("1"))
            return None

        return self.execute_grasp_target(target)

    def drop_pose_for_stack(self, stack_count, x=None, y=None):
        stack_index = max(0, int(stack_count))
        release_z = min(
            self.drop_base_z + stack_index * self.drop_stack_step_z,
            self.drop_max_z)
        hover_z = release_z + self.drop_hover_margin_z
        if self.drop_max_hover_z is not None:
            hover_z = min(hover_z, self.drop_max_hover_z)
        drop_x = self.drop_x if x is None else float(x)
        drop_y = self.drop_y if y is None else float(y)
        return drop_x, drop_y, release_z, hover_z

    def drop_target_reachable(self, x, y, stack_count):
        _, _, release_z, hover_z = self.drop_pose_for_stack(
            stack_count, x=x, y=y)
        return (
            self.swiftpro_ik_ok(x, y, hover_z) and
            self.swiftpro_ik_ok(x, y, release_z))

    def drop(self, check=False, stack_count=0, x=None, y=None):
        '''
        放置方块, 可以先判断是否有方块, 从而调整放置高度
        @param check: 是否判断有无方块, 默认判断
        @param stack_count: 当前回收区已有物品数量，用于逐层提高放置高度
        @return item_id: 执行结果
        '''
        r1 = rospy.Rate(0.25)
        r2 = rospy.Rate(10)
        x, y, z, hover_z = self.drop_pose_for_stack(
            stack_count, x=x, y=y)

        if (check):
            # 控制机械臂移动到其他地方，以免挡住摄像头
            self.interface.set_pose(0, 225, 160)
            # r1.sleep()
            rospy.sleep(2)
            cube_list = self.cam.detector()
            if len(cube_list) > 0:
                x = self.x_kb[0] * cube_list[0][1][1] + self.x_kb[1]
                y = self.y_kb[0] * cube_list[0][1][0] + self.y_kb[1]
                z = self.drop_base_z
                hover_z = z + self.drop_hover_margin_z
                if self.drop_max_hover_z is not None:
                    hover_z = min(hover_z, self.drop_max_hover_z)

        rospy.loginfo(
            "stacked drop pose count=%d x=%.1f y=%.1f release_z=%.1f hover_z=%.1f",
            max(0, int(stack_count)), x, y, z, hover_z)

        if not self.interface.set_pose(x, y, hover_z):
            return False
        if not self.interface.wait_duration(self.drop_settle_sec):
            return False
        if not self.interface.set_pose(x, y, z):
            return False
        if not self.interface.wait_duration(self.drop_settle_sec):
            return False


        # 关闭气泵
        self.interface.set_pump(0)
        # r2.sleep()
        if not self.interface.wait_duration(self.drop_release_wait_sec):
            return False

        if not self.interface.set_pose(x, y, hover_z):
            return False
        if not self.interface.wait_duration(self.drop_settle_sec):
            return False

        # self.interface.set_pose(300, 0, 120)
        # r1.sleep()
        if self.async_retract_enabled:
            self.arm_grasp_ready_async()  # 收臂与返回导航并行
            if not self.interface.wait_duration(self.retract_clearance_wait_sec):
                return False
        elif not self.arm_grasp_ready():
            return False

        self.grasp_status_pub.publish(String("0"))

        return True

    def arm_position_reset(self):
        '''
        校准机械臂的坐标系, 机械臂因碰撞导致坐标计算出问题时使用
        '''
        r1 = rospy.Rate(10)
        self.interface.set_status(False)
        r1.sleep()
        self.interface.set_status(True)
        r1.sleep()

    def arm_home(self, block=False):
        '''
        收起机械臂(无物品)
        '''
        result = self.interface.set_pose(130, 0, 35)
        if block:
            result = result and self.interface.wait_for_motion()
        return result

    def arm_grasp_ready(self, block=False):
        '''
        移动机械臂到摄像头看不到的地方，以方便识别与抓取
        '''
        result = self.interface.set_pose(50, 180, 160)
        if block:
            result = result and self.interface.wait_for_motion()
        return result

    def arm_grasp_ready_async(self):
        return self.interface.set_pose_async(50, 180, 160)

    def arm_carry_ready(self, block=False):
        '''
        抓到物品后收回到车体中线高位，减少底盘转弯时扫到分拣区物品。
        '''
        result = self.interface.set_pose(120, 0, 160)
        if block:
            result = result and self.interface.wait_for_motion()
        return result

    def arm_carry_ready_async(self):
        self.interface.set_pose_async(120, 0, 160)

    def arm_grasp_laser(self, block=False):
        self.interface.set_pose(160, 0, 20)
        if block:
            rospy.sleep(3)

class RobotMoveAction:
    def __init__(self):
        self.stop_checker = None
        self.cancel_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=1)

        # 创建控制spark直走的action客户端
        self.move_action_cli = actionlib.SimpleActionClient(
            'move_straight', MoveStraightDistanceAction)
        self.move_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))

        # 创建控制spark旋转的action客户端
        self.turn_action_cli = actionlib.SimpleActionClient(
            'turn_body', TurnBodyDegreeAction)
        self.turn_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))


        # 创建获取spark前后距离的service客户端
        rospy.wait_for_service('/get_distance')
        self.distance_srv = rospy.ServiceProxy(
            'get_distance', common.srv.GetFrontBackDistance)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(20))
        self.tf_listener = tf.TransformListener()

        rospack = rospkg.RosPack()
        self.mark_map_path = os.path.join(
            rospack.get_path('auto_match'), 'config', 'mark_map.pkl')

    def set_stop_checker(self, stop_checker):
        self.stop_checker = stop_checker

    def effective_stop_checker(self, should_stop):
        return should_stop if should_stop is not None else self.stop_checker

    def _load_mark_map(self):
        with open(self.mark_map_path, 'rb') as file:
            mark_map = pickle.load(file)
        clean_map = {}
        for name, pose in mark_map.items():
            clean_map[str(name).strip().strip('"').strip("'")] = pose
        return clean_map

    def _normalized_mark_names(self):
        try:
            mark_map = self._load_mark_map()
        except Exception as e:
            rospy.logerr("failed to load mark map: %s", e)
            return set()
        return set(mark_map.keys())

    def has_mark(self, name):
        return name in self._normalized_mark_names()

    def mark_pose(self, name):
        mark_map = self._load_mark_map()
        if name not in mark_map:
            raise KeyError(name)
        mark_matrix = mark_map[name]
        tran = tf_transformations.translation_from_matrix(mark_matrix)
        quat = tf_transformations.quaternion_from_matrix(mark_matrix)
        return Pose(
            Point(tran[0], tran[1], tran[2]),
            Quaternion(quat[0], quat[1], quat[2], quat[3]))

    def pose_yaw(self, pose):
        quat = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        return tf_transformations.euler_from_quaternion(quat)[2]

    def current_yaw(self, timeout=1.0):
        try:
            self.tf_listener.waitForTransform(
                "map", "base_footprint", rospy.Time(0), rospy.Duration(timeout))
            _, rot = self.tf_listener.lookupTransform(
                "map", "base_footprint", rospy.Time(0))
            return tf_transformations.euler_from_quaternion(rot)[2]
        except Exception as e:
            rospy.logwarn("failed to read current robot yaw: %s", e)
            return None

    def shortest_angle_degrees(self, target_yaw, current_yaw):
        diff = target_yaw - current_yaw
        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi
        return math.degrees(diff)

    def cancel_all(self):
        self.move_base.cancel_all_goals()
        self.cancel_pub.publish(GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
        self.move_action_cli.cancel_goal()
        self.turn_action_cli.cancel_goal()

    def goto_local(self, name, timeout=60, should_stop=None):
        '''
        根据目标点名称，直接发送目标位置到MoveBase服务器。
        @return: True 为成功到达, False 为失败
        '''
        if not self.has_mark(name):
            rospy.logerr("navigation mark not found: %s", name)
            return False
        try:
            pose = self.mark_pose(name)
        except Exception as e:
            rospy.logerr("failed to load navigation mark %s: %s", name, e)
            return False

        return self.goto_pose(name, pose, timeout=timeout, should_stop=should_stop)

    def goto_pose(self, name, pose, timeout=60, should_stop=None):
        '''
        直接发送运行时目标位置到MoveBase服务器。
        @return: True 为成功到达, False 为失败
        '''

        should_stop = self.effective_stop_checker(should_stop)
        start_time = rospy.Time.now()
        deadline = start_time + rospy.Duration(timeout)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = start_time
        goal.target_pose.pose = pose
        self.move_base.send_goal(goal)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if should_stop is not None and should_stop():
                rospy.logwarn("navigation to %s cancelled by task guard", name)
                self.cancel_all()
                return False
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("==========Goal succeeded: %s==========", name)
                return True
            if state in [
                    GoalStatus.ABORTED,
                    GoalStatus.REJECTED,
                    GoalStatus.PREEMPTED,
                    GoalStatus.RECALLED,
                    GoalStatus.LOST]:
                rospy.logwarn("navigation to %s failed with state %s", name, state)
                return False
            if rospy.Time.now() >= deadline:
                rospy.logwarn("nav timeout: %s", name)
                self.cancel_all()
                return False
            rate.sleep()

        self.cancel_all()
        return False
    
    def run_straight(self, move_distance, velocity, timeout=None, should_stop=None):
        move_distance = abs(float(move_distance))
        direction = -1.0 if float(velocity) < 0.0 else 1.0
        speed = max(0.03, min(0.35, abs(float(velocity))))
        velocity = direction * speed
        if timeout is None:
            timeout = max(5.0, move_distance / speed + 3.0)
        should_stop = self.effective_stop_checker(should_stop)

        goal = common.msg.MoveStraightDistanceGoal(
            type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
            const_rot_vel=velocity,
            move_distance=move_distance,
        )
        self.move_action_cli.send_goal(goal)
        deadline = rospy.Time.now() + rospy.Duration(float(timeout))
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if should_stop is not None and should_stop():
                self.move_action_cli.cancel_goal()
                return False
            state = self.move_action_cli.get_state()
            if state == GoalStatus.SUCCEEDED:
                return True
            if state in [
                    GoalStatus.ABORTED,
                    GoalStatus.REJECTED,
                    GoalStatus.PREEMPTED,
                    GoalStatus.RECALLED,
                    GoalStatus.LOST]:
                rospy.logwarn("straight motion failed with state %s", state)
                return False
            if rospy.Time.now() >= deadline:
                rospy.logwarn(
                    "straight motion timeout: distance=%.2f velocity=%.2f",
                    move_distance,
                    velocity)
                self.move_action_cli.cancel_goal()
                return False
            rate.sleep()
        self.move_action_cli.cancel_goal()
        return False

    def step_back(
            self,
            move_distance=0.28,
            vel=0.1,
            timeout=None,
            should_stop=None):
        '''
        后退, 用于离开抓取点
        @return: True 为调整成功, False 为调整失败
        '''
        rospy.loginfo("step back %.2fm at %.2fm/s", move_distance, abs(vel))
        return self.run_straight(
            move_distance,
            -abs(vel),
            timeout=timeout,
            should_stop=should_stop)
    
    def step_go(self, dis, vel=0.1, timeout=None, should_stop=None):
        '''
        前进, 用于抓取或放置前使用
        @return: True 为调整成功, False 为调整失败
        '''
        rospy.loginfo("step forward %.2fm at %.2fm/s", dis, abs(vel))
        return self.run_straight(
            dis,
            abs(vel),
            timeout=timeout,
            should_stop=should_stop)

    def turn_body(self, degree, vel=0.18, timeout=8, should_stop=None):
        should_stop = self.effective_stop_checker(should_stop)
        if abs(degree) < 0.1:
            return True
        if degree > 180:
            degree -= 360
        elif degree < -180:
            degree += 360

        goal = common.msg.TurnBodyDegreeGoal(
            is_const_vel=True,
            goal_degree=degree,
            const_rot_vel=vel,
        )
        self.turn_action_cli.send_goal(goal)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if should_stop is not None and should_stop():
                self.turn_action_cli.cancel_goal()
                return False
            state = self.turn_action_cli.get_state()
            if state == GoalStatus.SUCCEEDED:
                return True
            if state in [
                    GoalStatus.ABORTED,
                    GoalStatus.REJECTED,
                    GoalStatus.PREEMPTED,
                    GoalStatus.RECALLED,
                    GoalStatus.LOST]:
                rospy.logwarn("turn_body %.1f failed with state %s", degree, state)
                return False
            if rospy.Time.now() >= deadline:
                rospy.logwarn("turn_body %.1f timeout", degree)
                self.turn_action_cli.cancel_goal()
                return False
            rate.sleep()
        self.turn_action_cli.cancel_goal()
        return False

    def refine_mark_heading(
            self,
            name,
            tolerance_deg=3.0,
            max_turn_deg=25.0,
            vel=0.16,
            should_stop=None):
        try:
            target_pose = self.mark_pose(name)
        except Exception as e:
            rospy.logwarn("heading refine skipped, failed to load mark %s: %s", name, e)
            return False

        return self.refine_pose_heading(
            name,
            target_pose,
            tolerance_deg=tolerance_deg,
            max_turn_deg=max_turn_deg,
            vel=vel,
            should_stop=should_stop)

    def refine_pose_heading(
            self,
            name,
            target_pose,
            tolerance_deg=3.0,
            max_turn_deg=25.0,
            vel=0.16,
            should_stop=None):

        current_yaw = self.current_yaw()
        if current_yaw is None:
            return False

        target_yaw = self.pose_yaw(target_pose)
        error_deg = self.shortest_angle_degrees(target_yaw, current_yaw)
        rospy.loginfo("heading error at %s: %.1f deg", name, error_deg)
        if abs(error_deg) <= tolerance_deg:
            return True
        if abs(error_deg) > max_turn_deg:
            rospy.logwarn(
                "heading refine at %s rejected: %.1f deg exceeds %.1f deg",
                name, error_deg, max_turn_deg)
            return False
        return self.turn_body(error_deg, vel=vel, should_stop=should_stop)


class AutoAction:
    STATE_INIT = "INIT"
    STATE_LEAVE_START = "LEAVE_START"
    STATE_ARM_READY = "ARM_READY"
    STATE_GO_CLASSIFICATION = "GO_CLASSIFICATION"
    STATE_READ_BOARD = "READ_BOARD"
    STATE_PICK_CAKE = "PICK_CAKE"
    STATE_DROP_CAKE_A = "DROP_CAKE_A"
    STATE_PICK_SCORING_ITEM = "PICK_SCORING_ITEM"
    STATE_DROP_SCORING_ITEM = "DROP_SCORING_ITEM"
    STATE_FINISH = "FINISH"
    STATE_ABORT = "ABORT"

    def __init__(self):
        # # 初始化节点
        # if init_node:
        rospy.init_node('spark_auto_match_node', anonymous=True)

        print("========ready to task===== ")

        # 实例化Cam
        try: self.cam = CamAction()
        except Exception as e:  print("except cam:",e)
        print("========实例化Cam===== ")
        # 实例化Arm
        try: self.arm = ArmAction()
        except Exception as e:  print("except arm:",e)
        print("========实例化Arm===== ")
        # 实例化Robot
        try: self.robot = RobotMoveAction()
        except Exception as e:  print("except robot:",e)
        print("========实例化Robot===== ")

        self.full_rule_config = self.cam.config.get("full_rule", {})
        self.placement_config = self.full_rule_config.get("placement", {})
        self.main_stack_x = float(self.placement_config.get(
            "main_x", self.placement_config.get("x", 200.0)))
        self.main_stack_y = float(self.placement_config.get(
            "main_y", self.placement_config.get("y", 0.0)))
        self.fallback_stack_x = float(self.placement_config.get(
            "fallback_x", 100.0))
        self.fallback_stack_y = float(self.placement_config.get(
            "fallback_y", 0.0))
        self.scoring_names = self.full_rule_config.get(
            "scoring_objects", ["teddybear", "clock", "banana"])
        self.scoring_ids = self.cam.ids_for_names(self.scoring_names)
        self.cake_name = self.full_rule_config.get("cake", "cake")
        self.cake_id = self.cam.ids_for_names([self.cake_name])[0]
        self.cake_drop_zone = self.full_rule_config.get(
            "cake_drop_zone", "Collection_A")
        self.board_drop_zones = self.full_rule_config.get(
            "board_drop_zones", ["Collection_B", "Collection_C", "Collection_D"])
        self.fallback_board_order = self.full_rule_config.get(
            "fallback_board_order", self.scoring_names)
        default_zone_capacities = {
            "Collection_A": 1,
            "Collection_B": 3,
            "Collection_C": 3,
            "Collection_D": 3,
        }
        zone_capacities = self.full_rule_config.get(
            "zone_capacities", default_zone_capacities)
        self.zone_capacities = {
            str(zone): int(count)
            for zone, count in zone_capacities.items()
        }
        self.north_only_config = self.full_rule_config.get("north_only", {})
        self.north_only_enabled = bool(self.north_only_config.get("enabled", False))
        self.north_sorting_mark = str(
            self.north_only_config.get("sorting_mark", "Sorting_N"))
        self.north_transit_mark = str(
            self.north_only_config.get("transit_mark", "Transit_N"))
        self.north_advance_step = max(
            0.0, float(self.north_only_config.get("advance_step", 0.10)))
        self.north_configured_max_advance = max(
            0.0, float(self.north_only_config.get("max_advance", 0.30)))
        self.north_max_advance = self.north_configured_max_advance
        self.north_advance_settle_sec = max(
            0.0, float(self.north_only_config.get("advance_settle_sec", 0.3)))
        self.north_grasp_offset = 0.0
        self.drop_side_escape = self.full_rule_config.get("drop_side_escape", {})
        self.sorting_points = self.full_rule_config.get(
            "sorting_points", ["Sorting_S", "Sorting_W", "Sorting_N", "Sorting_E"])
        self.cake_sorting_points = self.full_rule_config.get(
            "cake_sorting_points", self.sorting_points)
        self.cake_forward_adjustments = {
            str(mark): float(distance)
            for mark, distance in self.full_rule_config.get(
                "cake_forward_adjustments", {}).items()
        }
        self.sorting_forward_adjustments = {
            str(mark): float(distance)
            for mark, distance in self.full_rule_config.get(
                "sorting_forward_adjustments", {}).items()
        }
        self.sorting_exit_back_distance = float(
            self.north_only_config.get(
                "exit_back_distance",
                self.full_rule_config.get("sorting_exit_back_distance", 0.30)))
        self.grasp_search_config = self.full_rule_config.get("grasp_search", {})
        self.grasp_search_enabled = bool(self.grasp_search_config.get("enabled", True))
        self.grasp_search_forward_distance = float(
            self.grasp_search_config.get("forward_distance", 0.05))
        self.grasp_search_yaw_offsets = [
            float(degree)
            for degree in self.grasp_search_config.get("yaw_offsets", [0, 8, -8])
        ]
        self.grasp_search_turn_vel = float(self.grasp_search_config.get("turn_vel", 0.16))
        self.grasp_search_settle_sec = float(self.grasp_search_config.get("settle_sec", 0.35))
        self.grasp_verify_retry_count = max(0, int(
            self.grasp_search_config.get("verify_retry_count", 0)))
        self.grasp_verify_retry_forward_distance = max(
            0.0, float(self.grasp_search_config.get(
                "verify_retry_forward_distance", 0.0)))
        self.cake_search_rounds = int(self.full_rule_config.get("cake_search_rounds", 2))
        self.no_progress_round_limit = int(self.full_rule_config.get(
            "no_progress_round_limit", 2))
        self.navigation_config = self.full_rule_config.get("navigation", {})
        self.route_enabled = self.navigation_config.get("enabled", True)
        self.require_transit_marks = self.navigation_config.get(
            "require_transit_marks", True)
        self.route_pause_sec = float(self.navigation_config.get("route_pause_sec", 0.3))
        self.transit_ring = self.navigation_config.get(
            "transit_ring", ["Transit_N", "Transit_E", "Transit_S", "Transit_W"])
        self.target_sides = self.navigation_config.get("target_sides", {})
        arrival_settle = self.navigation_config.get("arrival_settle_sec", {})
        self.classification_settle_sec = max(
            0.0, float(arrival_settle.get("classification", 0.8)))
        self.sorting_settle_sec = max(
            0.0, float(arrival_settle.get("sorting", 0.4)))
        self.drop_settle_sec = max(
            0.0, float(arrival_settle.get("drop", 0.6)))
        self.last_nav_side = None
        self.last_nav_mark = None
        self.max_scoring_items = int(self.full_rule_config.get("max_scoring_items", 9))
        self.board_detection_frames = int(self.full_rule_config.get("board_detection_frames", 8))
        self.board_retry_count = int(self.full_rule_config.get("board_retry_count", 3))
        self.board_min_conf = float(self.full_rule_config.get("board_min_conf", 0.45))
        self.board_min_x_gap = float(self.full_rule_config.get("board_min_x_gap", 30.0))
        self.board_scan_degrees = [
            float(degree)
            for degree in self.full_rule_config.get("board_scan_degrees", [0, -8, 8, -16, 16])
        ]
        self.board_scan_turn_vel = float(self.full_rule_config.get("board_scan_turn_vel", 0.18))
        self.board_scan_settle_sec = float(self.full_rule_config.get("board_scan_settle_sec", 0.6))
        self.board_roi = self.full_rule_config.get("board_roi", {"enabled": False})
        default_heading_refine_marks = [
            "Classification_area",
            self.cake_drop_zone,
        ]
        default_heading_refine_marks.extend(self.board_drop_zones)
        default_heading_refine_marks.extend(self.sorting_points)
        self.heading_refine_config = self.full_rule_config.get("heading_refine", {})
        self.heading_refine_enabled = bool(self.heading_refine_config.get("enabled", True))
        self.heading_refine_marks = set(self.heading_refine_config.get(
            "marks", default_heading_refine_marks))
        self.heading_refine_tolerance_deg = float(self.heading_refine_config.get(
            "tolerance_deg", 3.0))
        self.heading_refine_max_turn_deg = float(self.heading_refine_config.get(
            "max_turn_deg", 25.0))
        self.heading_refine_turn_vel = float(self.heading_refine_config.get(
            "turn_vel", 0.16))
        self.strict_board_required = bool(self.full_rule_config.get("strict_board_required", True))
        self.debug_disable_time_limit = bool(self.full_rule_config.get("debug_disable_time_limit", False))
        self.task_time_limit_sec = float(self.full_rule_config.get("task_time_limit_sec", 570.0))
        self.task_safety_margin_sec = float(self.full_rule_config.get("task_safety_margin_sec", 15.0))
        self.min_nav_time_left_sec = float(self.full_rule_config.get("min_nav_time_left_sec", 25.0))
        self.min_grasp_time_left_sec = float(self.full_rule_config.get("min_grasp_time_left_sec", 30.0))
        self.min_drop_time_left_sec = float(self.full_rule_config.get("min_drop_time_left_sec", 25.0))
        self.task_start_time = None
        self.task_state = self.STATE_INIT

        # 订阅任务控制指令的话题
        self.task_cmd_sub = rospy.Subscriber("/task_start_flag", String, self.task_cmd_cb) # 订阅任务开始与否信号
        self.task_run_th = threading.Thread(target=lambda: "pass") # 创建线程对象
        self.stop_flag = False  # 任务的启停标志
        self.arm.set_stop_checker(self.task_guard_stop)
        self.robot.set_stop_checker(self.task_guard_stop)

        # 订阅机械臂手动控制的话题
        self.grasp_sub = rospy.Subscriber("grasp", String, self.grasp_cb)

        rospy.loginfo("spark_auto_match_node is ready")
        if self.debug_disable_time_limit:
            rospy.logwarn("debug_disable_time_limit is true: match timer checks are disabled")

    def time_left(self):
        if self.debug_disable_time_limit:
            return float("inf")
        if self.task_start_time is None:
            return float("inf")
        elapsed = (rospy.Time.now() - self.task_start_time).to_sec()
        return self.task_time_limit_sec - elapsed

    def task_guard_stop(self):
        if self.stop_flag:
            return True
        if self.task_start_time is None:
            return False
        return self.time_left() <= self.task_safety_margin_sec

    def enough_time(self, min_left_sec, label):
        if self.stop_flag:
            rospy.logwarn("%s skipped: stop flag is set", label)
            return False
        left = self.time_left()
        if left <= min_left_sec:
            rospy.logwarn(
                "%s skipped: only %.1fs left, need %.1fs",
                label, left, min_left_sec)
            self.stop_flag = True
            return False
        return True

    def safe_sleep(self, seconds):
        end_time = rospy.Time.now() + rospy.Duration(seconds)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            if self.task_guard_stop():
                return False
            rate.sleep()
        return True

    def safe_shutdown_motion(self):
        try:
            self.robot.cancel_all()
        except Exception:
            pass
        try:
            self.arm.interface.set_pump(False)
        except Exception:
            pass
        try:
            self.arm.arm_home()
        except Exception:
            pass

    def validate_marks(self):
        required_marks = ["Classification_area", self.cake_drop_zone]
        required_marks.extend(self.board_drop_zones)
        required_marks.extend(self.cake_sorting_points)
        required_marks.extend(self.sorting_points)
        if self.north_only_enabled:
            required_marks.extend([
                self.north_sorting_mark,
                self.north_transit_mark,
            ])
        if self.route_enabled and self.require_transit_marks:
            required_marks.extend(self.transit_ring)
        if self.route_enabled:
            for target in list(required_marks):
                side = self.target_sides.get(target)
                if side:
                    required_marks.append(side)
        missing = []
        for name in sorted(set(required_marks)):
            if not self.robot.has_mark(name):
                missing.append(name)
        if len(missing) > 0:
            rospy.logerr("missing navigation marks: %s", ", ".join(missing))
            rospy.logerr("please re-run open_guide and record all required points")
            return False
        if self.route_enabled:
            rospy.loginfo("navigation ring enabled: %s", self.transit_ring)
        return True

    def current_task_side(self):
        if self.last_nav_mark in self.transit_ring:
            return None
        return self.target_sides.get(self.last_nav_mark)

    def ring_route_between(self, start_side, target_side):
        if not start_side or start_side == target_side:
            return [] if start_side == target_side else [target_side]
        if start_side not in self.transit_ring or target_side not in self.transit_ring:
            return [target_side]

        ring_len = len(self.transit_ring)
        start_index = self.transit_ring.index(start_side)
        target_index = self.transit_ring.index(target_side)

        clockwise = []
        index = start_index
        while index != target_index:
            index = (index + 1) % ring_len
            clockwise.append(self.transit_ring[index])

        counter_clockwise = []
        index = start_index
        while index != target_index:
            index = (index - 1) % ring_len
            counter_clockwise.append(self.transit_ring[index])

        if len(clockwise) <= len(counter_clockwise):
            return clockwise
        return counter_clockwise

    def ring_route(self, target_side):
        return self.ring_route_between(self.last_nav_side, target_side)

    def route_for_mark(self, name):
        if not self.route_enabled:
            return [name]

        current_side = self.current_task_side()
        if name in self.transit_ring:
            route = []
            if current_side:
                route.append(current_side)
                route.extend(self.ring_route_between(current_side, name))
            else:
                route = self.ring_route(name)
                if len(route) == 0 and self.last_nav_mark != name:
                    route = [name]
            return route

        target_side = self.target_sides.get(name)
        if not target_side:
            return [name]

        route = []
        if current_side:
            route.append(current_side)
            route.extend(self.ring_route_between(current_side, target_side))
        else:
            route = self.ring_route(target_side)
            if len(route) == 0 and self.last_nav_mark != target_side:
                route = [target_side]
        route.append(name)
        return route

    def update_nav_memory(self, waypoint, final_target=None):
        if waypoint in self.transit_ring:
            self.last_nav_side = waypoint
        elif final_target and final_target in self.target_sides:
            self.last_nav_side = self.target_sides[final_target]
        self.last_nav_mark = waypoint

    def goto_task_mark(self, name):
        route = self.route_for_mark(name)
        rospy.loginfo("navigation route to %s: %s", name, route)
        if len(route) == 0:
            return True

        for index, waypoint in enumerate(route):
            if not self.robot.has_mark(waypoint):
                rospy.logerr("route waypoint not found: %s", waypoint)
                return False
            if self.last_nav_mark == waypoint and waypoint in self.transit_ring:
                rospy.loginfo("already at transit waypoint: %s", waypoint)
                continue
            if not self.enough_time(self.min_nav_time_left_sec, "navigation"):
                return False
            if not self.robot.goto_local(
                    waypoint,
                    timeout=max(5.0, min(60.0, self.time_left() - self.task_safety_margin_sec)),
                    should_stop=self.task_guard_stop):
                rospy.logerr("navigation failed at route waypoint: %s", waypoint)
                return False
            final_target = name if index == len(route) - 1 else None
            if final_target and self.should_refine_heading(final_target):
                if not self.robot.refine_mark_heading(
                        final_target,
                        tolerance_deg=self.heading_refine_tolerance_deg,
                        max_turn_deg=self.heading_refine_max_turn_deg,
                        vel=self.heading_refine_turn_vel,
                        should_stop=self.task_guard_stop):
                    rospy.logerr("heading refine failed at route waypoint: %s", waypoint)
                    return False
            self.update_nav_memory(waypoint, final_target)
            if index < len(route) - 1:
                if not self.safe_sleep(self.route_pause_sec):
                    return False
        return True

    def should_refine_heading(self, name):
        return self.heading_refine_enabled and name in self.heading_refine_marks

    def north_grasp_pose(self):
        base_pose = self.robot.mark_pose(self.north_sorting_mark)
        yaw = self.robot.pose_yaw(base_pose)
        return Pose(
            Point(
                base_pose.position.x + self.north_grasp_offset * math.cos(yaw),
                base_pose.position.y + self.north_grasp_offset * math.sin(yaw),
                base_pose.position.z),
            Quaternion(
                base_pose.orientation.x,
                base_pose.orientation.y,
                base_pose.orientation.z,
                base_pose.orientation.w))

    def goto_north_grasp_point(self):
        if not self.goto_task_mark(self.north_transit_mark):
            rospy.logerr("navigation to north transit failed: %s", self.north_transit_mark)
            return False

        fallback_attempted = False
        while True:
            target_pose = self.north_grasp_pose()
            target_name = "%s_runtime_%.2fm" % (
                self.north_sorting_mark,
                self.north_grasp_offset)
            rospy.loginfo(
                "go to north runtime grasp point: base=%s offset=%.2fm",
                self.north_sorting_mark,
                self.north_grasp_offset)
            if not self.enough_time(self.min_nav_time_left_sec, "north grasp navigation"):
                return False
            if self.robot.goto_pose(
                    target_name,
                    target_pose,
                    timeout=max(5.0, min(60.0, self.time_left() - self.task_safety_margin_sec)),
                    should_stop=self.task_guard_stop):
                break
            if (
                    fallback_attempted or
                    self.north_advance_step <= 0.0 or
                    self.north_grasp_offset <= 1e-6):
                rospy.logerr("navigation failed at north runtime grasp point")
                return False
            failed_offset = self.north_grasp_offset
            self.north_grasp_offset = max(
                0.0, self.north_grasp_offset - self.north_advance_step)
            self.north_max_advance = min(
                self.north_max_advance, self.north_grasp_offset)
            fallback_attempted = True
            rospy.logwarn(
                "north runtime grasp navigation failed at %.2fm, retry previous offset %.2fm and cap this task there",
                failed_offset,
                self.north_grasp_offset)
        if self.should_refine_heading(self.north_sorting_mark):
            if not self.robot.refine_pose_heading(
                    target_name,
                    target_pose,
                    tolerance_deg=self.heading_refine_tolerance_deg,
                    max_turn_deg=self.heading_refine_max_turn_deg,
                    vel=self.heading_refine_turn_vel,
                    should_stop=self.task_guard_stop):
                rospy.logerr("heading refine failed at north runtime grasp point")
                return False
        self.last_nav_side = self.north_transit_mark
        self.last_nav_mark = self.north_sorting_mark
        return True

    def advance_north_grasp_point(self):
        remaining = self.north_max_advance - self.north_grasp_offset
        if self.north_advance_step <= 0.0 or remaining <= 1e-6:
            rospy.logwarn(
                "north grasp advance limit reached: %.2fm",
                self.north_grasp_offset)
            return False

        move_distance = min(self.north_advance_step, remaining)
        rospy.logwarn(
            "no graspable target at north point, advance %.2fm",
            move_distance)
        if not self.robot.step_go(move_distance):
            rospy.logerr("north grasp advance failed")
            return False
        self.north_grasp_offset += move_distance
        rospy.logwarn(
            "north runtime grasp point updated: offset=%.2fm",
            self.north_grasp_offset)
        return self.safe_sleep(self.north_advance_settle_sec)

    def in_board_roi(self, item):
        if not self.board_roi.get("enabled", False):
            return True
        x = item[1][0]
        y = item[1][1]
        return (
            x >= float(self.board_roi.get("x_min", 0)) and
            x <= float(self.board_roi.get("x_max", 9999)) and
            y >= float(self.board_roi.get("y_min", 0)) and
            y <= float(self.board_roi.get("y_max", 9999))
        )

    def detect_board_order_once(self, board_ids, attempt_label):
        samples = []
        for _ in range(self.board_detection_frames):
            if self.task_guard_stop():
                return []
            detections = self.cam.detector(search_ids=board_ids, timeout=2)
            for item in detections:
                if item[1][2] >= self.board_min_conf and self.in_board_roi(item):
                    samples.append(item)
            if not self.safe_sleep(0.2):
                return []

        best_by_id = {}
        for item in samples:
            item_id = int(item[0])
            score = item[1][2]
            if item_id not in best_by_id or score > best_by_id[item_id][1][2]:
                best_by_id[item_id] = item

        if len(best_by_id) < len(board_ids):
            missing = [
                self.cam.item_name(item_id)
                for item_id in board_ids
                if item_id not in best_by_id
            ]
            rospy.logwarn(
                "classification board %s incomplete, missing: %s",
                attempt_label, missing)
            return []

        ordered_items = sorted(best_by_id.values(), key=lambda item: item[1][0])
        x_positions = [item[1][0] for item in ordered_items]
        gaps = [
            x_positions[index + 1] - x_positions[index]
            for index in range(len(x_positions) - 1)
        ]
        if not all(gap >= self.board_min_x_gap for gap in gaps):
            rospy.logwarn(
                "classification board %s rejected: x gap too small, x=%s, gaps=%s",
                attempt_label, x_positions, gaps)
            return []

        ordered_names = [self.cam.item_name(item[0]) for item in ordered_items]
        y_positions = [item[1][1] for item in ordered_items]
        rospy.loginfo(
            "classification board %s detected order: %s, x=%s, y=%s, gaps=%s",
            attempt_label, ordered_names, x_positions, y_positions, gaps)
        return ordered_names

    def detect_board_mapping(self):
        '''
        分类板从左到右依次对应 B/C/D。严格模式下识别不完整则返回空映射。
        @return: {item_id: drop_zone}
        '''
        board_ids = self.cam.ids_for_names(self.scoring_names)
        ordered_names = []
        current_offset = 0.0

        for target_offset in self.board_scan_degrees:
            if self.task_guard_stop():
                return {}
            delta = target_offset - current_offset
            if abs(delta) > 0.1:
                if not self.enough_time(self.min_nav_time_left_sec, "classification board scan"):
                    return {}
                rospy.logwarn("scan classification board: turn %.1f deg", delta)
                if not self.robot.turn_body(
                        delta,
                        vel=self.board_scan_turn_vel,
                        should_stop=self.task_guard_stop):
                    return {}
                current_offset = target_offset
                if not self.safe_sleep(self.board_scan_settle_sec):
                    return {}

            for attempt in range(self.board_retry_count):
                if not self.enough_time(self.min_nav_time_left_sec, "classification board detection"):
                    return {}
                label = "offset %.1f attempt %d/%d" % (
                    current_offset, attempt + 1, self.board_retry_count)
                ordered_names = self.detect_board_order_once(board_ids, label)
                if len(ordered_names) > 0:
                    break
            if len(ordered_names) > 0:
                break

        if len(ordered_names) == 0:
            if abs(current_offset) > 0.1:
                self.robot.turn_body(
                    -current_offset,
                    vel=self.board_scan_turn_vel,
                    should_stop=self.task_guard_stop)
            if self.strict_board_required:
                rospy.logerr("classification board detection failed in strict mode")
                return {}
            ordered_names = self.fallback_board_order
            rospy.logwarn(
                "classification board detection failed, fallback order: %s",
                ordered_names)

        drop_map = {}
        for index, item_name in enumerate(ordered_names):
            if index >= len(self.board_drop_zones):
                break
            item_ids = self.cam.ids_for_names([item_name])
            if len(item_ids) == 0:
                continue
            drop_map[item_ids[0]] = self.board_drop_zones[index]

        readable = {
            self.cam.item_name(item_id): drop_zone
            for item_id, drop_zone in drop_map.items()
        }
        rospy.loginfo("drop map: %s", readable)
        return drop_map

    def try_grasp_scan(
            self,
            sorting_name,
            search_ids,
            allow_distant_fallback=True):
        current_offset = 0.0
        best_comfort_target = None
        best_comfort_offset = 0.0
        best_fallback_target = None
        best_fallback_offset = 0.0
        grasp_result = None
        saw_target = False
        saw_too_far_target = False
        saw_other_rejection = False
        try:
            for target_offset in self.grasp_search_yaw_offsets:
                if self.task_guard_stop():
                    return None
                delta = target_offset - current_offset
                if abs(delta) > 0.1:
                    rospy.loginfo(
                        "grasp search turn at %s: %.1f deg",
                        sorting_name, delta)
                    if not self.robot.turn_body(
                            delta,
                            vel=self.grasp_search_turn_vel,
                            should_stop=self.task_guard_stop):
                        return None
                    current_offset = target_offset
                    if not self.safe_sleep(self.grasp_search_settle_sec):
                        return None

                rospy.loginfo(
                    "evaluate grasp at %s, view offset %.1f deg",
                    sorting_name, current_offset)
                target = self.arm.find_best_grasp_target(search_ids=search_ids)
                if target is None:
                    if self.arm.last_grasp_failure_reason == "target_too_far":
                        saw_target = True
                        saw_too_far_target = True
                    elif self.arm.last_grasp_failure_reason != "no_detection":
                        saw_target = True
                        saw_other_rejection = True
                    continue
                saw_target = True
                target["view_offset"] = current_offset
                if target["preference_tier"] == "comfort":
                    if (
                            best_comfort_target is None or
                            target["preference_score"] >
                            best_comfort_target["preference_score"]):
                        best_comfort_target = target
                        best_comfort_offset = current_offset
                elif (
                        best_fallback_target is None or
                        target["preference_score"] >
                        best_fallback_target["preference_score"]):
                    best_fallback_target = target
                    best_fallback_offset = current_offset

            best_target = best_comfort_target
            best_offset = best_comfort_offset
            if best_target is None:
                best_target = best_fallback_target
                best_offset = best_fallback_offset
                if best_target is not None:
                    preference = self.arm.target_preference_for(best_target)
                    advance_x_min = float(preference.get(
                        "advance_x_min",
                        preference.get("comfort_x_max", 330.0)))
                    if (
                            not allow_distant_fallback and
                            best_target["x"] > advance_x_min):
                        rospy.logwarn(
                            "defer distant fallback %s at x=%.1f above %.1f; advance first",
                            best_target["item_name"],
                            best_target["x"],
                            advance_x_min)
                        self.arm.grasp_status_pub.publish(String("1"))
                        self.arm.last_grasp_failure_reason = "target_too_far"
                        return None
                    rospy.logwarn(
                        "no comfort target in complete grasp scan, use fallback %s",
                        best_target["item_name"])

            if best_target is None:
                self.arm.grasp_status_pub.publish(String("1"))
                if saw_other_rejection:
                    self.arm.last_grasp_failure_reason = "no_accepted_target"
                elif saw_too_far_target:
                    self.arm.last_grasp_failure_reason = "target_too_far"
                elif saw_target:
                    self.arm.last_grasp_failure_reason = "no_accepted_target"
                else:
                    self.arm.last_grasp_failure_reason = "no_detection"
                return None

            delta = best_offset - current_offset
            if abs(delta) > 0.1:
                rospy.loginfo(
                    "turn back to best grasp view at %s: %.1f deg",
                    sorting_name, delta)
                if not self.robot.turn_body(
                        delta,
                        vel=self.grasp_search_turn_vel,
                        should_stop=self.task_guard_stop):
                    return None
                current_offset = best_offset
                if not self.safe_sleep(self.grasp_search_settle_sec):
                    return None

            rospy.loginfo(
                "best grasp view at %s: %.1f deg, comfort_score=%.1f",
                sorting_name,
                best_offset,
                best_target["preference_score"])
            grasp_result = self.arm.execute_grasp_target(best_target)
            return grasp_result
        finally:
            if (
                    grasp_result is None and
                    abs(current_offset) > 0.1 and
                    not rospy.is_shutdown()):
                rospy.loginfo(
                    "grasp search return to center at %s: %.1f deg",
                    sorting_name, -current_offset)
                self.robot.turn_body(
                    -current_offset,
                    vel=self.grasp_search_turn_vel,
                    should_stop=self.task_guard_stop)

    def grasp_attempts_at_current_pose(
            self,
            sorting_name,
            search_ids,
            allow_distant_fallback=True):
        item_type = None
        grasp_attempts = max(1, self.grasp_verify_retry_count + 1)
        for attempt in range(grasp_attempts):
            self.arm.last_grasp_failure_reason = None
            if self.grasp_search_enabled:
                item_type = self.try_grasp_scan(
                    sorting_name,
                    search_ids,
                    allow_distant_fallback=allow_distant_fallback)
            else:
                rospy.loginfo("try grasp at %s", sorting_name)
                item_type = self.arm.grasp(search_ids=search_ids)

            if item_type is not None:
                break
            if self.arm.last_grasp_failure_reason != "verify_failed":
                break
            if attempt >= grasp_attempts - 1:
                break
            rospy.logwarn(
                "grasp verification failed at %s, retry %d/%d",
                sorting_name, attempt + 1, grasp_attempts - 1)
            if self.grasp_verify_retry_forward_distance > 0.0:
                rospy.logwarn(
                    "advance %.2fm before grasp verification retry at %s",
                    self.grasp_verify_retry_forward_distance,
                    sorting_name)
                if not self.robot.step_go(
                        self.grasp_verify_retry_forward_distance):
                    return None
            if not self.safe_sleep(0.3):
                return None
        return item_type

    def pick_from_sorting(self, sorting_name, search_ids, pre_grasp_forward=None):
        if not self.enough_time(self.min_nav_time_left_sec, "go to sorting point"):
            return None
        rospy.loginfo("go to sorting point: %s", sorting_name)
        north_runtime_pick = (
            self.north_only_enabled and
            sorting_name == self.north_sorting_mark)
        if north_runtime_pick:
            if not self.goto_north_grasp_point():
                return None
        elif not self.goto_task_mark(sorting_name):
            return None
        if not self.arm.wait_for_motion():
            return None
        if not self.safe_sleep(self.sorting_settle_sec):
            return None
        if not self.enough_time(self.min_grasp_time_left_sec, "grasp object"):
            return None
        if pre_grasp_forward is None:
            if self.grasp_search_enabled:
                pre_grasp_forward = self.grasp_search_forward_distance
            else:
                pre_grasp_forward = 0.0
        else:
            pre_grasp_forward = float(pre_grasp_forward)
        if pre_grasp_forward > 0.0:
            rospy.loginfo(
                "pre-grasp forward adjustment at %s: %.2fm",
                sorting_name, pre_grasp_forward)
            if not self.robot.step_go(pre_grasp_forward):
                return None
            if not self.safe_sleep(0.3):
                return None

        while True:
            allow_distant_fallback = (
                not north_runtime_pick or
                self.north_grasp_offset >= self.north_max_advance - 1e-6)
            item_type = self.grasp_attempts_at_current_pose(
                sorting_name,
                search_ids,
                allow_distant_fallback=allow_distant_fallback)
            if item_type is not None:
                break
            if not north_runtime_pick:
                break
            if self.arm.last_grasp_failure_reason not in [
                    "no_detection", "target_too_far", "no_accepted_target"]:
                break
            if not self.advance_north_grasp_point():
                break

        if north_runtime_pick:
            rospy.loginfo(
                "step back %.2fm from north sorting point, grasped=%s",
                self.sorting_exit_back_distance,
                item_type is not None)
            if not self.robot.step_back(self.sorting_exit_back_distance):
                rospy.logerr("failed to leave north sorting point")
                self.stop_flag = True
                return item_type
            if not self.safe_sleep(0.3):
                return item_type
        elif item_type is not None:
            rospy.loginfo("step back from sorting point with grasped item")
            self.robot.step_back(self.sorting_exit_back_distance)
            self.safe_sleep(0.3)
        else:
            rospy.loginfo("step back from sorting point without item")
            self.robot.step_back()
        return item_type

    def shift_before_drop(self, drop_zone):
        escape = self.drop_side_escape.get(drop_zone)
        if not escape:
            return True

        turn_degree = float(escape.get("turn_degree", 0.0))
        move_distance = max(0.0, float(escape.get("distance", 0.0)))
        turn_vel = max(0.08, min(0.35, abs(float(
            escape.get("turn_vel", 0.18)))))
        linear_vel = max(0.05, min(0.35, abs(float(
            escape.get("linear_vel", 0.10)))))
        turn_timeout = max(3.0, float(escape.get("turn_timeout_sec", 12.0)))
        move_timeout = max(
            3.0,
            float(escape.get(
                "move_timeout_sec", move_distance / linear_vel + 3.0)))
        if abs(turn_degree) < 0.1 or move_distance <= 0.0:
            return True

        direction = "left" if turn_degree > 0.0 else "right"
        rospy.loginfo(
            "drop escape for %s: turn %s %.1fdeg at %.2frad/s, move %.2fm at %.2fm/s, restore heading",
            drop_zone,
            direction,
            abs(turn_degree),
            turn_vel,
            move_distance,
            linear_vel)
        if not self.robot.turn_body(
                turn_degree,
                vel=turn_vel,
                timeout=turn_timeout,
                should_stop=self.task_guard_stop):
            rospy.logerr("drop escape initial turn failed for %s", drop_zone)
            return False

        moved = self.robot.step_go(
            move_distance,
            vel=linear_vel,
            timeout=move_timeout,
            should_stop=self.task_guard_stop)
        restored = self.robot.turn_body(
            -turn_degree,
            vel=turn_vel,
            timeout=turn_timeout,
            should_stop=self.task_guard_stop)
        if not moved:
            rospy.logerr("drop escape lateral move failed for %s", drop_zone)
            return False
        if not restored:
            rospy.logerr("drop escape heading restore failed for %s", drop_zone)
            return False
        return self.safe_sleep(0.3)

    def select_scoring_drop_target(self, drop_zone, stack_state):
        main_count = int(stack_state.get("main_count", 0))
        stack_capacity = self.zone_capacity(drop_zone)

        stack_name = "main"
        stack_count = main_count
        x = self.main_stack_x
        y = self.main_stack_y
        vision = {"ok": True, "reason": "fixed_main_stack"}

        if stack_count >= stack_capacity:
            return None, "{}_stack_full".format(stack_name)
        if not self.arm.drop_target_reachable(x, y, stack_count):
            return None, "{}_drop_pose_unreachable".format(stack_name)

        target = {
            "stack": stack_name,
            "stack_count": stack_count,
            "x": x,
            "y": y,
            "vision": vision,
        }
        rospy.logwarn(
            "placement target %s stack=%s count=%d x=%.1f y=%.1f vision=%s stable=%s confidence=%s correction=(%s,%s)",
            drop_zone,
            stack_name,
            stack_count,
            x,
            y,
            vision.get("reason"),
            vision.get("stable_frames"),
            "%.2f" % vision["confidence"] if "confidence" in vision else "None",
            "%.1f" % vision["correction_x"] if "correction_x" in vision else "None",
            "%.1f" % vision["correction_y"] if "correction_y" in vision else "None")
        return target, None

    def drop_to_zone(
            self,
            drop_zone,
            stack_count=0,
            drop_x=None,
            drop_y=None):
        if not self.prepare_drop_zone(drop_zone):
            return False
        if not self.arm.drop(
                stack_count=stack_count,
                x=drop_x,
                y=drop_y):
            return False
        return self.return_after_drop()

    def prepare_drop_zone(self, drop_zone):
        if not self.enough_time(self.min_nav_time_left_sec, "go to drop zone"):
            return False
        if not self.shift_before_drop(drop_zone):
            return False
        rospy.loginfo("go to drop zone: %s", drop_zone)
        if not self.goto_task_mark(drop_zone):
            rospy.logerr("navigation to drop zone failed: %s", drop_zone)
            return False
        if not self.safe_sleep(self.drop_settle_sec):
            return False
        if not self.enough_time(self.min_drop_time_left_sec, "drop object"):
            return False
        return True

    def return_after_drop(self):
        if self.north_only_enabled:
            rospy.loginfo(
                "return to north transit after drop: %s",
                self.north_transit_mark)
            if not self.goto_task_mark(self.north_transit_mark):
                rospy.logerr(
                    "navigation to north transit after drop failed: %s",
                    self.north_transit_mark)
                return False
        return True

    def pick_and_drop_first_available(
            self,
            search_ids,
            sorting_points,
            drop_zone,
            label,
            rounds=1,
            forward_adjustments=None):
        if forward_adjustments is None:
            forward_adjustments = {}
        for _ in range(rounds):
            for sorting_name in sorting_points:
                if self.task_guard_stop():
                    return False
                item_type = self.pick_from_sorting(
                    sorting_name,
                    search_ids,
                    pre_grasp_forward=float(forward_adjustments.get(sorting_name, 0.0)))
                if item_type is None:
                    rospy.logwarn("%s not found at %s", label, sorting_name)
                    continue
                rospy.loginfo("got %s, drop to %s", self.cam.item_name(item_type), drop_zone)
                return self.drop_to_zone(drop_zone)
        return False

    def zone_capacity(self, zone):
        return int(self.zone_capacities.get(zone, self.max_scoring_items))

    def zone_full(self, zone, zone_counts):
        return zone_counts.get(zone, 0) >= self.zone_capacity(zone)

    def item_full(self, item_id, drop_zone, item_counts):
        return item_counts.get(item_id, 0) >= self.zone_capacity(drop_zone)

    def active_drop_map(self, drop_map, zone_counts, item_counts):
        return {
            item_id: drop_zone
            for item_id, drop_zone in drop_map.items()
            if (
                not self.zone_full(drop_zone, zone_counts) and
                not self.item_full(item_id, drop_zone, item_counts)
            )
        }

    def scoring_zones_full(self, zone_counts):
        for drop_zone in self.board_drop_zones:
            if not self.zone_full(drop_zone, zone_counts):
                return False
        return True

    def log_score_state(self, zone_counts, item_counts):
        readable_items = {
            self.cam.item_name(item_id): count
            for item_id, count in item_counts.items()
        }
        rospy.loginfo("zone counts: %s", zone_counts)
        rospy.loginfo("item counts: %s", readable_items)

    def set_task_state(self, state):
        self.task_state = state
        rospy.logwarn("===== TASK STATE: %s =====", state)

    def new_task_context(self):
        zone_counts = {self.cake_drop_zone: 0}
        for drop_zone in self.board_drop_zones:
            zone_counts[drop_zone] = 0
        return {
            "drop_map": {},
            "zone_counts": zone_counts,
            "stack_states": {
                drop_zone: {
                    "main_count": 0,
                    "fallback_count": 0,
                    "main_center": None,
                    "vision_confidence": 0.0,
                }
                for drop_zone in self.board_drop_zones
            },
            "item_counts": {},
            "delivered": 0,
            "held_item": None,
            "held_drop_zone": None,
            "no_progress_rounds": 0,
        }

    def pick_first_available(
            self,
            search_ids,
            sorting_points,
            label,
            rounds=1,
            forward_adjustments=None):
        if forward_adjustments is None:
            forward_adjustments = {}
        consumed_forward_adjustments = set()
        for _ in range(rounds):
            for sorting_name in sorting_points:
                if self.task_guard_stop():
                    return None
                pre_grasp_forward = 0.0
                if sorting_name not in consumed_forward_adjustments:
                    pre_grasp_forward = forward_adjustments.get(sorting_name)
                    consumed_forward_adjustments.add(sorting_name)
                item_type = self.pick_from_sorting(
                    sorting_name,
                    search_ids,
                    pre_grasp_forward=pre_grasp_forward)
                if item_type is None:
                    rospy.logwarn("%s not found at %s", label, sorting_name)
                    continue
                return item_type
        return None

    def pick_scoring_item_once(self, active_targets, context):
        for sorting_name in self.sorting_points:
            if self.task_guard_stop():
                return None, None

            item_type = self.pick_from_sorting(
                sorting_name,
                list(active_targets.keys()),
                pre_grasp_forward=self.sorting_forward_adjustments.get(sorting_name))
            if item_type is None:
                continue

            drop_zone = active_targets.get(item_type)
            if drop_zone is None:
                rospy.logwarn("unknown drop zone for item: %s", self.cam.item_name(item_type))
                continue
            if self.zone_full(drop_zone, context["zone_counts"]):
                rospy.logwarn("drop zone is full, skip item: %s -> %s",
                              self.cam.item_name(item_type), drop_zone)
                continue
            return item_type, drop_zone
        return None, None

    def state_init(self, context):
        if not self.validate_marks():
            self.stop_flag = True
            return self.STATE_ABORT

        self.arm.arm_home()
        rospy.loginfo("start 2026 full-rule task now.")
        return self.STATE_LEAVE_START

    def state_leave_start(self, context):
        if not self.enough_time(self.min_nav_time_left_sec, "leave start area"):
            return self.STATE_ABORT
        if not self.robot.step_go(0.3):
            self.stop_flag = True
            return self.STATE_ABORT
        return self.STATE_ARM_READY

    def state_arm_ready(self, context):
        self.arm.arm_position_reset()
        self.arm.arm_grasp_ready()
        return self.STATE_GO_CLASSIFICATION

    def state_go_classification(self, context):
        if not self.goto_task_mark("Classification_area"):
            rospy.logerr("Navigation to Classification_area failed, please run task again")
            self.stop_flag = True
            return self.STATE_ABORT
        if not self.safe_sleep(self.classification_settle_sec):
            return self.STATE_ABORT
        return self.STATE_READ_BOARD

    def state_read_board(self, context):
        context["drop_map"] = self.detect_board_mapping()
        return self.STATE_PICK_CAKE

    def state_pick_cake(self, context):
        item_type = self.pick_first_available(
            [self.cake_id],
            self.cake_sorting_points,
            self.cake_name,
            rounds=self.cake_search_rounds,
            forward_adjustments=self.cake_forward_adjustments)
        if item_type is None:
            rospy.logwarn("cake task failed, continue with other scoring objects")
            if len(context["drop_map"]) == 0:
                rospy.logerr("empty drop map, no scoring objects can be routed")
                return self.STATE_FINISH
            return self.STATE_PICK_SCORING_ITEM

        context["held_item"] = item_type
        context["held_drop_zone"] = self.cake_drop_zone
        rospy.loginfo("got %s, drop to %s", self.cam.item_name(item_type), self.cake_drop_zone)
        return self.STATE_DROP_CAKE_A

    def state_drop_cake_a(self, context):
        stack_count = context["zone_counts"].get(self.cake_drop_zone, 0)
        if not self.drop_to_zone(self.cake_drop_zone, stack_count=stack_count):
            self.stop_flag = True
            return self.STATE_ABORT

        zone_counts = context["zone_counts"]
        item_counts = context["item_counts"]
        zone_counts[self.cake_drop_zone] = zone_counts.get(self.cake_drop_zone, 0) + 1
        item_counts[self.cake_id] = item_counts.get(self.cake_id, 0) + 1
        context["held_item"] = None
        context["held_drop_zone"] = None
        rospy.loginfo("cake task finished")

        if len(context["drop_map"]) == 0:
            rospy.logerr("empty drop map, no scoring objects can be routed")
            return self.STATE_FINISH
        return self.STATE_PICK_SCORING_ITEM

    def state_pick_scoring_item(self, context):
        zone_counts = context["zone_counts"]
        item_counts = context["item_counts"]

        if (
                context["delivered"] >= self.max_scoring_items or
                self.scoring_zones_full(zone_counts)):
            return self.STATE_FINISH

        active_targets = self.active_drop_map(
            context["drop_map"], zone_counts, item_counts)
        if len(active_targets) == 0:
            rospy.logwarn("all mapped scoring items or zones are full")
            return self.STATE_FINISH

        item_type, drop_zone = self.pick_scoring_item_once(active_targets, context)
        if item_type is None:
            context["no_progress_rounds"] += 1
            rospy.logwarn(
                "no scoring object delivered in full sorting round %d/%d",
                context["no_progress_rounds"],
                self.no_progress_round_limit)
            if context["no_progress_rounds"] >= self.no_progress_round_limit:
                return self.STATE_FINISH
            return self.STATE_PICK_SCORING_ITEM

        context["held_item"] = item_type
        context["held_drop_zone"] = drop_zone
        rospy.loginfo(
            "got %s, mapped drop zone is %s",
            self.cam.item_name(item_type),
            drop_zone)
        return self.STATE_DROP_SCORING_ITEM

    def state_drop_scoring_item(self, context):
        item_type = context["held_item"]
        drop_zone = context["held_drop_zone"]
        if item_type is None or drop_zone is None:
            rospy.logerr("drop scoring state entered without a held item")
            self.stop_flag = True
            return self.STATE_ABORT

        if not self.prepare_drop_zone(drop_zone):
            self.stop_flag = True
            return self.STATE_ABORT

        stack_state = context["stack_states"][drop_zone]
        drop_target, failure_reason = self.select_scoring_drop_target(
            drop_zone, stack_state)
        if drop_target is None:
            rospy.logerr(
                "failed to select safe drop target for %s: %s",
                drop_zone,
                failure_reason)
            self.stop_flag = True
            return self.STATE_ABORT

        if not self.arm.drop(
                stack_count=drop_target["stack_count"],
                x=drop_target["x"],
                y=drop_target["y"]):
            self.stop_flag = True
            return self.STATE_ABORT
        if not self.return_after_drop():
            self.stop_flag = True
            return self.STATE_ABORT

        stack_name = drop_target["stack"]
        stack_state["{}_count".format(stack_name)] += 1
        vision = drop_target["vision"]
        if stack_name == "main" and vision.get("ok"):
            stack_state["main_center"] = [
                drop_target["x"], drop_target["y"]]
            stack_state["vision_confidence"] = float(
                vision.get("confidence", 1.0))
        rospy.logwarn(
            "placement state %s main_count=%d fallback_count=%d main_center=%s confidence=%.2f",
            drop_zone,
            stack_state["main_count"],
            stack_state["fallback_count"],
            stack_state["main_center"],
            stack_state["vision_confidence"])
        context["delivered"] += 1
        context["zone_counts"][drop_zone] = context["zone_counts"].get(drop_zone, 0) + 1
        context["item_counts"][item_type] = context["item_counts"].get(item_type, 0) + 1
        context["held_item"] = None
        context["held_drop_zone"] = None
        context["no_progress_rounds"] = 0
        self.log_score_state(context["zone_counts"], context["item_counts"])
        return self.STATE_PICK_SCORING_ITEM

    def state_finish(self, context):
        self.arm.arm_home()
        self.log_score_state(context["zone_counts"], context["item_counts"])
        rospy.logwarn(
            "***** task finished, delivered scoring objects: %d *****",
            context["delivered"])
        rospy.logwarn("if you want to run task again, resend /task_start_flag or press Ctrl+C")

    def state_abort(self, context):
        rospy.logerr("task aborted in state: %s", self.task_state)

    # 接收到启动自动赛信号，开始执行任务
    def run_task(self):
        self.task_start_time = rospy.Time.now()
        self.north_grasp_offset = 0.0
        self.north_max_advance = self.north_configured_max_advance
        if self.north_only_enabled:
            rospy.logwarn("north runtime grasp point reset: offset=0.00m")
        context = self.new_task_context()
        state = self.STATE_INIT

        while not rospy.is_shutdown():
            if self.task_guard_stop() and state not in [self.STATE_FINISH, self.STATE_ABORT]:
                state = self.STATE_ABORT

            self.set_task_state(state)

            if state == self.STATE_INIT:
                state = self.state_init(context)
            elif state == self.STATE_LEAVE_START:
                state = self.state_leave_start(context)
            elif state == self.STATE_ARM_READY:
                state = self.state_arm_ready(context)
            elif state == self.STATE_GO_CLASSIFICATION:
                state = self.state_go_classification(context)
            elif state == self.STATE_READ_BOARD:
                state = self.state_read_board(context)
            elif state == self.STATE_PICK_CAKE:
                state = self.state_pick_cake(context)
            elif state == self.STATE_DROP_CAKE_A:
                state = self.state_drop_cake_a(context)
            elif state == self.STATE_PICK_SCORING_ITEM:
                state = self.state_pick_scoring_item(context)
            elif state == self.STATE_DROP_SCORING_ITEM:
                state = self.state_drop_scoring_item(context)
            elif state == self.STATE_FINISH:
                self.state_finish(context)
                return
            elif state == self.STATE_ABORT:
                self.stop_flag = True
                self.state_abort(context)
                return
            else:
                rospy.logerr("unknown task state: %s", state)
                self.stop_flag = True
                return

    def task_thread_entry(self):
        try:
            self.run_task()
        except Exception as e:
            rospy.logerr("task thread crashed: %s", e)
            rospy.logerr(traceback.format_exc())
        finally:
            if self.stop_flag or self.task_guard_stop():
                self.safe_shutdown_motion()
            self.task_start_time = None
            rospy.logwarn("task thread exited")

    def task_cmd_cb(self, flag):
        cmd = flag.data.strip().lower()
        rospy.logwarn("received /task_start_flag: %s", cmd)

        if cmd in ["1", "start", "run", "true"]:
            if not self.task_run_th.is_alive():
                self.stop_flag = False
                self.last_nav_side = None
                self.last_nav_mark = None
                self.task_run_th = threading.Thread(target=self.task_thread_entry, args=())
                self.task_run_th.daemon = True
                self.task_run_th.start()
                rospy.logwarn("start task thread")
            else:
                rospy.logwarn("task is already running")
        elif cmd in ["0", "stop", "false"]:
            if self.task_run_th.is_alive():
                rospy.logwarn("stopping task thread...")
                self.stop_flag = True
                try:
                    self.robot.cancel_all()
                    self.arm.interface.set_pump(False)
                except Exception:
                    pass
            else:
                rospy.logwarn("task is not running")
        else:
            rospy.logwarn("unknown task command: %s, use 1/start or 0/stop", cmd)

    def grasp_cb(self, msg):
        if not self.task_run_th.is_alive():
            cmd = msg.data.strip().lower()
            if cmd == "1":
                self.arm.grasp()
            elif cmd == "0":
                self.arm.drop()
                self.arm.arm_grasp_ready()
            elif cmd in self.cam.items:
                rospy.logwarn("manual grasp target: %s", cmd)
                self.arm.grasp(search_names=[cmd])
            else:
                rospy.logwarn("grasp msg error: %s", cmd)


if __name__ == '__main__':
    try:
        AutoAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Mark_move finished.")
