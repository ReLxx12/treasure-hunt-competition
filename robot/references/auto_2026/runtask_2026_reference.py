#!/usr/bin/env python3
"""Reference task node for the 2026 Treasure Hunt auto match.

This is not a drop-in winning solution. It is a readable reference that keeps
the 2024 demo interfaces and rewrites the high-level task flow for the 2026
rules:

1. Leave A start area.
2. Go to the classification board.
3. Get the item -> zone mapping. This reference reads it from YAML; replace
   read_board_mapping() with real vision later.
4. Pick cake first and drop it in A.
5. Pick toy/clock/banana and drop them according to the board mapping.
"""

import os
import sys
import threading
from typing import Dict, Iterable, List, Optional, Tuple

import actionlib
import rospy
import rospkg
import yaml
from actionlib_msgs.msg import GoalID, GoalStatus
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

import common.msg
import swiftpro.msg
from common.msg import MoveStraightDistanceAction, TurnBodyDegreeAction
from swiftpro.msg import position


ItemDetection = Tuple[str, int, float, float]


def load_yaml(path: str) -> dict:
    with open(os.path.expanduser(path), "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data or {}


class SwiftProInterface:
    def __init__(self):
        self.arm_position_pub = rospy.Publisher(
            "position_write_topic", swiftpro.msg.position, queue_size=1
        )
        self.arm_pump_pub = rospy.Publisher(
            "pump_topic", swiftpro.msg.status, queue_size=1
        )
        self.arm_status_pub = rospy.Publisher(
            "swiftpro_status_topic", swiftpro.msg.status, queue_size=1
        )

    def set_pose(self, x: float, y: float, z: float, delay: float = 2.0):
        pos = position()
        pos.x = x
        pos.y = y
        pos.z = z
        self.arm_position_pub.publish(pos)
        rospy.sleep(delay)

    def set_pump(self, enable: bool, delay: float = 1.0):
        self.arm_pump_pub.publish(swiftpro.msg.status(1 if enable else 0))
        rospy.sleep(delay)

    def reset_status(self):
        self.arm_status_pub.publish(swiftpro.msg.status(0))
        rospy.sleep(0.2)
        self.arm_status_pub.publish(swiftpro.msg.status(1))
        rospy.sleep(0.2)


class VisionClient:
    def __init__(self, item_ids: Dict[str, int], detect_timeout_sec: float):
        self.item_ids = item_ids
        self.id_to_item = {int(v): k for k, v in item_ids.items()}
        self.detect_timeout_sec = detect_timeout_sec

    def detect_items(self, wanted: Iterable[str]) -> List[ItemDetection]:
        wanted_ids = {int(self.item_ids[name]) for name in wanted if name in self.item_ids}
        detections: List[ItemDetection] = []

        try:
            msg = rospy.wait_for_message(
                "/objects", Detection2DArray, timeout=self.detect_timeout_sec
            )
        except Exception:
            return detections

        for det in msg.detections:
            if not det.results:
                continue
            item_id = int(det.results[0].id)
            if item_id not in wanted_ids:
                continue

            item_name = self.id_to_item.get(item_id)
            if item_name is None:
                continue
            detections.append(
                (item_name, item_id, det.bbox.center.x, det.bbox.center.y)
            )

        # Prefer the visually nearest/lower block, matching the 2024 demo style.
        detections.sort(key=lambda item: item[3])
        return detections


class ArmAction:
    def __init__(self, cfg: dict, vision: VisionClient):
        self.cfg = cfg
        self.vision = vision
        self.interface = SwiftProInterface()
        self.grasp_status_pub = rospy.Publisher("/grasp_status", String, queue_size=1)

        calibration_file = cfg["arm"]["calibration_file"]
        with open(os.path.expanduser(calibration_file), "r", encoding="utf-8") as f:
            values = f.read().split()
        if len(values) < 4:
            raise RuntimeError("calibration file must contain four numbers")
        self.x_kb = [float(values[0]), float(values[1])]
        self.y_kb = [float(values[2]), float(values[3])]

    def pose(self, name: str) -> List[float]:
        return self.cfg["arm"][name]

    def home(self):
        self.interface.set_pose(*self.pose("home"))

    def ready(self):
        self.interface.set_pose(*self.pose("ready"))

    def reset(self):
        self.interface.reset_status()

    def grasp_any(self, wanted: Iterable[str]) -> Optional[str]:
        detections = self.vision.detect_items(wanted)
        if not detections:
            rospy.logwarn("No wanted item detected: %s", list(wanted))
            self.grasp_status_pub.publish(String("1"))
            return None

        item_name, _item_id, image_x, image_y = detections[0]
        arm_x = self.x_kb[0] * image_y + self.x_kb[1]
        arm_y = self.y_kb[0] * image_x + self.y_kb[1]
        grasp_z = float(self.cfg["arm"]["grasp_z"])
        pre_lift = float(self.cfg["arm"]["pre_grasp_lift"])
        post_lift = float(self.cfg["arm"]["post_grasp_lift"])

        rospy.loginfo(
            "Grasp target %s at image=(%.1f, %.1f), arm=(%.1f, %.1f, %.1f)",
            item_name,
            image_x,
            image_y,
            arm_x,
            arm_y,
            grasp_z,
        )

        self.interface.set_pose(arm_x, arm_y, grasp_z + pre_lift)
        self.interface.set_pump(True)
        self.interface.set_pose(arm_x, arm_y, grasp_z)
        self.interface.set_pose(arm_x, arm_y, grasp_z + post_lift)

        self.grasp_status_pub.publish(String("0"))
        return item_name

    def drop(self):
        self.interface.set_pose(*self.pose("drop"))
        self.interface.set_pump(False)
        self.ready()
        self.grasp_status_pub.publish(String("0"))


class RobotMoveAction:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.nav_timeout_sec = float(cfg["task"]["nav_timeout_sec"])

        self.move_action_cli = actionlib.SimpleActionClient(
            "move_straight", MoveStraightDistanceAction
        )
        self.move_action_cli.wait_for_server(timeout=rospy.Duration.from_sec(3))

        self.turn_action_cli = actionlib.SimpleActionClient(
            "turn_body", TurnBodyDegreeAction
        )
        self.turn_action_cli.wait_for_server(timeout=rospy.Duration.from_sec(3))

        self.goto_local_pub = rospy.Publisher("mark_nav", String, queue_size=1)

    def goto_mark(self, mark_name: str) -> bool:
        rospy.loginfo("Going to mark: %s", mark_name)
        self.goto_local_pub.publish(String("go " + mark_name))

        try:
            status = rospy.wait_for_message(
                "move_base/result",
                MoveBaseActionResult,
                rospy.Duration(self.nav_timeout_sec),
            ).status.status
        except Exception:
            rospy.logwarn("Navigation timeout: %s", mark_name)
            status = GoalStatus.ABORTED

        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Arrived at mark: %s", mark_name)
            return True

        rospy.Publisher("move_base/cancel", GoalID, queue_size=1).publish(
            GoalID(stamp=rospy.Time.from_sec(0.0), id="")
        )
        rospy.logwarn("Navigation failed: %s, status=%s", mark_name, status)
        return False

    def move_straight(self, distance: float, vel: float = 0.1) -> bool:
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                const_rot_vel=vel,
                move_distance=abs(distance),
            ),
            rospy.Duration.from_sec(5),
        )
        return True

    def leave_start(self):
        distance = float(self.cfg["marks"]["start_leave_distance"])
        self.move_straight(distance, vel=0.1)

    def step_back(self):
        self.move_straight(0.2, vel=-0.1)


class AutoMatch2026Reference:
    def __init__(self):
        rospy.init_node("auto_match_2026_reference", anonymous=True)

        rospack = rospkg.RosPack()
        package_path = rospack.get_path("auto_match")
        default_cfg = os.path.join(package_path, "config", "task_2026_reference.yaml")
        cfg_path = rospy.get_param("~config", default_cfg)
        self.cfg = load_yaml(cfg_path)

        self.vision = VisionClient(
            self.cfg["items"], float(self.cfg["task"]["detect_timeout_sec"])
        )
        self.arm = ArmAction(self.cfg, self.vision)
        self.robot = RobotMoveAction(self.cfg)
        self.stop_flag = False
        self.task_thread = threading.Thread(target=lambda: None)

        start_topic = self.cfg["task"]["wait_for_start_topic"]
        self.start_sub = rospy.Subscriber(start_topic, String, self.start_cb)

        rospy.loginfo("auto_match_2026_reference is ready")
        rospy.loginfo("Send std_msgs/String to %s to start/stop", start_topic)

    def read_board_mapping(self) -> Dict[str, str]:
        """Return item -> zone mapping.

        Replace this function with real classification-board recognition.
        The expected output is like {"toy": "B", "clock": "C", "banana": "D"}.
        """
        mapping = dict(self.cfg["classification_mapping"])
        valid_zones = {"B", "C", "D"}
        for item in ("toy", "clock", "banana"):
            zone = mapping.get(item)
            if zone not in valid_zones:
                raise RuntimeError(f"invalid mapping for {item}: {zone}")
        return mapping

    def drop_item_to_zone(self, item_name: str, zone: str) -> bool:
        mark = self.cfg["marks"]["drop_zones"][zone]
        if not self.robot.goto_mark(mark):
            return False
        rospy.sleep(1)
        self.arm.drop()
        self.robot.step_back()
        rospy.loginfo("Dropped %s to zone %s", item_name, zone)
        return True

    def try_pick_from_points(self, wanted: Iterable[str]) -> Optional[str]:
        for mark in self.cfg["marks"]["pickup_points"]:
            if self.stop_flag:
                return None
            if not self.robot.goto_mark(mark):
                continue
            rospy.sleep(1)
            item = self.arm.grasp_any(wanted)
            self.robot.step_back()
            if item is not None:
                return item
        return None

    def run_task(self):
        self.stop_flag = False
        self.arm.home()

        rospy.loginfo("2026 reference task start")
        self.robot.leave_start()
        if self.stop_flag:
            return

        self.arm.reset()
        self.arm.ready()

        board_mark = self.cfg["marks"]["board"]
        if not self.robot.goto_mark(board_mark):
            rospy.logerr("Cannot reach classification board")
            return

        mapping = self.read_board_mapping()
        rospy.loginfo("Board mapping: %s", mapping)

        # Rule: cake must be placed in A first, otherwise cake scores zero.
        cake = self.try_pick_from_points(["cake"])
        if cake == "cake":
            self.drop_item_to_zone("cake", "A")
        else:
            rospy.logwarn("Cake was not found. Continuing with normal blocks.")

        max_blocks = int(self.cfg["task"]["max_blocks"])
        placed = 1 if cake == "cake" else 0
        wanted_normal = ["toy", "clock", "banana"]

        while not self.stop_flag and placed < max_blocks:
            item = self.try_pick_from_points(wanted_normal)
            if item is None:
                rospy.logwarn("No more normal item found, task finished")
                break

            zone = mapping[item]
            if self.drop_item_to_zone(item, zone):
                placed += 1
            else:
                rospy.logerr("Drop failed for %s, releasing pump for safety", item)
                self.arm.interface.set_pump(False)

        self.arm.home()
        rospy.logwarn("2026 reference task finished, placed=%d", placed)

    def start_cb(self, _msg: String):
        if not self.task_thread.is_alive():
            self.task_thread = threading.Thread(target=self.run_task)
            self.task_thread.start()
            return

        rospy.logwarn("Task is running, requesting stop")
        self.stop_flag = True


if __name__ == "__main__":
    try:
        AutoMatch2026Reference()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("Fatal error: %s", exc)
        sys.exit(1)

