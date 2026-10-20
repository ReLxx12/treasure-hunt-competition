#!/usr/bin/env python3

import os
import sys
import unittest
from unittest import mock


NODES_DIR = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "nodes"))
if NODES_DIR not in sys.path:
    sys.path.insert(0, NODES_DIR)

from runtask import AutoAction


class FakeArm:
    def __init__(self, reachable=True, unreachable_x=None):
        self.reachable = reachable
        self.unreachable_x = unreachable_x
        self.observation_moves = 0

    def drop_target_reachable(self, x, y, stack_count):
        if self.unreachable_x is not None and x == self.unreachable_x:
            return False
        return self.reachable

    def arm_grasp_ready(self):
        self.observation_moves += 1


class DualStackSelectionTest(unittest.TestCase):
    def make_task(self):
        task = object.__new__(AutoAction)
        task.main_stack_x = 300.0
        task.main_stack_y = -30.0
        task.fallback_stack_x = 100.0
        task.fallback_stack_y = 0.0
        task.zone_capacities = {"Collection_B": 3}
        task.max_scoring_items = 9
        task.arm = FakeArm()
        task.task_guard_stop = lambda: False
        return task

    def test_first_block_uses_main_first_layer_without_vision(self):
        task = self.make_task()
        target, reason = task.select_scoring_drop_target(
            "Collection_B", {"main_count": 0, "fallback_count": 0})
        self.assertIsNone(reason)
        self.assertEqual(target["stack"], "main")
        self.assertEqual(target["stack_count"], 0)
        self.assertEqual(target["x"], 300.0)
        self.assertEqual(target["y"], -30.0)
        self.assertEqual(task.arm.observation_moves, 0)

    def test_uncalibrated_vision_continues_fixed_main_stack(self):
        task = self.make_task()
        target, reason = task.select_scoring_drop_target(
            "Collection_B", {"main_count": 1, "fallback_count": 1})
        self.assertIsNone(reason)
        self.assertEqual(target["stack"], "main")
        self.assertEqual(target["stack_count"], 1)
        self.assertEqual(target["x"], 300.0)
        self.assertEqual(target["y"], -30.0)
        self.assertEqual(task.arm.observation_moves, 0)

    def test_uncalibrated_vision_continues_fixed_main_third_layer(self):
        task = self.make_task()
        target, reason = task.select_scoring_drop_target(
            "Collection_B", {"main_count": 2, "fallback_count": 0})
        self.assertIsNone(reason)
        self.assertEqual(target["stack"], "main")
        self.assertEqual(target["stack_count"], 2)
        self.assertEqual(target["x"], 300.0)
        self.assertEqual(target["y"], -30.0)

    def test_unreachable_fixed_main_is_rejected(self):
        task = self.make_task()
        task.arm = FakeArm(unreachable_x=300.0)
        target, reason = task.select_scoring_drop_target(
            "Collection_B", {"main_count": 1, "fallback_count": 0})
        self.assertIsNone(target)
        self.assertEqual(reason, "main_drop_pose_unreachable")

if __name__ == "__main__":
    unittest.main()
