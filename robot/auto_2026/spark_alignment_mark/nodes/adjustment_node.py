#!/usr/bin/python3

import rospy
# import numpy as np
import math
import cmath
import actionlib
from actionlib_msgs.msg import GoalStatus
from spark_alignment_mark.msg import TagAdjustmentAction, TagAdjustmentGoal, TagAdjustmentFeedback, TagAdjustmentResult
from ar_track_alvar_msgs.msg import AlvarMarkers
import common.msg
import common.srv


class SparkAdjustment:
    def __init__(self):
        self.left_id = 21
        self.center_id = 22
        self.right_id = 23
        # Creates the SimpleActionClient, passing the type of the action
        # (MoveStraightDistanceAction) to the constructor.
        self.move_action_cli = actionlib.SimpleActionClient(
            'move_straight', common.msg.MoveStraightDistanceAction)

        # Waits until the action server has started up and started
        # listening for goals.
        self.move_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))

        # Creates the SimpleActionClient, passing the type of the action
        # (TurnBodyDegreeAction) to the constructor.
        self.turn_action_cli = actionlib.SimpleActionClient(
            'turn_body', common.msg.TurnBodyDegreeAction)

        # Waits until the action server has started up and started
        # listening for goals.
        self.turn_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))

        self.action_server = actionlib.SimpleActionServer(
            'spark_alignment_mark', TagAdjustmentAction, self.execute, auto_start=False)
        self.action_server.register_preempt_callback(self.preemptCB)
        self.action_server.start()

    def __del__(self):
        pass

    def move_straight(self, distance, vel=0.1, timeout=30, block=True, useLidar=False):
        # Creates a goal to send to the action server.
        goal = common.msg.MoveStraightDistanceGoal(
            type=common.msg.MoveStraightDistanceGoal.TYPE_SCAN if useLidar else common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
            move_distance=distance,
            const_rot_vel=vel
        )

        # Sends the goal to the action server.
        if (block):
            self.move_action_cli.send_goal_and_wait(
                goal, rospy.Duration.from_sec(timeout))
        else:
            self.move_action_cli.send_goal(goal)

        rospy.sleep(0.5)
        # Prints out the result of executing the action
        return self.move_action_cli.get_result()  # A FibonacciResult

    def turn_body(self, degree, vel=0.25, timeout=30, block=True, is_const=True):
        if(degree > 180):
            degree -= 360
        elif(degree < -180):
            degree += 360

        # Creates a goal to send to the action server.
        goal = common.msg.TurnBodyDegreeGoal(
            is_const_vel=is_const,
            goal_degree=degree,
            const_rot_vel=vel
        )

        # Sends the goal to the action server.
        if (block):
            self.turn_action_cli.send_goal_and_wait(
                goal, rospy.Duration.from_sec(timeout))
        else:
            self.turn_action_cli.send_goal(goal)

        rospy.sleep(0.5)
        # Prints out the result of executing the action

        return self.turn_action_cli.get_result()  # A FibonacciResult

    def execute(self, goal: TagAdjustmentGoal):

        self.action_server.publish_feedback(
            TagAdjustmentFeedback(state=TagAdjustmentFeedback.RUNNING))
        markers = rospy.wait_for_message("marker_center", AlvarMarkers, timeout=1).markers
        if len(markers) > 1:
            id_list = [x.id for x in markers]

            # get the slope of the connection between two marker
            delta_x = markers[0].center.x - markers[1].center.x
            delta_y = (480 - markers[0].center.y) - \
                (480 - markers[1].center.y)  # Y-axis flip

            # get the rotation angle and make the spark face to the table
            line_degress = math.degrees(math.atan(delta_y/delta_x))

            # Facing the front
            if (goal.adjustment_type == TagAdjustmentGoal.TYPE_JUST_FACE):
                adj.turn_body(line_degress)

            # or more detailed adjustment
            else:
                # !!!!!!!!!!!!!!!!!!!!!!!!!!!!
                # This processing method is only used for demonstration
                # and cannot perfectly handle this problem
                # We encourage you to find new ways and re-implement
                # !!!!!!!!!!!!!!!!!!!!!!!!!!!!

                # get center point
                if self.center_id in id_list:
                    mid_x = markers[id_list.index(self.center_id)].center.x
                    mid_y = markers[id_list.index(self.center_id)].center.y
                else:
                    mid_x = (markers[0].center.x + markers[1].center.x) / 2
                    # Y-axis flip
                    mid_y = 480 - (markers[0].center.y +
                                   markers[1].center.y) / 2

                mid_p, mid_theta = cmath.polar(complex(mid_x - 320, mid_y))
                mid_theta = math.degrees(mid_theta)

                # get the angle of spark turning to parallel desktop
                if line_degress < 0:
                    turn_degress = line_degress + 90
                else:
                    turn_degress = line_degress - 90

                # Estimate the distance to walk
                walk_distance = 0.0016 * \
                    (math.sin(math.radians(mid_theta - turn_degress)) * mid_p)

                self.action_server.publish_feedback(TagAdjustmentFeedback(
                    state=TagAdjustmentFeedback.EXEC_TURNNING))
                adj.turn_body(turn_degress)

                self.action_server.publish_feedback(
                    TagAdjustmentFeedback(state=TagAdjustmentFeedback.EXEC_MOVING))
                adj.move_straight(walk_distance)

                self.action_server.publish_feedback(TagAdjustmentFeedback(
                    state=TagAdjustmentFeedback.EXEC_TURNNING))
                if line_degress < 0:
                    adj.turn_body(-90)
                else:
                    adj.turn_body(90)
            self.action_server.set_succeeded(TagAdjustmentResult(result=TagAdjustmentResult.SUCCESS))
        else:
            rospy.logwarn("no enough tag has been found")
            self.action_server.set_succeeded(TagAdjustmentResult(result=TagAdjustmentResult.ERROR))

    def preemptCB(self):
        self.turn_body(degree=0, vel=0.05)
        self.move_straight(distance=0, vel=0.1)


if __name__ == '__main__':
    rospy.init_node('spark_alignment_mark', anonymous=True)

    adj = SparkAdjustment()
    rospy.spin()
