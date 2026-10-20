#! /usr/bin/env python

import rospy
#from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import common.msg

def move_straight_client(distance=0.2,vel = 0.1):
    # Creates the SimpleActionClient, passing the type of the action
    # (MoveStraightDistanceAction) to the constructor.
    client_m = actionlib.SimpleActionClient('move_straight', common.msg.MoveStraightDistanceAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client_m.wait_for_server()

    # Creates a goal to send to the action server.
    goal = common.msg.MoveStraightDistanceGoal(move_distance = distance,const_rot_vel = vel)

    # Sends the goal to the action server.
    client_m.send_goal(goal)

    # Waits for the server to finish performing the action.
    client_m.wait_for_result()  #rospy.Duration.from_sec(5.0)

    # Prints out the result of executing the action
    return client_m.get_result()  # A FibonacciResult

def turn_body_client(degree,vel = 1.1):
    # Creates the SimpleActionClient, passing the type of the action
    # (TurnBodyDegreeAction) to the constructor.
    client_t = actionlib.SimpleActionClient('turn_body', common.msg.TurnBodyDegreeAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client_t.wait_for_server()

    # Creates a goal to send to the action server.
    goal = common.msg.TurnBodyDegreeGoal(goal_degree = degree,const_rot_vel = vel)

    # Sends the goal to the action server.
    client_t.send_goal(goal)

    # Waits for the server to finish performing the action.
    client_t.wait_for_result()  #rospy.Duration.from_sec(5.0)

    # Prints out the result of executing the action
    return client_t.get_result()  # A FibonacciResult
if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('move_Py')
        result = move_straight_client(0.2,-0.1)
        print("Result:", result.result)
        turn_body_client(90,1.1)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", sys.stderr)
