#! /usr/bin/env python
# -*- coding: UTF-8 -*-
import sys
import numpy as np
import rospy
#from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import common.msg
import common.srv

# distance 为直线走的距离
# vel 为线速度
# timeout 为超时时间
# block 为是否阻塞
# useLidar 为是否使用雷达测距, True为使用雷达测距, false为使用ODOM

def move_straight_client(distance, vel=0.1, timeout=30, block=True, useLidar=True):
    # Creates the SimpleActionClient, passing the type of the action
    # (MoveStraightDistanceAction) to the constructor.
    client_m = actionlib.SimpleActionClient('move_straight', common.msg.MoveStraightDistanceAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client_m.wait_for_server()

    # Creates a goal to send to the action server.
    goal = common.msg.MoveStraightDistanceGoal(
        type = common.msg.MoveStraightDistanceGoal.TYPE_SCAN if useLidar else common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
        move_distance = distance,
        const_rot_vel = vel
    )
    # goal.type = 1

    # Sends the goal to the action server.
    if (block): client_m.send_goal_and_wait(goal,rospy.Duration.from_sec(timeout))
    else: client_m.send_goal(goal)
    
    # Waits for the server to finish performing the action.
    #client_m.wait_for_result(rospy.Duration.from_sec(timeout))  #rospy.Duration.from_sec(5.0)

    rospy.sleep(1)

    # Prints out the result of executing the action
    return client_m.get_result()  # A FibonacciResult

# degree 为转动的角度
# vel 为转动速度
# timeout 为超时时间
# block 为是否阻塞
# is_const 为是否匀速转动

def turn_body_client(degree, vel=0.25, timeout=30, block=True, is_const=True):
    # Creates the SimpleActionClient, passing the type of the action
    # (TurnBodyDegreeAction) to the constructor.
    client_t = actionlib.SimpleActionClient('turn_body', common.msg.TurnBodyDegreeAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client_t.wait_for_server()

    if(degree > 180):
        degree -= 360
    elif(degree < -180):
        degree += 360

    # Creates a goal to send to the action server.
    goal = common.msg.TurnBodyDegreeGoal(
        is_const_vel = is_const,
        goal_degree = degree,
        const_rot_vel = vel
    )

    # Sends the goal to the action server.
    if (block): client_t.send_goal_and_wait(goal,rospy.Duration.from_sec(timeout))
    else: client_t.send_goal(goal)

    # Waits for the server to finish performing the action.
    # client_t.wait_for_result(rospy.Duration.from_sec(timeout))  #rospy.Duration.from_sec(5.0)

    rospy.sleep(1)

    # Prints out the result of executing the action
    return client_t.get_result()  # A FibonacciResult

# 移动到雷达检测到的距离
# e.g.: move_to_lidar_distance_client(distance=1, move_vel=0.1)
#       以 0.1/s 速度向前走直到雷达检测到前方点距离为 1米
# distance 正为检测前方距离，负为检测后方距离
# move_vel 运动速度, 正是以前方为正方向，负是以后方为正方向

def move_to_lidar_distance_client(distance, move_vel=0.1):
    # Wait for the "get_distance" service
    rospy.wait_for_service('/get_distance')

    # Creates the ServiceProxy
    service_proxy = rospy.ServiceProxy('/get_distance', common.srv.GetFrontBackDistance)

    while(True):
        # Judge forward or backward and request lidar distance
        if distance > 0:
            lidar_distance = service_proxy(None).front_distance
        else:
            lidar_distance = service_proxy(None).back_distance

        # Calculate the point difference between 
        # the actual distance and the target distance
        lidar_distance_diff = lidar_distance - abs(distance)

        # If the difference is within the acceptable range, 
        # we think the robot is in place
        if abs(lidar_distance_diff) < 0.05:
            break

        # Otherwise, move the robot to the target distance
        move_straight_client(
            distance=abs(lidar_distance_diff),
            timeout=abs(lidar_distance_diff / move_vel),
            vel=np.sign(lidar_distance_diff) * np.sign(distance) * abs(move_vel)
        )
    rospy.sleep(1)
    return True



if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('move_Py')
        move_straight_client(distance=0.25, vel=0.05, timeout=10) # 修改此参数，可以改变直行的距离和速度。
        # print("Result:", result.result)
        turn_body_client(degree=90, vel=0.2)  # 修改此参数，可以改变转动的角度
        # move_to_lidar_distance_client(1) # 以 0.1/s 速度向前走直到雷达检测到前方点距离为 1米
    except rospy.ROSInterruptException:
        print("program interrupted before completion", sys.stderr)
