#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
import cv2
import numpy as np
import math
import _thread
import yaml
import std_msgs
# from std_msgs.msg import String
from sensor_msgs.msg import Image
from swiftpro.msg import *
from cv_bridge import CvBridge, CvBridgeError
from sklearn.linear_model import LinearRegression

xc = 0
yc = 0

# count = 20
# index = 0

# xarray = np.zeros(count)
# yarray = np.zeros(count)
# xc_array = np.zeros(count)
# yc_array = np.zeros(count)

cali_w = 20
cali_h = 30
collect_times = 30
move_position = 1
collecting = 0
c_cnt = 0
HSV_value = [0, 0, 0]


def mean_hsv(img, hsv_value):
    HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_value[0] += np.mean(HSV[:, :, 0])
    hsv_value[1] += np.mean(HSV[:, :, 1])
    hsv_value[2] += np.mean(HSV[:, :, 2])
    return hsv_value


def hsv_range(hsv_value):

    H_range = 6
    S_range = 120
    V_range = 120

    lower_H = int(hsv_value[0] - H_range)
    upper_H = int(hsv_value[0] + H_range)

    lower_S = int(hsv_value[1] - S_range)
    upper_S = int(hsv_value[1] + S_range)

    lower_V = int(hsv_value[2] - V_range)
    upper_V = int(hsv_value[2] + V_range)

    if lower_H < 0:
        lower_H = 0
    if upper_H > 180:
        upper_H = 180

    if lower_S < 50:
        lower_S = 50
    if upper_S > 255:
        upper_S = 255

    if lower_V < 50:
        lower_V = 50
    if upper_V > 255:
        upper_V = 255

    lower_HSV = np.array([lower_H, lower_S, lower_V])
    upper_HSV = np.array([upper_H, upper_S, upper_V])
    return lower_HSV, upper_HSV


def image_CB(data):
    global xc, yc
    global collecting
    global c_cnt
    global HSV_value
    global lower_HSV, upper_HSV
    # change to opencv
    try:
        cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    cv_image_draw = cv_image1.copy()
    if(move_position):
        cv2.putText(cv_image_draw, 'please move the camera to make ', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(cv_image_draw, 'the red color in the rectangle ', (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(cv_image_draw, 'green box!then press the \'ENTER\' ', (30, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(cv_image_draw, 'in another terminal!', (30, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2, cv2.LINE_AA)
        cv2.rectangle(cv_image_draw, (355, 310),
                      (355 + cali_w, 310 + cali_h), (0, 255, 0), 3)
        c_cnt = 0
        cv2.imshow("win_draw", cv_image_draw)
        cv2.waitKey(1)
        return
    elif(collecting):
        c_cnt = c_cnt+1
        cv2.putText(cv_image_draw, 'collecting the hsv 30 time!', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    (0, 255, 0), 2, cv2.LINE_AA)
        # cv2.putText(cv_image_draw, ' please wait for 5s.', (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
        #             (0, 255, 0), 2, cv2.LINE_AA)
        cv2.rectangle(cv_image_draw, (355, 310),
                      (355 + cali_w, 310 + cali_h), (0, 255, 0), 3)
        frame = cv_image1[315:310 + cali_h-5, 360:355 + cali_w-5]
        HSV_value = mean_hsv(frame, HSV_value)
        cv2.imshow("win_draw", cv_image_draw)
        cv2.waitKey(1)

        if(c_cnt >= collect_times):
            for i in range(len(HSV_value)):
                HSV_value[i] = HSV_value[i] / collect_times
                print(i, len(HSV_value), HSV_value[i])
            lower_HSV, upper_HSV = hsv_range(HSV_value)
            #save_hsv(name, lower_HSV, upper_HSV)
            print(lower_HSV, upper_HSV)
            collecting = 0
            cv2.destroyWindow("win_draw")
        return
    #test(lower_HSV, upper_HSV, cv_image1)

    cv_image_cp = cv_image1.copy()
    cv_image_hsv = cv2.cvtColor(cv_image_cp, cv2.COLOR_BGR2HSV)
    cv_image_gray = cv2.inRange(cv_image_hsv, lower_HSV, upper_HSV)
    # smooth and clean noise
    cv_image_gray = cv2.erode(cv_image_gray, None, iterations=2)
    cv_image_gray = cv2.dilate(cv_image_gray, None, iterations=2)
    cv_image_gray = cv2.GaussianBlur(cv_image_gray, (5, 5), 0)
    # detect contour
    cv2.imshow("win1", cv_image1)
    cv2.imshow("win2", cv_image_gray)
    cv2.waitKey(1)
    contours, _ = cv2.findContours(
        cv_image_gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    size = []
    size_max = 0
    for i, c in enumerate(contours):
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
        y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4
        w = math.sqrt((box[0][0] - box[1][0])**2 + (box[0][1] - box[1][1])**2)
        h = math.sqrt((box[0][0] - box[3][0])**2 + (box[0][1] - box[3][1])**2)
        size.append(w * h)
        if size[i] > size_max:
            size_max = size[i]
            index = i
            xc = x_mid
            yc = y_mid


def main(threadName, delay):
    global move_position
    global collecting
    global xc, yc

    count = 20
    xarray = np.zeros(count)
    yarray = np.zeros(count)
    xc_array = np.zeros(count)
    yc_array = np.zeros(count)

    r1 = rospy.Rate(1)  # 1s
    r2 = rospy.Rate(0.05)  # 20s
    r3 = rospy.Rate(0.2)  # 5s
    r4 = rospy.Rate(0.4)  # 2.5s

    cam_sub = rospy.Subscriber("/camera/color/image_raw", Image, image_CB)
    arm_position_pub = rospy.Publisher(
        "position_write_topic", position, queue_size=1)

    cali_z_axis = rospy.get_param("~cali_z_axis", 32)
    filename = os.path.join(rospkg.RosPack().get_path('spark_hm_task_match'), 'config', 'cali_config.yaml')
    try:
        with open(filename, "r") as f:
            content = yaml.load(f.read())
    except Exception:
        rospy.logerr("can't not open calibration file: ", filename)
        sys.exit(1)
    if isinstance(content, type(None)):
        content = {}

    r1.sleep()
    pos = position()
    pos.x = 120
    pos.y = 0
    pos.z = 35
    arm_position_pub.publish(pos)
    r3.sleep()

    print("Waiting the \'start_cali\' msg to start calibration... ")
    rospy.wait_for_message("start_cali", std_msgs.msg.String)
    collecting = True
    move_position = False

    while (collecting):
        pass

    for index in range(count):
        pos.x = 180 + index * 5
        pos.y = 200 - index * 10
        pos.z = cali_z_axis
        arm_position_pub.publish(pos)
        r1.sleep()
        if index == 0:
            r2.sleep()
        else:
            r4.sleep()

        xc_array[index] = xc
        yc_array[index] = yc
        xarray[index] = pos.x
        yarray[index] = pos.y
        print(
            f"{index + 1:2d}/20,pose x,y: ({pos.x:3d},{pos.y:3d}) cam x,y: ({xc:3.2f},{yc:3.2f})")

        # reshape to 2D array for linear regression
        xc_array = xc_array.reshape(-1, 1)
        yc_array = yc_array.reshape(-1, 1)
        xarray = xarray.reshape(-1, 1)
        yarray = yarray.reshape(-1, 1)

    Reg_x_yc = LinearRegression().fit(yc_array, xarray)
    Reg_y_xc = LinearRegression().fit(xc_array, yarray)
    k1 = Reg_x_yc.coef_[0][0]
    b1 = Reg_x_yc.intercept_[0]
    k2 = Reg_y_xc.coef_[0][0]
    b2 = Reg_y_xc.intercept_[0]

    if cali_z_axis in content.keys():
        content[cali_z_axis]['k1'] = float(k1)
        content[cali_z_axis]['b1'] = float(b1)
        content[cali_z_axis]['k2'] = float(k2)
        content[cali_z_axis]['b2'] = float(b2)
    else:
        content[cali_z_axis] = {
            'k1': float(k1),
            'b1': float(b1),
            'k2': float(k2),
            'b2': float(b2)
        }

    try:
        with open(filename, "w") as f:
            yaml.dump(content, f)
    except:
        rospy.logerr("can't not open calibration file: ", filename)

    pos.x = 120
    pos.y = 0
    pos.z = 35
    arm_position_pub.publish(pos)
    r3.sleep()

    print(f"Linear Regression for x and yc is :  x = {k1:.5f}yc + ({b1:.5f})")
    print(f"Linear Regression for y and yc is :  y = {k2:.5f}xc + ({b2:.5f})")
    print("finish the calibration. please check the value. then press ctrl-c to exit")


if __name__ == '__main__':
    rospy.init_node('arm_calibration', anonymous=True)
    _thread.start_new_thread(main, ("Thread-1", 2, ) )
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!!!")
