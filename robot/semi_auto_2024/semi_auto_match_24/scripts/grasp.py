#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import random
import rospy
import string
import math
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from spark_carry_object.msg import *
from swiftpro.msg import *


class GraspObject():

    def __init__(self):
        '''
        初始化参数
        x & y 代表水源的中心点位置信息
        x_prev & y_prev 代表前一轮记录的中心点
        found_count 代表识别中心点的次数
        self.found_*** 代表是否准确找到水源的中心点
        '''

        # 定义水源参数
        self.water_found_count = 0 ; self.found_water = False 
        self.water_x = 0 ; self.water_y = 0 ; self.water_x_prev = 0 ; self.water_y_prev = 0
        
        # 获取标定文件数据
        filename = os.environ['HOME'] + "/thefile.txt"
        with open(filename, 'r') as f:
            s = f.read()
        arr = s.split()
        self.x_kb = [float(arr[0]), float(arr[1])]
        self.y_kb = [float(arr[2]), float(arr[3])]
        rospy.logwarn('X axia k and b value: ' + str(self.x_kb))
        rospy.logwarn('X axia k and b value: ' + str(self.y_kb))

        # 发布机械臂位姿
        self.pub1 = rospy.Publisher('position_write_topic', position, queue_size=10)
        # 发布机械臂吸盘
        self.pub2 = rospy.Publisher('pump_topic', status, queue_size=1)
        # 发布TWist消息控制机器人底盘
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # 订阅机械臂抓取指令
        self.sub2 = rospy.Subscriber('/grasp', String, self.grasp_cp, queue_size=1)
        # 发布机械臂恢复状态指令
        self.arm_status_pub = rospy.Publisher('/swiftpro_status_topic', status, queue_size=1)
        # 订阅摄像头话题,接收图像信息后跳转image_cb进行处理
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb, queue_size=1)


        # 发布信息让机械臂到指定位置
        self.arm_position_reset()
        pos = position()
        pos.x = 20
        pos.y = 150
        pos.z = 35
        self.pub1.publish(pos)


    # 对传入的一帧图像进行处理，使用CV检测物体       
    def image_cb(self, data):
        # 将 ROS image消息类型转换成opencv类型  
        try:
            cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
            # print(cv_image1.shape)
        except CvBridgeError as e:
            print('error')
        # 由RGB颜色转换成HSV颜色空间
        cv_image2 = cv2.cvtColor(cv_image1, cv2.COLOR_BGR2HSV)
        # 蓝色物体颜色检测范围
        LowerBlue = np.array([95, 90, 80])
        UpperBlue = np.array([130, 255, 255])
        # 阈值处理
        mask = cv2.inRange(cv_image2, LowerBlue, UpperBlue)
        # 位运算，对图像进行掩膜处理
        cv_image3 = cv2.bitwise_and(cv_image2, cv_image2, mask=mask)
        # 取三维矩阵中第一维的所有数据
        cv_image4 = cv_image3[:, :, 0]
        # 均值滤波处理
        blurred = cv2.blur(cv_image4, (9, 9))
        # 简单阈值函数
        (_, thresh) = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY)
        # 获取结构元素
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
        # 执行高级形态变换
        cv_image5 = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        # 腐蚀处理
        cv_image5 = cv2.erode(cv_image5, None, iterations=4)
        # 扩张处理
        cv_image5 = cv2.dilate(cv_image5, None, iterations=4) 
        # 检测轮廓
        contours, hier = cv2.findContours(cv_image5, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(cv_image1, contours, 0, (0, 255, 0), 2)
        # 显示图像
        

        # 根据检测的轮廓信息，找到其中距离最近的水源
        # len(contours) 识别到的水源的个数
        if len(contours) > 0:
            dis = []    
            dis_min = 640          
            # enumerate()为迭代的对象添加序列号
            for i, c in enumerate(contours):
                # 绘制水源图形
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                # 计算水源中心点
                x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
                y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4
                # w = math.sqrt((box[0][0] - box[1][0]) ** 2 + (box[0][1] - box[1][1]) ** 2)
                # h = math.sqrt((box[0][0] - box[3][0]) ** 2 + (box[0][1] - box[3][1]) ** 2)
                distance =  math.sqrt((x_mid-320)**2+(480-y_mid)**2)
                # 找出距离最近的水源
                dis.append(distance)
                if dis[i] < dis_min:    
                    dis_min = dis[i]
                    self.water_x = x_mid
                    self.water_y = y_mid
            
            # 基于有可能出现在移动或旋转过程中识别到水源的情况
            # 设定found_count参数判断水源有无移动
            # 当found_count次数达到30时，认为水源没有移动，可以进行定位           
            if self.water_found_count >= 30:
                # 将找到物体的参数设定为True
                self.found_water = True                
            else:
                # 判断识别的水源中心点位置有无移动
                if abs(self.water_x - self.water_x_prev) <= 4 and abs(self.water_y - self.water_y_prev) <= 4:
                    self.water_found_count = self.water_found_count + 1
                else:
                    # 一旦移动，归零
                    self.water_found_count = 0
        else:
            # 没有找到轮廓，判断水源移动参数归零
            self.water_found_count = 0

        self.water_x_prev = self.water_x
        self.water_y_prev = self.water_y
        
        cv2.circle(cv_image1, (int(self.water_x), int(self.water_y)), 5, (0, 0, 255), -1)

        cv2.imshow("contours",cv_image1)
        cv2.waitKey(1)

    def grasp_cp(self, msg):
        # 设置睡眠时间
        rate = rospy.Rate(1)
        # 接受到进行抓取水源的信号
        if msg.data == '0':            
            # 若未能获取水源准确中心点进入此循环
            # self.arm_position_reset()
            while not self.found_water:
                rospy.logwarn('finding water' )
                rate.sleep()
            # 抓取检测到的物体    
            print("water grasp on!!!!!!")
            self.grasp(self.water_x,self.water_y)
            self.found_water = False

        # 接受到进行释放的信号
        else:
            height = int(msg.data)
            self.release_object(height)
            rate.sleep()

    # 执行抓取
    def grasp(self,y,x):        
        print("start to grasp\n")
        # 设定睡眠时间
        r1 = rospy.Rate(0.5)
        r2 = rospy.Rate(10)
        
        # 物体所在坐标+标定误差
        # 控制机械臂到目标上方
        pos = position()
        pos.x = self.x_kb[0] * x + self.x_kb[1]
        pos.y = self.y_kb[0] * y + self.y_kb[1]
        pos.z = 20
        print("pos.x=",pos.x,"pos.y=",pos.y,"pos.z=",pos.z)
        self.pub1.publish(pos)
        rospy.sleep(2)

        # 控制机械臂向下运动 
        pos.z = -55
        self.pub1.publish(pos)
        rospy.sleep(2)

        # 开始吸取物体
        self.pub2.publish(1)
        rospy.sleep(1)

        # 提起物体
        pos.x = 230  
        pos.y = 0
        pos.z = 160  
        self.pub1.publish(pos)
        rospy.sleep(1)

    # 释放物体
    def release_object(self,height):
        r1 = rospy.Rate(1)     # 1s
        pos = position()
        # 设定第一层位置    
        
        if height == 1:
            pos.x = 220
            pos.y = 0
            pos.z = -40  
            self.pub1.publish(pos)
            r1.sleep()

        # stop pump
        self.pub2.publish(0)
        r1.sleep()
        
        self.arm_to_home()
    
    def arm_to_home(self):
        pos = position()
        pos.x = 20;pos.y = 150;pos.z=60    
        self.pub1.publish(pos)
        rospy.sleep(1)


    def arm_position_reset(self):
        print("arm reset\n")
        r1 = rospy.Rate(1)
        self.arm_status_pub.publish(0)
        r1.sleep()
        self.arm_status_pub.publish(1)
        r1.sleep()


        
if __name__ == '__main__':
    try:
        rospy.init_node('GraspObject', anonymous=False)
        rospy.loginfo("Init GraspObject main")   
        GraspObject()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("End spark GraspObject main")

