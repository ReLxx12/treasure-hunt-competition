#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
import tf.transformations as tf_transformations
from visualization_msgs.msg import Marker
from math import radians, pi
import numpy as np
import yaml
import rospkg
import os
import pickle

class MarkNav():
    def __init__(self):
        # 初始化节点
        rospy.init_node('MarkNav')
        # 发布TWist消息控制机器人
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # 等待move_base服务器建立
        self.move_base.wait_for_server(rospy.Duration(20))
        print('ready')
        # 订阅标记事件
        rospy.Subscriber('mark_nav', String, self.mark_nav)
        # 定义标记地点字典
        global dict_mark
        dict_mark = {}
        # 监听TF坐标
        self.listener = tf.TransformListener()
        rospy.sleep(1) # need to delay
    
    def mark_nav(self,mark_name):
        '''
        设定标定地点，与后面导航相关，采用字典的方式存放
        建议标记地点名字,分类区、分拣区、收取区
        '''

        rospack = rospkg.RosPack() 
        mark_map_path = os.path.join(rospack.get_path('auto_match'),'config', 'mark_map.pkl')  # 标记地图yaml文件地址

        # 转化成字符型变量
        mark_name = str(mark_name)
        # 结束地点标记
        if str.find(mark_name,str("finish"))!= -1:
            # 将字典信息写入 pkl 文件
            with open(mark_map_path, 'wb') as file:
                pickle.dump(dict_mark, file)
            print("已保存pkl文件,文件保存在:",mark_map_path)
            print("当前字典内容：\n",dict_mark)


        # 前往地点    
        if str.find(mark_name,str("go"))!= -1:
            # 从文件中加载数据
            with open(mark_map_path, 'rb') as file:
                self.mark_map = pickle.load(file)
            # 输出加载的数据
            print("mark_dict=",self.mark_map)
            # 提取mark名称
            mark_name = (mark_name.split())[2]
            # 判断地点位置是否在地点字典中
            if mark_name in self.mark_map:
                self.navigation(mark_name)
            else:
                print("此地点尚未登陆在字典中")


        # 学习地点
        if str.find(mark_name,str("learn"))!= -1:
            # 提取mark名称
            mark_name = (mark_name.split())[2]
            # 获取当前位置
            self.currrent_position = self.get_currect_pose()
            # 标记当前位置
            dict_mark[mark_name] = self.currrent_position
            print("当前字典内容：\n",dict_mark)

    # 获取当前位置
    def get_currect_pose(self):
        (target_trans, target_rot) = self.listener.lookupTransform("map", "base_footprint", rospy.Time(0))
        print("target_trans:",target_trans)
        print("target_rot:",target_rot)
        return tf_transformations.compose_matrix(translate=target_trans, angles=tf_transformations.euler_from_quaternion(target_rot))

    def navigation(self,mark_name):
        '''
        根据地点进行导航
        '''
        print("start navigation")
        # movebase初始化
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        # 设定目标地点
        self.drop_position = self.mark_map[mark_name]
        tran = tf_transformations.translation_from_matrix(self.drop_position)
        quat = tf_transformations.quaternion_from_matrix(self.drop_position)
        pose = Pose(Point(tran[0], tran[1], tran[2]), Quaternion(quat[0], quat[1], quat[2], quat[3]))
        goal.target_pose.pose = pose
        # 把目标位置发送给MoveBaseAction的服务器
        self.move_base.send_goal(goal)

if __name__ == '__main__':
    try:
        MarkNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mark_move finished.")
