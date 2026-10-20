#!/usr/bin/python3

import traceback
import numpy as np
import math
import rospy
import actionlib
from spark_alignment_mark.msg import TagAdjustmentAction, TagAdjustmentGoal, TagAdjustmentResult
from common.msg import MoveStraightDistanceAction, TurnBodyDegreeAction, MoveStraightDistanceGoal, TurnBodyDegreeGoal
import tf.transformations


class SparkActionMotionCtrl:
    def __init__(self, dead_distance=0.05) -> None:
        self.DEAD_DISTANCE = dead_distance

        # instantiated the SimpleActionClient class that action name is /move_straight
        self.move_action_cli = actionlib.SimpleActionClient(
            'move_straight', MoveStraightDistanceAction)
        self.move_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))

        # instantiated the SimpleActionClient class that action name is /turn_body
        self.turn_action_cli = actionlib.SimpleActionClient(
            'turn_body', TurnBodyDegreeAction)
        self.turn_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))

    def move_straight(self, distance, vel=0.1, timeout=30, block=True, useLidar=False):
        ''' 向前走指定值的距离

        param {float} distance: 距离长度
        param {float} vel: 速度, 单位为m/s
        param {int} timeout: 超时时间, 默认30s
        param {bool} block: 是否等待动作完成后才退出函数, 默认为True
        param {bool} useLidar: True使用雷达数据闭环, False使用里程计数据闭环
        return {FibonacciResult}: 如果完成返回结果, 否则返回None
        '''
        # Creates a goal to send to the action server.
        goal = MoveStraightDistanceGoal(
            type=MoveStraightDistanceGoal.TYPE_SCAN if useLidar else MoveStraightDistanceGoal.TYPE_ODOM,
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
        ''' 旋转指定值的角度

        param {float} degree: 旋转角度, 单位使用角度制
        param {float} vel: 速度, 单位为rad/s
        param {int} timeout: 超时时间, 默认30s
        param {bool} block: 是否等待动作完成后才退出函数, 默认为True
        param {bool} is_const: True使用加减速, False不使用
        return {FibonacciResult}: 如果完成返回结果, 否则返回None
        '''
        if(degree > 180):
            degree -= 360
        elif(degree < -180):
            degree += 360

        # Creates a goal to send to the action server.
        goal = TurnBodyDegreeGoal(
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

    def to_pose(self, px, py, angle):
        ''' 以当前位置为原点, 移动 Spark 到目标点去

        param {float} px: 目标点 X 轴的值
        param {float} py: 目标点 Y 轴的值
        param {float} angle: 目标点角度的值, 单位是角度制
        return {bool}: True 完成
        '''
        tmp_angle = math.degrees(math.atan2(np.sign(px)*py, np.sign(px)*px))
        mileage = math.sqrt(px**2 + py**2)

        if np.abs(mileage) > self.DEAD_DISTANCE:
            rospy.loginfo(f"target x={px} y={py} theta={angle}")
            self.turn_body(degree=tmp_angle)
            self.move_straight(distance=mileage, vel=np.sign(px) * 0.1,
                                timeout=mileage/0.1*1.25)
            self.turn_body(degree=angle-tmp_angle)
        else:
            self.turn_body(degree=angle)

        return True

    def stop(self):
        self.move_action_cli.cancel_all_goals()
        self.turn_action_cli.cancel_all_goals()


class ARTagTool:
    def __init__(self) -> None:
        self.listener = tf.TransformListener()

    def get_tag(self, id):
        ''' 获取指定 ID 值的 tag 相对于Spark的位置姿态

        param {int} id: tag 的 ID 值
        return {numpy.NDarray} : 返回 tag 的矩阵, 没找到返回 None
        '''

        ret_posture = None
        try:
            if (rospy.Time().now().secs - self.listener.getLatestCommonTime('/base_link', '/ar_marker_' + str(id)).secs <= 1):
                (target_trans, target_rot) = self.listener.lookupTransform(
                    '/base_link', '/ar_marker_' + str(id), rospy.Time(0))
                ret_posture = tf.transformations.compose_matrix(
                    translate=target_trans,
                    angles=tf.transformations.euler_from_quaternion(target_rot)
                )
                
        except Exception as e:
            rospy.logwarn(e)
            ret_posture = None

        return ret_posture
    
    def shift_matrix(self, matrix, aixs_x=0, aixs_y=0, aixs_z=0):
        ''' 位移变换矩阵的位置

        param {numpy.NDarray} matrix: 要平移的变换矩阵
        param {float} aixs_x: X 轴的值
        param {float} aixs_x: Y 轴的值
        param {float} aixs_z: Z 轴的值
        return {numpy.NDarray}: 同形状则返回平移后的矩阵, 否则返回None
        '''
        if matrix.shape == tf.transformations.identity_matrix().shape:
            return np.dot(matrix, tf.transformations.translation_matrix((aixs_x, aixs_y, aixs_z)))
        else:
            return None

    def rot_matrix_roll(self, matrix, theta):
        ''' 旋转变换矩阵的旋转矩阵部分, 只旋转roll

        param {numpy.NDarray} matrix: 要旋转的变换矩阵
        param {float} theta: 旋转的值, 单位使用弧度制
        return {numpy.NDarray}: 同形状则返回旋转后的变换矩阵, 否则返回None
        '''
        if matrix.shape == tf.transformations.identity_matrix().shape:
            return np.dot(matrix, tf.transformations.euler_matrix(theta, 0, 0))
        else:
            return None

    def rot_matrix_pitch(self, matrix, theta):
        ''' 旋转变换矩阵的旋转矩阵部分, 只旋转pitch

        param {numpy.NDarray} matrix: 要旋转的变换矩阵
        param {float} theta: 旋转的值, 单位使用弧度制
        return {numpy.NDarray}: 同形状则返回旋转后的变换矩阵, 否则返回None
        '''
        if matrix.shape == tf.transformations.identity_matrix().shape:
            return np.dot(matrix, tf.transformations.euler_matrix(0, theta, 0))
        else:
            return None

    def rot_matrix_yaw(self, matrix, theta):
        ''' 旋转变换矩阵的旋转矩阵部分, 只旋转yaw

        param {numpy.NDarray} matrix: 要旋转的变换矩阵
        param {float} theta: 旋转的值, 单位使用弧度制
        return {numpy.NDarray}: 同形状则返回旋转后的变换矩阵, 否则返回None
        '''
        if matrix.shape == tf.transformations.identity_matrix().shape:
            return np.dot(matrix, tf.transformations.euler_matrix(0, 0, theta))
        else:
            return None
        
    def matrix_interpolation(self, matrix1, matrix2):
        ''' 获取两个变换矩阵中间插值

        param {numpy.NDarray} matrix1: 变换矩阵1
        param {numpy.NDarray} matrix2: 变换矩阵2
        return {numpy.NDarray}: 插值的变换矩阵
        '''
        q1 = tf.transformations.quaternion_from_matrix(matrix1)
        q2 = tf.transformations.quaternion_from_matrix(matrix2)
        q_int = tf.transformations.quaternion_slerp(q1, q2, fraction=0.5)

        t1 = tf.transformations.translation_from_matrix(matrix1)
        t2 = tf.transformations.translation_from_matrix(matrix2)
        t_int = (t1 + t2) / 2
        
        return tf.transformations.compose_matrix(translate=t_int, angles=tf.transformations.euler_from_quaternion(q_int))


class SparkAdjustment:
    def __init__(self, left_id=21, center_id=22, right_id=23, tag_distance=0.2):
        ''' Spark 根据二维码调整自己姿态的方法
        使用 actionlib client 进行调用, 名为 /spark_alignment_mark

        param {int} left_id: 左侧id
        param {int} center_id: 中间id
        param {int} right_id: 右侧id
        param {float} tag_distance: 二维码之间的距离, 用于找不到中心二维码时估算使用
        '''
        self.LEFT_ID = left_id
        self.CENTER_ID = center_id
        self.RIGHT_ID = right_id
        self.TAG_DISTANCE = tag_distance

        # instantiated class
        self.spark_motion = SparkActionMotionCtrl()
        self.ar_tag = ARTagTool()

        # instantiate the tf publishing class
        self.broadcaster = tf.TransformBroadcaster()

        # initialize action server
        self.action_server = actionlib.SimpleActionServer(
            'spark_alignment_mark', TagAdjustmentAction, self.execute, auto_start=False)
        self.action_server.register_preempt_callback(self.preemptCB)
        self.action_server.start()

    def just_face(self, axis, negative=False):
        ''' 只让spark正对二维码

        param {int} axis: 正对二维码的哪个轴, 可选['x', 'y', 'z']
        param {int} negative: 是否与二维码的轴相反方向
        param {bool}: True为执行成功, False为执行失败
        '''

        # get three tag matrix
        tag_l = self.ar_tag.get_tag(self.LEFT_ID)
        tag_c = self.ar_tag.get_tag(self.CENTER_ID)
        tag_r = self.ar_tag.get_tag(self.RIGHT_ID)
        if tag_c is not None:
            M = tag_c
        elif tag_l is not None:
            M = tag_l
        elif tag_r is not None:
            M = tag_r
        else:
            rospy.logwarn("can not found tag.")
            return False
        
        # get target axis point on 2D matrix
        px = 0
        py = 0
        if axis == 'x':
            [px, py] = M[:, 0][:2]
        elif axis == 'y':
            [px, py] = M[:, 1][:2]
        elif axis == 'z':
            [px, py] = M[:, 2][:2]
        else:
            rospy.logerr("axis error! please check your code.")
            return False
        
        # get turn theta
        theta = math.atan2(py, px)
        if negative:
            theta += math.pi

        if theta > math.pi:
            theta -= 2 * math.pi
        elif theta < -math.pi:
            theta += 2 * math.pi

        # turn the spark
        self.spark_motion.turn_body(math.degrees(theta))
        return True


    def center_face(self, axis, negative=False, target_distance=0.4):
        ''' spark正对中间的二维码

        param {int} axis: 正对二维码的哪个轴, 可选['x', 'y', 'z']
        param {int} negative: 是否与二维码的轴相反方向
        param {int} target_distance: 二维码正对轴前的距离
        param {bool}: True为执行成功, False为执行失败
        '''

        # get three tag matrix
        tag_l = self.ar_tag.get_tag(self.LEFT_ID)
        tag_c = self.ar_tag.get_tag(self.CENTER_ID)
        tag_r = self.ar_tag.get_tag(self.RIGHT_ID)
        if tag_c is not None:
            M = tag_c
        elif tag_l is not None:
            if tag_r is not None:
                M = self.ar_tag.matrix_interpolation(tag_l, tag_r)
            else:
                # Shift value should be based on reality
                M = self.ar_tag.shift_matrix(tag_l, aixs_x=self.TAG_DISTANCE)
        elif tag_r is not None:
            if tag_l is not None:
                M = self.ar_tag.matrix_interpolation(tag_l, tag_r)
            else:
                # Shift value should be based on reality
                M = self.ar_tag.shift_matrix(tag_r, aixs_x=-self.TAG_DISTANCE)
        else:
            rospy.logwarn("can not found tag.")
            return False

        # get target matrix and axis point on 2D matrix
        px = 0
        py = 0
        if axis == 'x':
            M = self.ar_tag.shift_matrix(M, aixs_x=target_distance)
            [px, py] = M[:, 0][:2]
        elif axis == 'y':
            M = self.ar_tag.shift_matrix(M, aixs_y=target_distance)
            [px, py] = M[:, 1][:2]
        elif axis == 'z':
            M = self.ar_tag.shift_matrix(M, aixs_z=target_distance)
            [px, py] = M[:, 2][:2]
        else:
            rospy.logerr("axis error! please check your code.")
            return False
        
        # get turn theta
        theta = math.atan2(py, px)
        if negative:
            theta += math.pi

        if theta > math.pi:
            theta -= 2 * math.pi
        elif theta < -math.pi:
            theta += 2 * math.pi

        # get position poin x-axis and y-axis
        [px, py] = M[:, 3][:2]

        # publish target position on tf tree
        (_, _, angles, translate, _) = tf.transformations.decompose_matrix(M)
        self.broadcaster.sendTransform(
            translation=translate, 
            rotation=tf.transformations.quaternion_from_euler(angles[0], angles[1], angles[2]), 
            time=rospy.Time().now(),
            child="/adjustment_target",
            parent="/base_link"
        )

        # move the spark
        self.spark_motion.to_pose(px, py, math.degrees(theta))
        return True

    def execute(self, goal: TagAdjustmentGoal):
        ret = TagAdjustmentResult()
        try:
            if goal.adjustment_type == goal.TYPE_JUST_FACE:
                ret.result = ret.SUCCESS if self.just_face('y') else ret.ERROR            

            elif goal.adjustment_type == goal.TYPE_CENTER_FACE:
                ret.result = ret.SUCCESS if self.center_face('y', target_distance=-0.4) else ret.ERROR

            else:
                ret.result = ret.ERROR

            if self.action_server.is_active():
                self.action_server.set_succeeded(ret)
        except Exception:
            print(traceback.print_exc())
            self.action_server.set_aborted()

    def preemptCB(self):
        if self.action_server.is_active():
            self.spark_motion.stop()
            self.action_server.set_preempted()


if __name__ == '__main__':
    rospy.init_node('spark_alignment_mark', anonymous=True)

    adj = SparkAdjustment(
        left_id=rospy.get_param("~left_id", 21),
        center_id=rospy.get_param("~center_id", 22),
        right_id=rospy.get_param("~right_id", 23),
        tag_distance=rospy.get_param("~tag_distance", 0.2)
    )
    rospy.spin()
