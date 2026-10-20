#!/usr/bin/python3

import sys
import os
import yaml
import _thread
import threading
import pickle

import rospy
import rospkg
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseResult
import common.msg
import common.srv
from common.msg import MoveStraightDistanceAction, TurnBodyDegreeAction
# import common.action
import swiftpro.msg
from std_msgs.msg import String
from swiftpro.msg import position
from vision_msgs.msg import Detection2DArray

class SwiftProInterface:
    def __init__(self):
        # 创建控制机械臂的topic发布者
        self.arm_position_pub = rospy.Publisher(
            "position_write_topic", swiftpro.msg.position, queue_size=1)   # 机械臂运动位置发布者
        self.arm_pump_pub = rospy.Publisher(
            "pump_topic", swiftpro.msg.status, queue_size=1)               # 机械臂气泵状态发布者
        self.arm_status_pub = rospy.Publisher(
            "swiftpro_status_topic", swiftpro.msg.status, queue_size=1)    # 机械臂开关状态发布者


    def set_pose(self, x, y, z):
        '''
        发布机械臂运动位置
        '''
        pos = position()
        pos.x = x
        pos.y = y
        pos.z = z
        # rospy.loginfo(f"set pose {x},{y},{z}")
        self.arm_position_pub.publish(pos)
        rospy.sleep(3)

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

        # 根据yaml文件，确定抓取物品的id号
        self.search_id = [
            items_content["items"][items_content["objects"]["objects_a"]]
        ]

    def detector(self):
        '''
        获取需要抓取的物品在显示屏上的坐标位置
        @return: 需要抓取的物品列表cube_list

        cube_list[i]:代表第几个物体
        cube_list[i][0]:代表第i个物体的ID信息;               cube_list[i][1]:代表第i个物体的位置信息
        cube_list[i][1][1]:代表第i个物体的x方向上的位置;     cube_list[i][1][2:代表第i个物体的y方向上的位置
        '''
        obj_dist = {}
        cube_list = []
        obj_array = None

        try:
            obj_array = rospy.wait_for_message(
                "/objects", Detection2DArray, timeout=5)
        except Exception:
            cube_list.clear()
            return cube_list
        
        # 提取
        for obj in obj_array.detections:
            obj_dist[obj.results[0].id] = [obj.bbox.center.x, obj.bbox.center.y, 0]

        # 筛选出需要的物品 cube_list中的key代表识别物体的ID，value代表位置信息
        for key, value in obj_dist.items():
            if key in self.search_id:
                cube_list.append([key, value])

        return cube_list


class ArmAction:
    def __init__(self):

        self.cam = CamAction()

        # 获取标定文件数据
        filename = os.environ['HOME'] + "/thefile.txt"
        with open(filename, 'r') as f:
            s = f.read()
        arr = s.split()
        self.x_kb = [float(arr[0]), float(arr[1])]
        self.y_kb = [float(arr[2]), float(arr[3])]        

        # 创建机械臂控制接口的对象
        self.interface = SwiftProInterface()

        self.grasp_status_pub = rospy.Publisher("/grasp_status", String, queue_size=1)
 
    
    def grasp(self):
        '''
        使用深度学习找到所需物品在图像上的位置, 估算物品实际位置, 让机械臂抓取
        @return: 抓取到物品的id, 0为未识别到需要的物品
        '''
        r1 = rospy.Rate(0.25)
        r2 = rospy.Rate(10)
        x = 0
        y = 0
        z = 0

        # 寻找物品
        cube_list = self.cam.detector()
        # rospy.sleep(2)
        if len(cube_list) == 0:
            rospy.logwarn("没有找到物品啊。。。去下一个地方")
            self.grasp_status_pub.publish(String("1"))
            return 0

        cube_list = sorted(cube_list, key=lambda x: x[1][1], reverse=False)

        # 获取机械臂目标位置
        x = self.x_kb[0] * cube_list[0][1][1] + self.x_kb[1]
        y = self.y_kb[0] * cube_list[0][1][0] + self.y_kb[1]
        z = -55

        print(f"找到物品了！它在: {x}, {y}, {z}")

        # 机械臂移动到目标位置上方
        self.interface.set_pose(x, y, z + 20)
        rospy.sleep(1)

        # 打开气泵，进行吸取
        self.interface.set_pump(True)
        rospy.sleep(1)

        # 机械臂移动到目标位置
        self.interface.set_pose(x, y, z)
        rospy.sleep(1)

        
        # r2.sleep()

        # 抬起目标方块
        print(f"我把物品抬起来了")
        self.interface.set_pose(x, y, z + 120)
        rospy.sleep(2)
        # r1.sleep()

        self.grasp_status_pub.publish(String("0"))

        return cube_list[0][0]

    def drop(self, check=False):
        '''
        放置方块, 可以先判断是否有方块, 从而调整放置高度
        @param check: 是否判断有无方块, 默认判断
        @return item_id: 执行结果
        '''
        r1 = rospy.Rate(0.25)
        r2 = rospy.Rate(10)
        x = 300
        y = 0
        z = 120

        if (check):
            # 控制机械臂移动到其他地方，以免挡住摄像头
            self.interface.set_pose(0, 225, 160)
            # r1.sleep()
            rospy.sleep(2)
            cube_list = self.detector()
            if len(cube_list) > 0:
                x = self.x_kb[0] * cube_list[0][1][1] + self.x_kb[1]
                y = self.y_kb[0] * cube_list[0][1][0] + self.y_kb[1]
                z = 120

        # 默认放置位置
        self.interface.set_pose(x, y, z)
        rospy.sleep(2)


        # 关闭气泵
        self.interface.set_pump(0)
        # r2.sleep()
        rospy.sleep(2)

        # self.interface.set_pose(300, 0, 120)
        # r1.sleep()
        self.arm_grasp_ready()  # 移动机械臂到其他地方
        rospy.sleep(2)

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
        self.interface.set_pose(130, 0, 35)
        if block:
            rospy.sleep(3)

    def arm_grasp_ready(self, block=False):
        '''
        移动机械臂到摄像头看不到的地方，以方便识别与抓取
        '''
        self.interface.set_pose(50, 180, 160)  
        if block:
            rospy.sleep(3)

    def arm_grasp_laser(self, block=False):
        self.interface.set_pose(160, 0, 20)
        if block:
            rospy.sleep(3)

class RobotMoveAction:
    def __init__(self):
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

        # 创建导航地点的话题发布者
        self.goto_local_pub = rospy.Publisher(
            "mark_nav", String, queue_size=1)

    def goto_local(self, name):
        '''
        根据目标点名称,发布目标位置到MoveBase服务器,根据返回状态进行判断
        @return: True 为成功到达, False 为失败
        '''

        # 发布目标位置
        self.goto_local_pub.publish("go "+name)

        # 设定1分钟的时间限制，进行阻塞等待
        try:
            ret_status = rospy.wait_for_message(
                'move_base/result', MoveBaseActionResult, rospy.Duration(60)).status.status
        except Exception:
            rospy.logwarn("nav timeout!!!")
            ret_status = GoalStatus.ABORTED

        # 如果一分钟之内没有到达，放弃目标
        if ret_status != GoalStatus.SUCCEEDED:
            rospy.Publisher("move_base/cancel", GoalID, queue_size=1).publish(
                GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
            try:
                rospy.wait_for_message(
                    'move_base/result', MoveBaseActionResult, rospy.Duration(3))
            except Exception:
                rospy.logwarn("move_base result timeout. this is abnormal.")
            rospy.loginfo("==========Timed out achieving goal==========")
            return False
        else:
            rospy.loginfo("==========Goal succeeded==========")
            return True
    
    def step_back(self):
        '''
        后退, 用于抓取或放置后使用
        @return: True 为调整成功, False 为调整失败
        '''
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                const_rot_vel=-0.1,
                move_distance=0.2,
            ),
            rospy.Duration.from_sec(5)  # 超过5s为超时
        )
        return True
    
    def step_go(self,dis):
        '''
        前进, 用于抓取或放置前使用
        @return: True 为调整成功, False 为调整失败
        '''
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                const_rot_vel=0.1,
                move_distance=dis,
            ),
            rospy.Duration.from_sec(5)  # 超过5s为超时
        )
        return True


class AutoAction:
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

        # 订阅任务控制指令的话题
        self.task_cmd_sub = rospy.Subscriber("/task_start_flag", String, self.task_cmd_cb) # 订阅任务开始与否信号
        self.task_run_th = threading.Thread(target=lambda: "pass") # 创建线程对象
        self.stop_flag = False  # 任务的启停标志

        # 订阅机械臂手动控制的话题
        self.grasp_sub = rospy.Subscriber("grasp", String, self.grasp_cb)

        rospy.loginfo("spark_auto_match_node is ready")

    # 接收到启动自动赛信号，开始执行任务
    def run_task(self):
        ret = False # 是否导航成功标志
        item_type = 0 
        self.arm.arm_home()  # 移动机械臂到其他地方

        # ===== 现在开始执行任务 =====
        rospy.loginfo("start task now.")

        # ==== 离开起始区,避免在膨胀区域中，导致导航失败 =====
        self.robot.step_go(0.3)

        if self.stop_flag: return

        # ==== 移动机械臂 =====
        self.arm.arm_position_reset()  # 重置机械臂坐标系
        self.arm.arm_grasp_ready()  # 移动机械臂到其他地方

        # ===== 导航到分类区 =====
        if self.robot.goto_local("Classification_area"):
            rospy.sleep(2)
        else :
            rospy.logerr("Navigation to Classification_area failed,please run task again ")
            self.stop_flag = True

        # 创建任务安排字典，设定前往的抓取地点与次数
        sorting_status_times = {
            "Sorting_N":1,
            "Sorting_W":1,
        }
        sorting_name = "Sorting_N"

        # =======开始循环运行前往中心区域抓取与放置任务======
        while True:
            # 根据任务安排逐步执行
            print("readying to sort_areas")
            if sorting_status_times[sorting_name] == 0:
                sorting_status_times.pop(sorting_name)
                if len(sorting_status_times) == 0:
                    break
                else:
                    sorting_name = list(sorting_status_times.keys())[0]

            # =====导航到目标地点====

            ret = self.robot.goto_local(sorting_name) # 导航到目标点
            rospy.sleep(1) # 停稳

            if self.stop_flag: return

            # =====识别并抓取物体====
            item_type = 0
            if ret: # 判断是否成功到达目标点
                print("========扫描中，准备抓取===== ")
                item_type = self.arm.grasp()  # 抓取物品并返回抓取物品的类型
                print("========向后退一点===== ")
                self.robot.step_back()  # 后退

                if self.stop_flag: return

            if item_type == 0: # 如果没有识别到物体，将该地点的抓取次数归0
                sorting_status_times[sorting_name] = 0
                continue

            
            # ====放置物品====
            self.arm.arm_grasp_ready()
            print("========前往放置区===== ")
            ret = self.robot.goto_local("Collection_B") # 根据抓到的物品类型，导航到对应的放置区
            rospy.sleep(2) # 停稳
            if self.stop_flag: return

            if ret: 
                self.arm.drop()  # 放下物品
                self.robot.step_back()  # 后退
                if self.stop_flag: return
            else:
                rospy.logerr("task error: navigation to the drop_place fails")
                self.arm.drop()
            if self.stop_flag: return

                # 下一步
            sorting_status_times[sorting_name] = sorting_status_times[sorting_name] - 1

        self.arm.arm_home()
        # act.goto_local("sp")

        rospy.logwarn("***** task finished *****")
        rospy.logwarn("if you want to run task again. ")
        rospy.logwarn("Re-send a message to hm_task_cmd topic. ")
        rospy.logwarn("Or press Ctrl+C to exit the program")

    def task_cmd_cb(self,flag):
        if flag :
            if not self.task_run_th.is_alive():
                self.stop_flag = False
                self.task_run_th = threading.Thread(target=self.run_task, args=())
                self.task_run_th.start()
                rospy.loginfo("start task!!!")
            else:
                rospy.logwarn("waiting for thread exit...")
                self.stop_flag = True
                self.task_run_th.join()
                rospy.logwarn("thread exit success")

    def grasp_cb(self, msg):
        if not self.task_run_th.is_alive():
            if msg.data == "1":
                self.arm.grasp()
            elif msg.data == "0":
                self.arm.drop()
                self.arm.arm_grasp_ready()
            else:
                rospy.logwarn("grasp msg error")


if __name__ == '__main__':
    try:
        AutoAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Mark_move finished.")
