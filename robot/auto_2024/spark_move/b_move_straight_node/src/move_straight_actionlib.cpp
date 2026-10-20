/*
 * SPARK is the best robot for education!
 * Copyright (c) 2021.05 ShenZhen NXROBO
 * All rights reserved.move_straight
 * Author: litian.zhuang
 */
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <common/MoveStraightDistanceAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>    // odom
#include <sensor_msgs/LaserScan.h>
#include <common/GetFrontBackDistance.h>

#include "math.h"
/**
 * @brief actionlib在这个命名空间
 */
namespace move_straight_action
{
/**
 * @brief The move_straight class 实现地盘旋转的actionlib srv
 */
class MoveStraight
{
 public:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber odom_sub_, lidar_sub_;
  ros::Publisher direct_cmd_pub_, cmd_pub_;
  common::MoveStraightDistanceResult result_;
  ros::ServiceServer distance_srv;
  geometry_msgs::Point curr_odom_pose_, diff_p, first_pose;
  float first_scan_distance, current_scan_distance, front_distance, back_distance;
  float last_gyro_yaw_, curr_gyro_yaw_, delta_gyro_yaw_, turn_done_degree_,
      turn_start_yaw_, target_angular_;
  boost::mutex odom_move_done_lock_, scan_move_done_lock_;
  actionlib::SimpleActionServer<common::MoveStraightDistanceAction> as_;
  double  move_done_distance, c_p, target_pose_;

  MoveStraight(ros::NodeHandle nh, ros::NodeHandle pnh)
      : nh_(nh),
        pnh_(pnh),
        as_(nh_, "move_straight", boost::bind(&MoveStraight::executeCb, this, _1),
            false)  // bool: auto start
  {
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 1,
                                                   &MoveStraight::getOdomCb, this);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    lidar_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, &MoveStraight::lidarCallback, this);
    distance_srv = nh_.advertiseService("get_distance", &MoveStraight::getDistance, this);

    as_.registerPreemptCallback(boost::bind(&MoveStraight::preemptCb, this));


    as_.start();
  }

  bool getDistance(common::GetFrontBackDistance::Request  &req, common::GetFrontBackDistance::Response &res )
  {
    ROS_INFO("getDistance request, response: [%ld][%ld]", front_distance, back_distance);
    boost::unique_lock<boost::mutex> lock_scan(scan_move_done_lock_);
    res.front_distance = front_distance;
    res.back_distance = back_distance;
    lock_scan.unlock();
    res.result = res.SUCCESS;
    return true;
  }

  /**
   * @brief getOdomCb 里程计的回调
   * @param odommsg  里程计数据   
   */
  void getOdomCb(const nav_msgs::OdometryConstPtr &odommsg)
  {
    boost::unique_lock<boost::mutex> lock_odom(odom_move_done_lock_);
    curr_odom_pose_ = odommsg->pose.pose.position;
    lock_odom.unlock();
  }
  /**
   * @brief lidarCallback 雷达的回调
   * @param scan_msg  雷达数据   
   */
  void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
  {

    int num = (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment;

    // choose the direction and range you want to test
    double expect_angle = 180;

    // get the front direction value
    int front_direction = (expect_angle / 360) * num;

    // get the distance value
    boost::unique_lock<boost::mutex> lock_scan(scan_move_done_lock_);
    if((scan_msg->ranges[1]>0)&&(scan_msg->ranges[1]<8))
      back_distance = scan_msg->ranges[1];
    if((scan_msg->ranges[front_direction]>0)&&(scan_msg->ranges[front_direction]<8))
      front_distance = scan_msg->ranges[front_direction];
    lock_scan.unlock();

    //printf("front_distance=%lf,back_distance=%lf\n", front_distance, back_distance);
  }

  /**
   * @brief preemptCb 抢占或取消的回调函数，发送0速度并设置抢占态
   */
  void preemptCb()
  {
    ROS_ERROR("Move action preempted!");
    if (as_.isActive())
    {
      geometry_msgs::Twist cmd;
      result_.result = false;
      as_.setPreempted(result_);
      cmd.linear.x = 0;
      cmd.angular.z = 0;
      cmd_pub_.publish(cmd);
      ROS_ERROR("move done distance: %.2f", move_done_distance);
    }
  }

  /**
   * @brief executeCb 执行旋转
   * @param goal      单位角度，+则正转，-则反转
   */
  void executeCb(const common::MoveStraightDistanceGoalConstPtr &goal)
  {
    ros::Rate r(30);
    geometry_msgs::Twist cmd;
    int finish_bit = 0;
    common::MoveStraightDistanceFeedback feedback;
    first_pose = curr_odom_pose_;
    if(goal->type==goal->TYPE_SCAN)
    {
      if(goal->const_rot_vel>0)
      {
        first_scan_distance = front_distance;
      }  
      else
      {
        first_scan_distance = back_distance;
      }
      if(goal->move_distance>first_scan_distance)
      {
        ROS_ERROR("the distance you want to go is less than the scan distace! %f>%f.",goal->move_distance, first_scan_distance);
         result_.result = false;
         finish_bit = 0;
         as_.setSucceeded(result_);
        return;
      }
    }
    while (ros::ok() && as_.isActive())
    {
      r.sleep();
      if(goal->type==goal->TYPE_SCAN)
      {
        boost::unique_lock<boost::mutex> lock_scan(scan_move_done_lock_);
        if(goal->const_rot_vel>0)
        {
          current_scan_distance = front_distance;
        }
        else
        {
          current_scan_distance = back_distance;
        }

        lock_scan.unlock();
        diff_p.x = first_scan_distance - current_scan_distance;
        move_done_distance = diff_p.x;
        if(diff_p.x >= goal->move_distance)
        {
          cmd.linear.x = 0;
          cmd.angular.z = 0;
          finish_bit = 1;
        }
        else
        {
          cmd.linear.x = goal->const_rot_vel;
          cmd.angular.z = 0;
        } 
      }
      else
      {
        boost::unique_lock<boost::mutex> lock_odom(odom_move_done_lock_);
        diff_p.x = curr_odom_pose_.x-first_pose.x;
        diff_p.y = curr_odom_pose_.y-first_pose.y;	
        lock_odom.unlock();
        move_done_distance = sqrt(diff_p.x * diff_p.x + diff_p.y * diff_p.y);
        if ((diff_p.x * diff_p.x + diff_p.y * diff_p.y) >= (goal->move_distance * goal->move_distance))
        {
          cmd.linear.x = 0;
          cmd.angular.z = 0;
          finish_bit = 1;
        }
        else
        {
          cmd.linear.x = goal->const_rot_vel;
          cmd.angular.z = 0;
        }
      }

      cmd_pub_.publish(cmd);
      feedback.current_distance = move_done_distance;
      as_.publishFeedback(feedback);
      if(finish_bit)
      {
         result_.result = true;
         finish_bit = 0;
         as_.setSucceeded(result_);
         return;
      }
    }
    cmd.linear.x = 0;
    cmd.angular.z = 0;
    cmd_pub_.publish(cmd);
    ROS_ERROR("stop move , distance: %.2f", move_done_distance);

  }
};
}  // end namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_straight_actionlib");
  ros::NodeHandle nh, pnh("~");
  move_straight_action::MoveStraight move_straight(nh, pnh);
  ros::spin();
  return 0;
}
