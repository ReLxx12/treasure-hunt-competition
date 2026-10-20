/*
 * SPARK is the best robot for education!
 * Copyright (c) 2021.05 ShenZhen NXROBO
 * All rights reserved.
 * Author: litian.zhuang
 */
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <common/TurnBodyDegreeAction.h>
#include "spark_base/GyroMessage.h"
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include "b_turnbody_node/PIDConfig.h"
#include <std_msgs/Float64.h>

/**
 * @brief actionlib在这个命名空间
 */
namespace turn_body_action
{
/**
 * @brief The TurnBody class 实现地盘旋转的actionlib srv
 */
class TurnBody
{
 public:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber gyro_sub_, head_yaw_sub_;
  ros::Publisher direct_cmd_pub_, raw_cmd_pub_;
  float last_gyro_yaw_, curr_gyro_yaw_, delta_gyro_yaw_, turn_done_degree_,
      turn_start_yaw_, target_angular_;
  boost::mutex turn_done_lock_;
  actionlib::SimpleActionServer<common::TurnBodyDegreeAction> as_;

  double error_now_, error_sum_;
  double P_, I_, max_rotation_vel_, min_rotation_vel_, dead_degree_;
  dynamic_reconfigure::Server<b_turnbody_node::PIDConfig> dynamic_;  ///< Dynamic reconfigure server

  TurnBody(ros::NodeHandle nh, ros::NodeHandle pnh)
      : nh_(nh),
        pnh_(pnh),
        last_gyro_yaw_(0),
        curr_gyro_yaw_(0),
        delta_gyro_yaw_(0),
        turn_done_degree_(0),
        turn_start_yaw_(0),
        error_now_(0),
        error_sum_(0),
        as_(nh_, "turn_body", boost::bind(&TurnBody::executeCb, this, _1),
            false)  // bool: auto start
  {
    gyro_sub_ = nh_.subscribe<spark_base::GyroMessage>("/spark_base/gyro", 1,
                                                   &TurnBody::baseGyroCb, this);
    // head_yaw_sub_ = nh_.subscribe<std_msgs::Float64>("/head_yaw", 1, &TurnBody::headYawCb, this);

    direct_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    raw_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);  //给b_movebase速度平滑使用，stupid!
    as_.registerPreemptCallback(boost::bind(&TurnBody::preemptCb, this));

    dynamic_reconfigure::Server<b_turnbody_node::PIDConfig>::CallbackType cb;
    cb = boost::bind(&TurnBody::reconfigureCb, this, _1, _2);
    dynamic_.setCallback(cb);

    pnh_.param("angular_p", P_, 0.017);
    pnh_.param("angular_i", I_, 0.0);
    pnh_.param("max_rotation_vel", max_rotation_vel_, 1.15);
    pnh_.param("min_rotation_vel", min_rotation_vel_, 0.3);
    pnh_.param("dead_degree", dead_degree_, 2.5);

    as_.start();
  }

  /**
   * @brief reconfigureCb 动态调整PID参数，最大旋转速度等参数
   * @param config
   * @param level
   */
  void reconfigureCb(b_turnbody_node::PIDConfig &config, uint32_t level)
  {
    P_ = config.Angular_P;
    I_ = config.Angular_I;
    max_rotation_vel_ = config.Angular_MaxZVel;
    min_rotation_vel_ = config.Angular_MinZVel;
    dead_degree_ = config.Angular_deaddegree;
  }

  void headYawCb(const std_msgs::Float64ConstPtr &head_yaw)
  {
    curr_gyro_yaw_ = head_yaw->data;
    if (as_.isActive())
    {
      delta_gyro_yaw_ = curr_gyro_yaw_ - last_gyro_yaw_;
      delta_gyro_yaw_ =
          (delta_gyro_yaw_ > 180) ?
              (delta_gyro_yaw_ - 360) :
              ((delta_gyro_yaw_ < -180) ?
                  (360 + delta_gyro_yaw_) : delta_gyro_yaw_);
      boost::unique_lock<boost::mutex> lock(turn_done_lock_);
      turn_done_degree_ += delta_gyro_yaw_;
      // ROS_INFO("raw_gyro = %.2f, done_degree = %.2f", curr_gyro_yaw_, turn_done_degree_);
    }
    last_gyro_yaw_ = curr_gyro_yaw_;
  }

  /**
   * @brief baseGyroCb 陀螺的回调，累加已经旋转的角度，处理陀螺+-180的问题
   * @param gyro   陀螺范围：+-180角度
   */
  void baseGyroCb(const spark_base::GyroMessageConstPtr &gyro)
  {
    boost::unique_lock<boost::mutex> lock(turn_done_lock_);
    curr_gyro_yaw_ = gyro->yaw;
    lock.unlock();
    return;
    if (as_.isActive())
    {
      boost::unique_lock<boost::mutex> lock(turn_done_lock_);
      delta_gyro_yaw_ = target_angular_ - curr_gyro_yaw_;
      int mu = delta_gyro_yaw_ < -180 ? 1 : (delta_gyro_yaw_ > 180 ? -1 : 0);  //此步骤非常关键,判断夹角是否超过+180/-180
      delta_gyro_yaw_ = delta_gyro_yaw_ + mu * 360;
      lock.unlock();

      // turn_done_degree_ += delta_gyro_yaw_;
      // ROS_INFO("raw_gyro = %.2f, done_degree = %.2f", curr_gyro_yaw_, turn_done_degree_);
    }
    // last_gyro_yaw_ = curr_gyro_yaw_;
  }

  /**
   * @brief preemptCb 抢占或取消的回调函数，发送0速度并设置抢占态
   */
  void preemptCb()
  {
    ROS_ERROR("Turn body action preempted!");
    if (as_.isActive())
    {
      boost::unique_lock<boost::mutex> lock(turn_done_lock_);
      turn_done_degree_ = 0;
      delta_gyro_yaw_ = 0;
      lock.unlock();
      geometry_msgs::Twist cmd;
      cmd.linear.x = 0;
      cmd.angular.z = 0;
      direct_cmd_pub_.publish(cmd);
      as_.setPreempted();
      ROS_ERROR("turn body real degree: %.2f", turn_done_degree_);
    }
  }

  /**
   * @brief executeCb 执行旋转，旋转速度的大小cmd=p*error + i*sigma(error)
   * @param goal      单位角度，+则正转，-则反转
   */
  void executeCb(const common::TurnBodyDegreeGoalConstPtr &goal)
  {
    ros::Rate r(30);

    //ROS_WARN("turn body goal degree: %.2f", goal->goal_degree);
    delta_gyro_yaw_ = 0;
    target_angular_ = curr_gyro_yaw_ + goal->goal_degree;
    //归一化到陀螺仪的角度
    target_angular_ =
        (target_angular_ < -180) ?
            (360 + target_angular_) :
            ((target_angular_ > 180) ? (target_angular_ - 360) : target_angular_);

    double gap = 0;
    boost::unique_lock<boost::mutex> lock(turn_done_lock_);
    gap = target_angular_ - curr_gyro_yaw_;
    int mu = gap < -180 ? 1 : (gap > 180 ? -1 : 0);  //此步骤非常关键,判断夹角是否超过+180/-180
    gap = gap + mu * 360;
    /*ROS_ERROR("turn body goal degree: %.2f,%.2f,%.2f", target_angular_,
     curr_gyro_yaw_, gap);*/
    lock.unlock();
    if (fabs(gap) <= (fabs(dead_degree_)))
      return;

    float MaxD = 15.0;
    float D_ = MaxD;
    double last_gap = gap;
    while (ros::ok() && as_.isActive())
    {
      r.sleep();
      boost::unique_lock<boost::mutex> lock(turn_done_lock_);
      gap = target_angular_ - curr_gyro_yaw_;
      int mu = gap < -180 ? 1 : (gap > 180 ? -1 : 0);  
      gap = gap + mu * 360;
      ROS_ERROR("turn body goal degree: %.2f,%.2f,%.2f", target_angular_,
       curr_gyro_yaw_, gap);
      lock.unlock();

      if (fabs(gap) > (fabs(dead_degree_)))
      {
        // if(fabs(turn_done_degree_) > (fabs(goal->goal_degree) + dead_degree_))
        // ROS_INFO("BELOW.");
        // if(fabs(turn_done_degree_) < (fabs(goal->goal_degree) - dead_degree_))
        // ROS_INFO("OVER.");
        //如果指定了按照恒定速度
        geometry_msgs::Twist cmd;
        if (goal->is_const_vel == true)
        {
          cmd.linear.x = 0;
          cmd.angular.z =
              goal->goal_degree > 0 ?
                  fabs(goal->const_rot_vel) : -fabs(goal->const_rot_vel);
          if (goal->name == "b_movebase")
            raw_cmd_pub_.publish(cmd);
          else
            direct_cmd_pub_.publish(cmd);
          continue;
        }

        error_now_ = gap;
        error_sum_ += error_now_;
        //防积分饱和
        // error_sum_ = error_sum_ > xxx ? xxx : error_sum_;

        cmd.linear.x = 0;
        cmd.angular.z = P_ * error_now_ + I_ * error_sum_;

        //速度上限幅
        cmd.angular.z =
            cmd.angular.z > max_rotation_vel_ ?
                max_rotation_vel_ : cmd.angular.z;
        cmd.angular.z =
            cmd.angular.z < -max_rotation_vel_ ?
                -max_rotation_vel_ : cmd.angular.z;
        //速度下限幅
        if ((cmd.angular.z > 0) && (cmd.angular.z < min_rotation_vel_))
          cmd.angular.z = min_rotation_vel_;
        else if ((cmd.angular.z < 0) && (cmd.angular.z > -min_rotation_vel_))
          cmd.angular.z = -min_rotation_vel_;

        if (D_ > 0)
          D_--;
        cmd.angular.z = cmd.angular.z - (D_) / MaxD * cmd.angular.z;  // D smoother
        if (as_.isActive())
        {
          if (goal->name == "b_movebase")
            raw_cmd_pub_.publish(cmd);
          else
          {
            direct_cmd_pub_.publish(cmd);
            // ROS_INFO("%f", ros::Time::now().toSec());
          }
        }
        /*ROS_ERROR("public angle vel:%.2f", cmd.angular.z);*/
      }
      else
      {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        if (goal->name == "b_movebase")
          raw_cmd_pub_.publish(cmd);
        else
          direct_cmd_pub_.publish(cmd);
        boost::unique_lock<boost::mutex> lock(turn_done_lock_);

        ROS_WARN("turn body real degree: %.2f", turn_done_degree_);

        delta_gyro_yaw_ = 0;
        target_angular_ = 0;
        error_now_ = 0;
        error_sum_ = 0;
        as_.setSucceeded();
        lock.unlock();
        return;
      }
    }
  }
};
}  // end namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turn_body_actionlib");
  ros::NodeHandle nh, pnh("~");
  turn_body_action::TurnBody turn_body(nh, pnh);
  ros::spin();
  return 0;
}
