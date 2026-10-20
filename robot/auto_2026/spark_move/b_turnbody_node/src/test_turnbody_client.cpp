/*
 * SPARK is the best robot for education!
 * Copyright (c) 2021.05 ShenZhen NXROBO
 * All rights reserved.
 * Author: litian.zhuang
 */

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <common/TurnBodyDegreeAction.h>

#include <actionlib/client/simple_action_client.h>

namespace turn_body_action
{
class TurnBodyClient
{
public:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  actionlib::SimpleActionClient<common::TurnBodyDegreeAction> ac_;
  int last_;

  TurnBodyClient(ros::NodeHandle nh) : nh_(nh), last_(0), ac_("turn_body", true)  // true: 不需要spin,有线程在spin
  {
    sub_ = nh_.subscribe<std_msgs::Int32>("/turnbody_client", 1, &TurnBodyClient::srvCb, this);
    ac_.waitForServer(ros::Duration(5));
  }

  void srvCb(const std_msgs::Int32ConstPtr &degree)
  {
    /*if (degree->data == last_)
      return;*/
    common::TurnBodyDegreeGoal goal;
    goal.goal_degree = degree->data;
    ac_.sendGoal(goal);
    last_ = degree->data;
    ROS_INFO("Send turn body degree: %.2f", goal.goal_degree);
  }
};
}  // end namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "client_node");
  ros::NodeHandle nh;
  turn_body_action::TurnBodyClient client_node(nh);
  ros::spin();
  return 0;
}
