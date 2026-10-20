/*********************************************************************
 *
 *  Copyright (c) 2012, NXROBO.
 *  All rights reserved.
 *
 * Author: litian.zhuang on 02/22/2022
 *
 *********************************************************************/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <time.h>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>
#include <common/MoveStraightDistanceAction.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int32.h>
using namespace std;
class MoveStraightNode
{

private:
    ros::NodeHandle n;

    ros::Subscriber move_dist_sub;

    actionlib::SimpleActionClient<common::MoveStraightDistanceAction> move_base_client_; // move base actionlib client

public:
    common::MoveStraightDistanceGoal move_base_goal;
    /**
     * 	构造函数
     */
    MoveStraightNode(ros::NodeHandle nh) : move_base_client_("move_straight", true)
    {
        n = nh;
        move_dist_sub = n.subscribe("/get_move_start", 1, &MoveStraightNode::move_dist_Callback, this);
    }

    /**
     * 	析构函数
     */
    virtual ~MoveStraightNode()
    {

        // destroyThread(&thtbd);
    }

    void Spin()
    {
        ros::spin();
    }

    void move_dist_Callback(std_msgs::String msg)
    {
        ROS_INFO("%s", msg.data.c_str());
        move_base_goal.move_distance = 0.2;
        move_base_goal.const_rot_vel = atof(msg.data.c_str());
        move_base_client_.sendGoalAndWait(move_base_goal, ros::Duration(10));
        ROS_WARN("------finish move,vel is %2f", move_base_goal.const_rot_vel);
    }

    /**
     * @brief destroyThread     消毁线程
     * @param th                线程地址指针
     * @return
     */
    bool destroyThread(boost::thread **th)
    {

        if ((*th) != NULL)
        {
            (*th)->interrupt();
            (*th)->join();
            delete (*th);
            (*th) = NULL;
            return true;
        }
        return true;
    }
};

/**
 *主函数
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "b_mvs_node");
    ros::NodeHandle n;
    MoveStraightNode mvs(n);
    mvs.Spin();
    return 0;
}
