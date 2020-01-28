#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
//#include "wsg_50_tcp_driver/function.h"
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "gripper_test_node");
    ROS_INFO("Init");
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> client("/wsg50_gripper/wsg50_gripper_action", true);
    ROS_INFO("Waiting for sever");
    client.waitForServer(ros::Duration(5.0));
    control_msgs::GripperCommandGoal goal;
    goal.command.position = 0.05;
    ROS_INFO("Sending goal");
    client.sendGoal(goal);
    ROS_INFO("Wait for result");
    bool a = client.waitForResult(ros::Duration(30.0));
    if (a) ROS_INFO("DOne ...");
    return 0;
}