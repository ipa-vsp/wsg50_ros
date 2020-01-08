#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "wsg_50_tcp_driver/function.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "gripper test node");
    std::string addr = "172.31.1.160";
    unsigned short port = 1000;
    iwtros::function test(addr.c_str(), port);
    test.ack_fault();
    test.homing();
    test.setGraspingForceLimit(70);
    test.move(25, 50, false, true);
    return 0;
}