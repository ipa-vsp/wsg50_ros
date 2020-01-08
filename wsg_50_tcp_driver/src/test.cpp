#include <ros/ros.h>
#include "wsg_50_tcp_driver/function.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "test");
    std::string addr = "172.31.1.160";
    unsigned short port = 22;
    iwtros::function test(addr.c_str(), port);
    test.homing();
    
    return 0;
}