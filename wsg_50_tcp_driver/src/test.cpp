#include <ros/ros.h>
#include "wsg_50_tcp_driver/function.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "test");
    std::string addr = "17.64.64.6";
    unsigned short port = 22;
    iwtros::function test(addr.c_str(), port);
    
    return 0;
}