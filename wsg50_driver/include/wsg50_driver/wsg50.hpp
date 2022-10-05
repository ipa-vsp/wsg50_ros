#ifndef WSG50_HPP
#define WSG50_HPP

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <thread>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "control_msgs/action/gripper_command.hpp"


namespace gripper
{
    class wsg50
    {
        public:
        wsg50(const char * addr, unsigned short port);
        ~wsg50();

        void gripperCommandExecution(const control_msgs::action::GripperCommand_Goal::ConstPtr goal);
    };
} // namespace gripper



#endif // WSG50_HPP