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

#include <wsg50_driver/visibility_control.h>

#define GRIPPER_MAX_OPEN 110.0
#define GRIPPER_MIN_OPEN 0.0

namespace wsg50
{
    class GripperActionServer : public rclcpp::Node
    {
        public:
            using GripperCommand = control_msgs::action::GripperCommand;
            using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;

            GripperActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
            ~GripperActionServer();
        
        private:
            rclcpp_action::Server<GripperCommand>::SharedPtr gripper_command_server_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
            rclcpp::TimerBase::SharedPtr timer_;
            std::mutex gripper_state_mutex_;
            std::string addr_;
            int port_;

            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                                    std::shared_ptr<const GripperCommand::Goal> goal);
            rclcpp_action::CancelResponse handel_cancel(const std::shared_ptr<GoalHandleGripperCommand> goal_handle);

            void executeGripperCommand(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle,
                                       const std::function<bool()>& command_handler);
            void onExecuteGripperCommand(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle);

            void publishJointStates();
    };
} // namespace wsg50



#endif // WSG50_HPP