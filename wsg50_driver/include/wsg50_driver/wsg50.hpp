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
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "control_msgs/action/gripper_command.hpp"

#include <wsg50_driver/visibility_control.h>

#include <wsg50_driver/function.h>

#define GRIPPER_MAX_OPEN 110.0
#define GRIPPER_MIN_OPEN 0.0

namespace wsg50
{
    typedef struct 
    {
        /* data */
        std::string status;
        double width;
        double speed;
        double force;
    }gripper_status;
    
    class GripperActionServer : public rclcpp::Node
    {
        public:
            using GripperCommand = control_msgs::action::GripperCommand;
            using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;

            ACTION_WSG50_GRIPPER_CPP_PUBLIC
            explicit GripperActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        
        private:
            rclcpp_action::Server<GripperCommand>::SharedPtr gripper_command_server_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_ack_;
            rclcpp::TimerBase::SharedPtr timer_;
            std::mutex gripper_state_mutex_;
            std::string addr_;
            int port_;
            std::shared_ptr<iwtros::function> gripper_;
            int grasp_force_;
            double speed_;
            gripper_status status_;

            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                                    std::shared_ptr<const GripperCommand::Goal> goal);
            rclcpp_action::CancelResponse handel_cancel(const std::shared_ptr<GoalHandleGripperCommand> goal_handle);
            void handle_accepted(const std::shared_ptr<GoalHandleGripperCommand> goal_handle);

            void executeGripperCommand(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle,
                                       const std::function<bool()>& command_handler);
            void onExecuteGripperCommand(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle);

            void publishJointStates();
            void ackCallback(const std_msgs::msg::Bool::SharedPtr msg);

            bool gripper_command(const double targetWidth, const double targetForce);
    };
} // namespace wsg50



#endif // WSG50_HPP