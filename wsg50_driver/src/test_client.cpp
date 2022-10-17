#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "control_msgs/action/gripper_command.hpp"

namespace wsg50
{
    class GripperActionClient : public rclcpp::Node
    {
        public:
            using GripperCommand = control_msgs::action::GripperCommand;
            using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

            explicit GripperActionClient(const rclcpp::NodeOptions &options) : Node("gripper_client_node", options)
            {
                this->client_ptr = rclcpp_action::create_client<GripperCommand>(this, "~/gripper_action");
                
                this->timer_ = this->create_wall_timer(std::chrono::microseconds(500), std::bind(&GripperActionClient::send_goal, this));
            }

            void send_goal()
            {
                using namespace std::placeholders;

                //this->timer_->cancel();
                if(!this->client_ptr->wait_for_action_server())
                {
                    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                    rclcpp::shutdown();
                }

                auto open_goal = GripperCommand::Goal();
                auto close_goal = GripperCommand::Goal();

                open_goal.command.position = 0.054;
                open_goal.command.max_effort = 0.0;
                close_goal.command.position = 0.001;
                close_goal.command.max_effort = 40.0;

                RCLCPP_INFO(this->get_logger(), "Sending goal");

                auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
                send_goal_options.goal_response_callback = std::bind(&GripperActionClient::goal_response_callback, this, _1);
                send_goal_options.result_callback = std::bind(&GripperActionClient::result_callback, this, _1);

                RCLCPP_INFO(this->get_logger(), "Open");
                this->client_ptr->async_send_goal(open_goal, send_goal_options);
                
                rclcpp::sleep_for(std::chrono::seconds(5));
                RCLCPP_INFO(this->get_logger(), "Close");
                this->client_ptr->async_send_goal(close_goal, send_goal_options);
                rclcpp::sleep_for(std::chrono::seconds(5));
                //this->timer_->call();
            }
        
        private:
            rclcpp_action::Client<GripperCommand>::SharedPtr client_ptr;
            rclcpp::TimerBase::SharedPtr timer_;

            void goal_response_callback(const GoalHandleGripper::SharedPtr & goal_handle)
            {
                if(!goal_handle)
                {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
                }
            }

            void feedback_callback(GoalHandleGripper::SharedPtr, const std::shared_ptr<const GripperCommand::Feedback> feedback)
            {
                std::stringstream ss;
                ss << "Next received: " ;
                // for (auto number : feedback->position)
                // {
                //     ss << number << " ";
                // }

                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            }

            void result_callback(const GoalHandleGripper::WrappedResult & result)
            {
                switch (result.code)
                {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was Success");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                    return;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    return;
                }
            }
    };
} // namespace wsg50

RCLCPP_COMPONENTS_REGISTER_NODE(wsg50::GripperActionClient)