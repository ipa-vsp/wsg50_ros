#include <wsg50_driver/wsg50.hpp>


namespace wsg50
{
    GripperActionServer::GripperActionServer(const rclcpp::NodeOptions & options): Node("wsg50_gripper_node", options)
    {
        // Declare Gripper Parameters
        this->declare_parameter("gripper_ip", "0.0.0.0");
        this->declare_parameter("port", 0);

        this->addr_ = this->get_parameter("gripper_ip").as_string();
        this->port_ = this->get_parameter("port").as_int();

        this->gripper_command_server_ = rclcpp_action::create_server<GripperCommand>(
            this, "~/gripper_action",
            [this](auto uuid, auto goal){return handle_goal(uuid, goal);},
            [this](const auto& goal_handle){return handel_cancel(goal_handle);},
            [this](const auto& goal_handle){
                return std::thread{[goal_handle, this](){ onExecuteGripperCommand(goal_handle); }}.detach();
            }
        );

        this->joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 1);
        this->timer_ = this->create_wall_timer(rclcpp::WallRate(30).period(), [this](){return publishJointStates(); });
    }

    rclcpp_action::GoalResponse GripperActionServer::handle_goal(const rclcpp_action::GoalUUID & uuid,
                                                    std::shared_ptr<const GripperCommand::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse GripperActionServer::handel_cancel(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void GripperActionServer::onExecuteGripperCommand(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        const auto targetWidth = std::abs(2000 * goal->command.position);
        const auto targetForce = goal->command.max_effort;

        std::unique_lock<std::mutex> guard(gripper_state_mutex_);
        constexpr double kSamplePositionThreshold = 1e-3;
        auto result = std::make_shared<control_msgs::action::GripperCommand::Result>();
        if(targetWidth > GRIPPER_MAX_OPEN || targetWidth < 0.0)
        {
            RCLCPP_ERROR(this-get_logger(), "GripperServer: Commanding out of range width! max_width = %f, command = %f ", GRIPPER_MAX_OPEN, targetWidth);
            goal_handle->abort(result);
            return;
        }
        // Todo: Add current state elements. 
        guard.unlock();

        auto command = [targetWidth, targetForce, goal, this](){
            if(targetForce > 0)
            {
                // Return Grasp action
            }

            // return move action this->status_msgs.width
            return false;
        };

        executeGripperCommand(goal_handle, command);
    }

    void GripperActionServer::executeGripperCommand(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle,
                                       const std::function<bool()>& command_handler)
    {
        auto command_execution_thread = [command_handler, this](){
            auto result = std::make_shared<GripperCommand::Result>();
            try
            {
                result->reached_goal = command_handler();
            } catch(const std::exception &e) {
                result->reached_goal = false;
                RCLCPP_ERROR(this->get_logger(), e.what());
            }

            return result;
        };
    }
    
} // namespace wsg50
