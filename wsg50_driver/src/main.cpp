#include <wsg50_driver/wsg50.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;


namespace wsg50
{
    GripperActionServer::GripperActionServer(const rclcpp::NodeOptions & options): Node("gripper_server_node", options)
    {
        // Declare Gripper Parameters
        RCLCPP_INFO(this->get_logger(), "Starting Gripper Server");
        this->declare_parameter("gripper_ip", "172.31.1.160");
        this->declare_parameter("port", 1501);
        this->declare_parameter("grasp_force", 40);
        this->declare_parameter("grasp_speed", 10.0);

        this->addr_ = this->get_parameter("gripper_ip").as_string();
        this->port_ = this->get_parameter("port").as_int();
        this->grasp_force_ = this->get_parameter("grasp_force").as_int();
        this->speed_ = (float)this->get_parameter("grasp_speed").as_double();

        RCLCPP_INFO(this->get_logger(), "Connecting to Gripper with IP addr: %s and Port: %d.", this->addr_.c_str(), this->port_);

        this->gripper_ = std::make_shared<iwtros::function>(this->addr_.c_str(), (unsigned short)port_);

        this->gripper_command_server_ = rclcpp_action::create_server<GripperCommand>(
            this, "~/gripper_action",
            std::bind(&GripperActionServer::handle_goal, this, _1, _2),
            std::bind(&GripperActionServer::handel_cancel, this, _1),
            std::bind(&GripperActionServer::handle_accepted, this, _1)
        );

        this->joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 1);
        this->gripper_result_  = this->create_publisher<std_msgs::msg::Bool>("~/gripper_result_feedback", 100);
        //this->timer_ = this->create_wall_timer(rclcpp::WallRate(30).period(), [this](){return publishJointStates(); });

        // Check Gripper Connection
        if(gripper_->connected)
        {
            RCLCPP_INFO(this->get_logger(), "Gripper acknowledge ... ");
            gripper_->ack_fault();
            RCLCPP_INFO(this->get_logger(), "Gripper Homing ... ");
            gripper_->homing();
            gripper_->grasp(0, 20.0);
            gripper_->homing();
            rclcpp::sleep_for(5s);
            if (grasp_force_ > 0) gripper_->setGraspingForceLimit(grasp_force_);
            this->sub_ack_ = this->create_subscription<std_msgs::msg::Bool>("~/ack_gripper", 10, std::bind(&GripperActionServer::ackCallback, this, _1));
        }
    }

    void GripperActionServer::ackCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if(msg->data)
        {
            RCLCPP_WARN(this->get_logger(), "Acknowledging the gripper again!");
            gripper_->ack_fault();
            gripper_->homing();
        }
    }

    rclcpp_action::GoalResponse GripperActionServer::handle_goal(const rclcpp_action::GoalUUID & uuid,
                                                    std::shared_ptr<const GripperCommand::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with position %f", goal->command.position);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse GripperActionServer::handel_cancel(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void GripperActionServer::handle_accepted(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
    {
        std::thread{std::bind(&GripperActionServer::onExecuteGripperCommand, this, _1), goal_handle}.detach();
    }

    void GripperActionServer::onExecuteGripperCommand(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        const auto targetWidth = std::abs(2000 * goal->command.position);
        const auto targetForce = goal->command.max_effort;
        auto pub_result_feedback = std_msgs::msg::Bool();

        std::unique_lock<std::mutex> guard(gripper_state_mutex_);
        //constexpr double kSamplePositionThreshold = 1e-3;
        auto result = std::make_shared<control_msgs::action::GripperCommand::Result>();
        if(targetWidth > GRIPPER_MAX_OPEN || targetWidth < 0.0)
        {
            RCLCPP_ERROR(this->get_logger(), "GripperServer: Commanding out of range width! max_width = %f, command = %f ", GRIPPER_MAX_OPEN, targetWidth);
            goal_handle->abort(result);
            return;
        }
        // Todo: Add current state elements. 
        guard.unlock();

        auto gripper_result = std::make_shared<GripperCommand::Result>();
        try
        {
            gripper_result->reached_goal = gripper_command(targetWidth, targetForce);
        } catch(const std::exception &e) {
            gripper_result->reached_goal = false;
            RCLCPP_ERROR(this->get_logger(), e.what());
        }

        if(rclcpp::ok())
        {
            if(gripper_result->reached_goal)
            {
                //rclcpp::sleep_for(std::chrono::seconds(5));
                RCLCPP_INFO(this->get_logger(), "Gripper succeeded");
                goal_handle->succeed(gripper_result);
                pub_result_feedback.data = true;
                gripper_result_->publish(pub_result_feedback);
            } else {
                RCLCPP_INFO(this->get_logger(), "Gripper failed");
                goal_handle->abort(gripper_result);
                pub_result_feedback.data = false;
                gripper_result_->publish(pub_result_feedback);
            }
        }
    }

    bool GripperActionServer::gripper_command(const double targetWidth, const double targetForce)
    {
        if(targetForce > 0 && grasp_force_ > 0)
        {
            RCLCPP_INFO(this->get_logger(), "Setting Grasp force to %5.1f", targetForce);
            gripper_->setGraspingForceLimit(targetForce);
        }

        // return move action this->status_msgs.width
        bool res = gripper_->move(targetWidth, 40.0, false, false);
        while(!res) res = gripper_->move(targetWidth, 40.0, false, false);
        return res;
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

        std::future<std::shared_ptr<typename GripperCommand::Result>> result_future = std::async(std::launch::async, command_execution_thread);

        // Todo While
        if(rclcpp::ok())
        {
            const auto result = result_future.get();
            std::lock_guard<std::mutex> gaurd(gripper_state_mutex_);
            // result->position = // todo current position
            result->effort = 0.;

            if(result->reached_goal)
            {
                //rclcpp::sleep_for(std::chrono::seconds(5));
                RCLCPP_INFO(this->get_logger(), "Gripper succeeded");
                goal_handle->succeed(result);
            } else {
                RCLCPP_INFO(this->get_logger(), "Gripper failed");
                goal_handle->abort(result);
            }
        }
    }

    // void GripperActionServer::publishJointStates()
    // {
    //     std::string names[3] = { "opening", "speed", "force" };

    //     status_.status = "UNKNOWN";

    //     sensor_msgs::msg::JointState joint;
    //     joint.header.frame_id = "";
    //     joint.name.push_back("wsg50_finger_left_joint");
    //     joint.name.push_back("wsg50_finger_right_joint");
    //     joint.position.resize(2);
    //     joint.velocity.resize(2);
    //     joint.effort.resize(2);
    // }
    
} // namespace wsg50

RCLCPP_COMPONENTS_REGISTER_NODE(wsg50::GripperActionServer)