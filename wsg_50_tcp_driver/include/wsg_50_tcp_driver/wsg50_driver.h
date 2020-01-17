#ifndef WSG50_DRIVER_H
#define WSG50_DRIVER_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include <wsg_50_tcp_driver/function.h>

namespace controller_manager{
    class ControllerManager;
}

namespace iwtros{
    class wsg50 : public hardware_interface::RobotHW{
        public:
            wsg50(ros::NodeHandle& nh);
            ~wsg50();
            void init(ros::NodeHandle& nh);
            void run();
        protected:
            void _init();
            void _ctrl_loop();
            void _load_param();
            void _read(ros::Duration elapsed_time);
            void _write(ros::Duration elapsed_time);

            // Interface
            hardware_interface::JointStateInterface _joint_state_interface;
            hardware_interface::PositionJointInterface _position_joint_interface;
            hardware_interface::EffortJointInterface _effort_joint_interface;
            hardware_interface::VelocityJointInterface _velocity_joint_interface;

            joint_limits_interface::PositionJointSaturationInterface _position_joint_saturation_interface;
            joint_limits_interface::PositionJointSoftLimitsInterface _position_joint_limit_interface;
            joint_limits_interface::EffortJointSaturationInterface _effort_joint_saturation_interface;
            joint_limits_interface::EffortJointSoftLimitsInterface _effort_joint_limit_interface;
            joint_limits_interface::VelocityJointSaturationInterface _velocity_joint_saturation_interface;
            joint_limits_interface::VelocityJointSoftLimitsInterface _velocity_joint_limit_interface;

            // Shared memory
            int _num_joints;
            int _joint_modes;
            std::vector<std::string> _joint_names;
            std::vector<double> _joint_position, _joint_position_prev;
            std::vector<double> _joint_velocity;
            std::vector<double> _joint_effort;
            std::vector<double> _joint_position_command;
            std::vector<double> _joint_velocity_command;
            std::vector<double> _joint_effort_command;

            // controller manager
            std::shared_ptr<controller_manager::ControllerManager> _controller_manager;

            // Hardware communication
            int _port;
            std::string _ip_addr;

            // ROS communication
            ros::NodeHandle _nh;
            std::string _robot_description;
            ros::Duration _control_period;
            double _control_freq;
            bool _initialization;
    };
}

#endif // WSG50_DRIVER_H