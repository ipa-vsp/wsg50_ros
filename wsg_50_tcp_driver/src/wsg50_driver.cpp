#include <wsg_50_tcp_driver/wsg50_driver.h>

#include <control_toolbox/filters.h>
#include <controller_manager/controller_manager.h>

#include <urdf/model.h>
#include <thread>

namespace iwtros
{
    wsg50::wsg50(ros::NodeHandle& nh){
        init(nh);
    }

    wsg50::~wsg50(){
        // disconnect gripper
    }

    void wsg50::init(ros::NodeHandle& nh){
        _nh = nh;
        _load_param();
        _init();
        _controller_manager.reset(new controller_manager::ControllerManager(this, _nh));

        // Establish the gripper connection       
    }

    void wsg50::_load_param(){
        ros::NodeHandle n_p("~");

        n_p.param("tcp/port", _port, 1000);
        n_p.param<std::string>("tcp/ip_addr", _ip_addr, "172.31.1.160");
        n_p.param("hardware_interface/control_freq", _control_freq, 50.);
        n_p.getParam("hardware_interface/joints", _joint_names);
    }

    void wsg50::_init(){
        _num_joints = _joint_names.size();

        // Resize vectors;
        _joint_position.resize(_num_joints);
        _joint_velocity.resize(_num_joints);
        _joint_effort.resize(_num_joints);
        _joint_position_command.resize(_num_joints);
        _joint_velocity_command.resize(_num_joints);
        _joint_effort_command.resize(_num_joints);

        urdf::Model urdf_model;
        std::string urdf_string;

        // Search and wait for robot_description on param server
        while(urdf_string.empty()){
            ROS_INFO_ONCE_NAMED("WSG50", "WSG50 driver is waiting for model"
                                        " URDF in parameter [%s] on the ROS param server.",
                                        _robot_description.c_str());
            _nh.getParam(_robot_description, urdf_string);
            usleep(100000);
        }
        ROS_INFO_STREAM_NAMED("Iiwa", "Received urdf from param server, parsing...");

        const urdf::Model* const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : nullptr;
        if (urdf_model_ptr == nullptr)
            ROS_WARN_STREAM_NAMED("Iiwa", "Could not read URDF from '" << _robot_description << "' parameters. Joint limits will not work.");

        // Initialize the controller
        for(int i = 0; i < _num_joints; i++){
            _joint_position[i] = _joint_velocity[i] = _joint_effort[i] = 0;

            // Create joint state interface
            hardware_interface::JointStateHandle joint_state_handle(_joint_names[i], &_joint_position[i], &_joint_velocity[i], &_joint_effort[i]);
            _joint_state_interface.registerHandle(joint_state_handle);
            
            // Get joint limits from URDF
            bool has_soft_limits = false;
            bool has_limits = urdf_model_ptr != nullptr;
            joint_limits_interface::JointLimits limits;
            joint_limits_interface::SoftJointLimits soft_limits;

            if(has_limits){
                auto urdf_joint = urdf_model_ptr->getJoint(_joint_names[i]);
                if(!urdf_joint){
                    ROS_WARN_STREAM_NAMED("WSG50", "Could not find joint ’" << _joint_names[i] << "’ in URDF. No limits will be added to this joint.");
                    continue;
                }
                joint_limits_interface::getJointLimits(urdf_joint, limits);
                if(joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits)) has_soft_limits = true;
            }

            // Create Position joint interface
            hardware_interface::JointHandle joint_position_handle(joint_state_handle, &_joint_position_command[i]);
            if(has_soft_limits){
                joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_handle(joint_position_handle, limits, soft_limits);
                _position_joint_limit_interface.registerHandle(joint_limit_handle);
            }else{
                joint_limits_interface::PositionJointSaturationHandle joint_limit_handle(joint_position_handle, limits);
                _position_joint_saturation_interface.registerHandle(joint_limit_handle);
            }
            _position_joint_interface.registerHandle(joint_position_handle);
        }
        registerInterface(&_joint_state_interface);
        registerInterface(&_position_joint_interface);  
    }
} // namespace iwtros
