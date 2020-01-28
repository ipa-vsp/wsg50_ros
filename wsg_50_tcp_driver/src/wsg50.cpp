/*
* This ws50 node is created for ROS framework in the LernFabrik project under 
* IWT Wirtschaft und Technik GmbH
* This file is the modification of the old wsg50 node
* author: Vishnuprasad Prachandanhanu
* Date: 20.01.2020
*/

#include <wsg_50_tcp_driver/wsg50.h>

namespace iwtros{

    template<typename T_action, typename T_goal, typename T_result>
    void wsg50::handleError(actionlib::SimpleActionServer<T_action>* server,
                        std::function<bool(const T_goal&)> handler,
                        const T_goal& goal){
        T_result result;
        try{
            result.success = handler(goal);
            server->setSucceeded(result);
        }catch(int e){
            ROS_ERROR_STREAM("Error handling error" << e);
            result.success = false;
            result.error = e;
            server->setAborted(result);
        }
    }

    void wsg50::gripperCommandExecution(const control_msgs::GripperCommandGoalConstPtr& goal){
        auto gripper_command_handler = [goal, this](auto default_speed){
            /*HACK: Gripper motion given by the MoveIt is for one 
            finger and other should mimic respectively.
            For the single gripper position is from the midway*/
            double target_width = std::abs(2000*goal->command.position);
            if(target_width > GRIPPER_MAX_OPEN || target_width < 0.0){
                ROS_ERROR_STREAM("GripperServer: Commanding out of range with max_dith= " << GRIPPER_MAX_OPEN << "command = " << target_width);
                return false;
            }
            constexpr double kSamplePositionThreshold = 1e-4;
            if(std::abs(target_width - this->status_msg.width/1000) < kSamplePositionThreshold){
                return true;
            }
            if(target_width >= this->status_msg.width/1000){
                ROS_WARN_STREAM("Executing move command current = " <<  this->status_msg.width/1000 << " width");
                return gripperCom.move(target_width, default_speed, false, false);
            }
            return gripperCom.grasp(target_width, default_speed);

        };

        try{
            if(gripper_command_handler(this->speed)){
                if(this->status == E_SUCCESS){
                    ROS_WARN("Successful gripper action ");
                    control_msgs::GripperCommandResult result;
                    result.effort = 0.0;
                    result.position = this->status_msg.width/1000;
                    result.reached_goal = static_cast<decltype(result.reached_goal)>(true);
                    result.stalled = static_cast<decltype(result.stalled)>(false);
                    gs_.setSucceeded(result);
                    return;
                }
            }
        }catch(const std::exception& e){
            std::cerr << e.what() << '\n';
            ROS_ERROR("Failed to move the gripper");
        }
        gs_.setAborted();
    }
        
    wsg50::wsg50(ros::NodeHandle& nh, const char *addr, unsigned short port):gs_(nh, "gripper_action", boost::bind(&wsg50::gripperCommandExecution, this, _1), false), _nh(nh),
                                        gripperCom(addr, port){
        // _nh.param("ip", ip, std::string("172.31.1.160"));
        // _nh.param("port", port, 1000);
        _nh.param("local_port", local_port, 1501);
        _nh.param("com_mode", com_mode, std::string("auto_update"));
        _nh.param("rate", rate, 50.0); // With custom script, up to 30Hz are possible
        _nh.param("grasping_force", grasping_force, 40.0);
        _nh.param("speed", speed, 40.0);

        if (com_mode == "script")
            g_mode_script = true;
        else if (com_mode == "auto_update")
            g_mode_periodic = true;
        else {
            com_mode = "polling";
            g_mode_polling = true;
        }

        ROS_INFO("Connecting to %s:%d (%s); communication mode: %s ...", ip.c_str(), port, protocol.c_str(), com_mode.c_str());

        if (gripperCom.connected){
            ROS_INFO("Gripper connection stablished");
            gs_.start();

            //Publisher
            _pub_state = _nh.advertise<wsg_50_common::Status>("status", 1000);
            _pub_joint = _nh.advertise<sensor_msgs::JointState>("joint_states", 10);
            if(g_mode_script || g_mode_periodic)
                _pub_moving = _nh.advertise<std_msgs::Bool>("moving", 10);
            
            ROS_INFO("Ready to use. Homing now....");
            gripperCom.ack_fault();
            gripperCom.homing();
            gripperCom.grasp(0.0, 50);
            ros::Duration(0.5).sleep();
            if(grasping_force > 0){
                ROS_INFO("Setting grasping force limit to %5.1f", grasping_force);
                gripperCom.setGraspingForceLimit(grasping_force);
            }

            ROS_INFO("Init done. Starting timer/thread with target rate %.1f.", rate);
            std::thread th;
            if(g_mode_periodic){
                ROS_INFO("Initializing threading");
                th = std::thread(boost::bind(&wsg50::read_thread, this, _1), (int)(1000.0/rate));
            }
            th.join();
            ros::spin();

        }else{
            ROS_ERROR("Unable to connect. please check the port and ip address used");
        }
        ROS_INFO("Exiting...");
        g_mode_periodic = false;
        g_mode_script = false;
        g_mode_polling = false;
        sleep(1);
        gripperCom.disconnected();
    }

    wsg50::~wsg50(){
        g_mode_periodic = false;
        g_mode_script = false;
        g_mode_polling = false;
        sleep(1);
        gripperCom.disconnected();
        ros::shutdown();
    }


    void wsg50::read_thread(int interval_ms){
        ROS_INFO("Thread started");
        int res;
        bool pub_state = false;

        double rate_exp = 1000.0 / (double)interval_ms;
        std::string names[3] = { "opening", "speed", "force" };

        // Prepare messages
        status_msg.status = "UNKNOWN";

        sensor_msgs::JointState joint_states;
        joint_states.header.frame_id = "";
        joint_states.name.push_back("wsg50_finger_left_joint");
        joint_states.name.push_back("wsg50_finger_right_joint");
        joint_states.position.resize(2);
        joint_states.velocity.resize(2);
        joint_states.effort.resize(2);

        // Request automatic updates (error checking is done below)
        gripperCom.getOpening(interval_ms);
        gripperCom.getSpeed(interval_ms);
        gripperCom.getForce(interval_ms);


        msg_t msg; msg.id = 0; msg.data = 0; msg.len = 0;
        int cnt[3] = {0,0,0};
        auto time_start = std::chrono::system_clock::now();


        while (g_mode_periodic) {
            // Receive gripper response
            gripperCom._msg->_delete(&msg);
            res = gripperCom._msg->receive( &msg );
            if (res < 0 || msg.len < 2) {
                ROS_ERROR("Gripper response failure");
                continue;
            }

            float val = 0.0;
            status = gripperCom.get_response_status(msg.data);

            // Decode float for opening/speed/force
            if (msg.id >= 0x43 && msg.id <= 0x45 && msg.len == 6) {
                if (status != E_SUCCESS) {
                    ROS_ERROR("Gripper response failure for opening/speed/force\n");
                    continue;
                }
                val = gripperCom.convert(&msg.data[2]);
            }

            // Handle response types
            int motion = -1;
            switch (msg.id) {
                /*** Opening ***/
                case 0x43:
                    status_msg.width = val;
                    pub_state = true;
                    cnt[0]++;
                    break;

                /*** Speed ***/
                case 0x44:
                    status_msg.speed = val;
                    cnt[1]++;
                    break;

                /*** Force ***/
                case 0x45:
                    status_msg.force = val;
                    cnt[2]++;
                    break;

                /*** Move ***/
                // Move commands are sent from outside this thread
                case 0x21:
                    if (status == E_SUCCESS) {
                        ROS_INFO("Position reached");
                        motion = 0;
                    } else if (status == E_AXIS_BLOCKED) {
                        ROS_INFO("Axis blocked");
                        motion = 0;
                    } else if (status == E_CMD_PENDING) {
                        ROS_INFO("Movement started");
                        motion = 1;
                    } else if (status == E_ALREADY_RUNNING) {
                        ROS_INFO("Movement error: already running");
                    } else if (status == E_CMD_ABORTED) {
                        ROS_INFO("Movement aborted");
                        motion = 0;
                    } else {
                        ROS_INFO("Movement error");
                        motion = 0;
                    }
                    break;

                /*** Stop ***/
                // Stop commands are sent from outside this thread
                case 0x22:
                    // Stop command; nothing to do
                    break;
                default:
                    ROS_INFO("Received unknown respone 0x%02x (%2dB)\n", msg.id, msg.len);
            }

            // ***** PUBLISH motion message
            if (motion == 0 || motion == 1) {
                std_msgs::Bool moving_msg;
                moving_msg.data = motion;
                _pub_moving.publish(moving_msg);
                g_ismoving = motion;
            }

            // ***** PUBLISH state message & joint message
            if (pub_state) {
                pub_state = false;
                _pub_state.publish(status_msg);

                joint_states.header.stamp = ros::Time::now();;
                joint_states.position[0] = -status_msg.width/2000.0;
                joint_states.position[1] = status_msg.width/2000.0;
                joint_states.velocity[0] = status_msg.speed/1000.0;
                joint_states.velocity[1] = status_msg.speed/1000.0;
                joint_states.effort[0] = status_msg.force;
                joint_states.effort[1] = status_msg.force;
                _pub_joint.publish(joint_states);
            }

            // Check # of received messages regularly
            std::chrono::duration<float> t = std::chrono::system_clock::now() - time_start;
            double t_ = t.count();
            if (t_ > 5.0) {
                time_start = std::chrono::system_clock::now();
                //printf("Infos for %5.1fHz, %5.1fHz, %5.1fHz\n", (double)cnt[0]/t_, (double)cnt[1]/t_, (double)cnt[2]/t_);

                std::string info = "Rates for ";
                for (int i=0; i<3; i++) {
                    double rate_is = (double)cnt[i]/t_;
                    info += names[i] + ": " + std::to_string((int)rate_is) + "Hz, ";
                    if (rate_is == 0.0)
                        ROS_ERROR("Did not receive data for %s", names[i].c_str());
                }
                ROS_DEBUG_STREAM((info + " expected: " + std::to_string((int)rate_exp) + "Hz").c_str());
                cnt[0] = 0; cnt[1] = 0; cnt[2] = 0;
            }
            ros::spinOnce();
        }

        // Disable automatic updates
        // TODO: The functions will receive an unexpected response
        gripperCom.getOpening(0);
        gripperCom.getSpeed(0);
        gripperCom.getForce(0);

        ROS_INFO("Thread ended");
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "wsg50_gripper");
    ros::NodeHandle nh("~");

    iwtros::wsg50 wsg(nh, "172.16.17.160", 1000);

    ros::spin();
}