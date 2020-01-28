#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <wsg_50_tcp_driver/wsg50MoveAction.h>


class gripperMove{
    protected:
        ros::NodeHandle _nh;
        actionlib::SimpleActionServer<wsg_50_tcp_driver::wsg50MoveAction> _as;
        std::string _action_name;
        wsg_50_tcp_driver::wsg50MoveFeedback _feedback;
        wsg_50_tcp_driver::wsg50MoveResult _result;
    public:
        gripperMove(std::string name): _as(_nh, name, boost::bind(&gripperMove::executeCB, this, _1), false){
            _as.start();
        }

        ~gripperMove(void);

        void executeCB(const wsg_50_tcp_driver::wsg50MoveGoalConstPtr &goal){

        }
};