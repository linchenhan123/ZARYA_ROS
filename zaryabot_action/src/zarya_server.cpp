#include<ros/ros.h>
#include<actionlib/server/simple_action_server.h>
#include<zaryabot_action/AutoMissionAAction.h>

class ZaryaAction
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<zaryabot_action::AutoMissionAAction> as_;
    std::string action_name_;
    
    zaryabot_action::AutoMissionAFeedback feedback_;
    zaryabot_action::AutoMissionAResult_ result_;

    
public:
    
    ZaryaAction(std::string name) :
    as_(nh_,name,boost::bind(&ZaryaAction::executeCB,this, _1),false),
      action_name_(name)
    {
        as_.start();
    }
    ~ZaryaAction()
    {
        
    }
    void executeCB(const zaryabot_action::AutoMissionAGoalConstPtr &goal)
    {
        ros::Rate r(1);
        bool success = true;

        
        
    }
}
