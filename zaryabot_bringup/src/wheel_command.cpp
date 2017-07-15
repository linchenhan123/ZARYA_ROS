#include <ros/ros.h>
#include "robottrancmd.h"

//#include <boost/thread.hpp>
//#include <boost/chrono.hpp>


//void publish_ros_topic_thread() {
//    while(ros::ok()) {
//        wheel_pub.publish(msg);
//        ros::Rate r(10);
//        r.sleep();
//    }
//}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wheel_command");
    ros::NodeHandle nh;

    RobotTranCMD::RobotWheelGeoDef geo(0.5,0.5,0.2);

    RobotTranCMD pub(nh, "output","/cmd/vel",geo);
    ROS_INFO("Hello world!");
//    ros::Rate r(10);
    ros::Rate r(30);


    while(ros::ok()){
        pub.publishJointCMD();

        ros::spinOnce();
        r.sleep();
    }

}
