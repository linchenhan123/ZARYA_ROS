#ifndef ROBOTTRANCMD_H
#define ROBOTTRANCMD_H

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <zaryabot_bringup/wheelCMD.h>

// Look this : http://www.gmii.tw/makeblock/4

#define JOINT_COUNT (4)

class RobotTranCMD {
public:
  // Nested Class
  class RobotWheelGeoDef {
    public:
    RobotWheelGeoDef(double center2lr, double center2fb, double wheelRadius)
      : a(center2lr),
        b(center2fb),
        radius(wheelRadius)
    {}

    double a;
    double b;
    double radius;
  };

public:
  RobotTranCMD(ros::NodeHandle& pNode, std::string pub_topic, std::string sub_topic, RobotWheelGeoDef& rbDef)
    : masterNH(&pNode), rbGeo(rbDef), fgMotorEN(false) {
    // initialize command message
    for(int i=0; i< JOINT_COUNT; i++) {
      command.jointid.push_back(static_cast<uint8_t>(i));
      command.mode.push_back(zaryabot_bringup::wheelCMD::MODE_STOP);
      command.position.push_back(0.0);
      command.velocity.push_back(0.0);
    }

    // Create publisher
    jointCmdPub = masterNH->advertise<zaryabot_bringup::wheelCMD>(pub_topic,10);
    rbMoveCmdSub = masterNH->subscribe(sub_topic, 10, &RobotTranCMD::updateMoveCMD,this);
  }

  void enableMotor(bool isEnable) {
    fgMotorEN = isEnable;
  }

  void breakMotor(bool isBreak) {
    fgMotorBreak = isBreak;
  }

  void publishJointCMD() {

//    if(fgMotorEN) {
//      // TODO set command.mode[0 ~ 3] to cw or ccw
//      for(int i=0; i<JOINT_COUNT; i++ ){
//        command.mode[i] = zaryabot_bringup::wheelCMD::MODE_EN;
//      }
//    }
//    else {
//      for(int i=0; i<JOINT_COUNT; i++) {
//        command.mode[i] = zaryabot_bringup::wheelCMD::MODE_STOP;
//      }
//    }

//    if(fgMotorBreak) {
//      for(int i=0; i<JOINT_COUNT; i++) {
//        command.mode[i] = zaryabot_bringup::wheelCMD::MODE_BRAKE;
//      }
//    }

    for(int i=0; i< JOINT_COUNT; i++){
        if(command.velocity[i] != 0){
            command.mode[i] = zaryabot_bringup::wheelCMD::MODE_EN;
        }
        else{command.mode[i] = zaryabot_bringup::wheelCMD::MODE_STOP;}

//        if(fgMotorBreak){
//            command.mode[i] = zaryabot_bringup::wheelCMD::MODE_BRAKE;
//        }
    }

    jointCmdPub.publish(command);
  }

private:
  void updateMoveCMD(const geometry_msgs::TwistConstPtr& msg) {
    double v_x = msg->linear.y;
    double v_y = msg->linear.x;
    double v_yaw = msg->angular.z;

    double v1 = v_y - v_x + v_yaw*(rbGeo.a + rbGeo.b);
    double v2 = v_y + v_x - v_yaw*(rbGeo.a + rbGeo.b);
    double v3 = v_y - v_x - v_yaw*(rbGeo.a + rbGeo.b);
    double v4 = v_y + v_x + v_yaw*(rbGeo.a + rbGeo.b);

    command.velocity[0] = v1;
    command.velocity[1] = v2;
    command.velocity[2] = v3;
    command.velocity[3] = v4;

    ROS_INFO("%10.5f , %10.5f , %10.5f , %10.5f",command.velocity[0],command.velocity[1],command.velocity[2],command.velocity[3]);
    // TODO translate v1 ~ v4 into Joint velocity(rad/sec)
  }

private:
  ros::NodeHandle* masterNH;
  zaryabot_bringup::wheelCMD command;
  ros::Publisher jointCmdPub;
  ros::Subscriber rbMoveCmdSub;
  RobotWheelGeoDef rbGeo;
  bool fgMotorEN;
  bool fgMotorBreak;

};

//int main(int argc, char **argv)
//{


//  ros::init(argc, argv, "wheel_test");
//  ros::NodeHandle nh;


//  RobotTranCMD::RobotWheelGeoDef geo(0.5,0.5,0.2);

//  RobotTranCMD pub(nh,"output", "input", geo);
//  ROS_INFO("Hello world!");
//  ros::Rate r(10);

//  pub.enableMotor(true);

//  while(ros::ok()) {

//    pub.publishJointCMD();

//    ros::spinOnce();
//  }

//}

#endif // ROBOTTRANCMD_H
