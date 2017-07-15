#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <modbus/modbus-rtu.h>

#define MOTOR_REG_ADDR_VELOCITY (100)
#define MODBUS_SLAVE_ID (10)

// Setup modbus
modbus_t *ctx;

void vel_command_callback(const geometry_msgs::TwistConstPtr& msg);

int main(int argc, char **argv)
{
  /*************** Set up ROS. *****************/
  ros::init(argc, argv, "modbus_exampe_node");
  ros::NodeHandle nh;


  /*********************************************/



  /*************** Setup modbus ****************/


  ctx = modbus_new_rtu("/dev/ttyACM1", 9600, 'N', 8, 1);

  modbus_set_debug(ctx, true);

  if (ctx == NULL) {
      ROS_ERROR("Can't create MODBUS context");
      return -1;
  }

  modbus_set_slave(ctx, MODBUS_SLAVE_ID);

  modbus_connect(ctx);




  /*********************************************/


  ROS_INFO("Hello world!");

  modbus_write_register(ctx, MOTOR_REG_ADDR_VELOCITY, 180);

  ros::Subscriber sub = nh.subscribe("/vel_cmd", 100, vel_command_callback);

  ros::spin();



  modbus_free(ctx);

  return 0;
}

void vel_command_callback(const geometry_msgs::TwistConstPtr& msg) {
  int dot_theta = static_cast<int>(msg->angular.z);

  modbus_write_register(ctx, MOTOR_REG_ADDR_VELOCITY, dot_theta);
}
