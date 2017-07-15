#include "ros/ros.h"
#include "robottrancmd.h"
#include "modbus/modbus-rtu.h"

#define MODBUS_SLAVE_ID (10)
#define WHEEL_COMMAND (100)

//setup modbus
modbus_t *ctx;

//variable declear
int v1 , v2 , v3 , v4;
int m1 , m2 , m3 , m4;



void WheelCallback(const zaryabot_bringup::wheelCMDConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wheel_sub");
    ros::NodeHandle nh;

    /*Setup Modbus****************************************/
    ctx = modbus_new_rtu("/dev/ttyUSB0", 57600 ,'N', 8 , 1);

//arduino also

    modbus_set_debug(ctx,true);

    modbus_set_slave(ctx,MODBUS_SLAVE_ID);

    modbus_connect(ctx);

    if(ctx == NULL){
        ROS_ERROR("Can't create MODBUS context");
        return -1;
    }
    /******************************************************/


    ros::Subscriber sub = nh.subscribe("output", 10, WheelCallback);
    ros::Rate looprate(30);
    while(ros::ok())
    {
        ros::spinOnce();


        const uint16_t value[8] = {v1 ,v2 ,v3, v4, m1, m2, m3, m4};
        modbus_write_registers(ctx,WHEEL_COMMAND,8,value);
        looprate.sleep();
     // usleep(0.04 * 1000000);
    }

    modbus_free(ctx);

    return 0;
}

// use constant rate publish

void WheelCallback(const zaryabot_bringup::wheelCMDConstPtr& msg)
{

    ROS_INFO("Velocity: %10.5f , %10.5f , %10.5f , %10.5f", msg->velocity[0],msg->velocity[1],msg->velocity[2],msg->velocity[3]);
    ROS_INFO("Mode: %d , %d , %d , %d",msg -> mode[0],msg -> mode[1],msg -> mode[2],msg -> mode[3]);

//    int v1 = static_cast <int>(msg ->velocity[0]);
//    int v2 = static_cast <int>(msg ->velocity[1]);
//    int v3 = static_cast <int>(msg ->velocity[2]);
//    int v4 = static_cast <int>(msg ->velocity[3]);
     v1 = static_cast <int>(msg ->velocity[0]);
     v2 = static_cast <int>(msg ->velocity[1]);
     v3 = static_cast <int>(msg ->velocity[2]);
     v4 = static_cast <int>(msg ->velocity[3]);
     m1 = static_cast <int>(msg ->mode[0]);
     m2 = static_cast <int>(msg ->mode[1]);
     m3 = static_cast <int>(msg ->mode[2]);
     m4 = static_cast <int>(msg ->mode[3]);

//    usleep(0.04 * 1000000);

//    const uint16_t value[8] = {v1 ,v2 ,v3, v4, m1, m2, m3, m4};
//    modbus_write_registers(ctx,WHEEL_COMMAND,8,value);
}
