#include<ros/ros.h>
#include<std_msgs/Int8.h>


int main(int argc , char** argv)
{
    ros::init(argc,argv,"test_pub");
    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<std_msgs::Int8>("/number",0);
    ros::Rate rate(30);
    while(ros::ok()){

        for(int i = 0 ; i < 100 ; i++)
        {
            std_msgs::Int8 int8;
            int8.data = i;
            ros::spinOnce();
            publisher.publish(int8);
            rate.sleep();
        }
    }
    return 0;
}
