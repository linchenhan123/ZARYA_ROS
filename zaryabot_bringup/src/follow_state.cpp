#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <zaryabot_bringup/working_state.h>
#include <geometry_msgs/Twist.h>
#include <zaryabot_bringup/zarya.h>

#define H_max1 180
#define H_min1 165
#define S_max1 255
#define S_min1 100
#define V_max1 255
#define V_min1 120

#define H_max2 10
#define H_min2 0
#define S_max2 255
#define S_min2 100
#define V_max2 255
#define V_min2 120

#define VELOCITY_OF_STARTING_UP 100
#define TIME_DURATION_OF_STARTING_UP 1
#define MAX_ERROR_ACCEPTABLE_X 10
#define MAX_ERROR_ACCEPTABLE_Y 10

using namespace cv;

//method initialization
void Callback(const sensor_msgs::Image &cap_msg);
void Err_Callback(const zaryabot_bringup::zarya &err);
int bSums(Mat src);
void number_Callback(const zaryabot_bringup::zarya &num);


//CV initialization
Mat cap_src;        //source image
Mat cap_hsv;        //hsv image using for detect red color
Mat hsv_thre1;
Mat hsv_thre2;
Mat front_ROI;   //ROI using for counting red pixel
int number;      //number of red pixel
int number_to_punch_(0);    //the number to punch


//robot follow state initialization

bool mode(true);
bool BUTTON_ON(false);
bool incenter(false);                       //whether in center or not
bool getnumber_to_punch(false);             //whether get the number ro not



ros::ServiceClient line_following_client;
ros::ServiceClient red_point_client;
ros::ServiceClient number_client;
ros::Publisher pub;
zaryabot_bringup::working_state srv;
geometry_msgs::Twist vel;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_state");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/webcam/image",1,Callback);
    ros::Subscriber sub2 = nh.subscribe("/cmd/err",1,Err_Callback);
    ros::Subscriber sub3 = nh.subscribe("/number",0,number_Callback);

    pub = nh.advertise<geometry_msgs::Twist>("/cmd/vel",1);
    red_point_client = nh.serviceClient<zaryabot_bringup::working_state>("red_point");
    line_following_client = nh.serviceClient<zaryabot_bringup::working_state>("line_following");
    number_client = nh.serviceClient<zaryabot_bringup::working_state>("number_detect");
    ros::spin();

}
void Callback(const sensor_msgs::Image &cap_msg)
{
    cv_bridge::CvImagePtr ipt  = cv_bridge::toCvCopy(cap_msg);
    cap_src = ipt->image.clone();

    cvtColor(cap_src,cap_hsv,CV_BGR2HSV);
    inRange(cap_hsv,Scalar(H_min1,S_min1,V_min1),Scalar(H_max1,S_max2,V_max2),hsv_thre1);
    inRange(cap_hsv,Scalar(H_min2,S_min2,V_min2),Scalar(H_max2,S_max2,V_max2),hsv_thre2);

    Mat red = Mat::zeros(cap_src.rows,cap_src.cols,CV_8UC1);
    red = hsv_thre1 + hsv_thre2;
    front_ROI = red(Rect(0,0,cap_src.cols,cap_src.rows/4));


    number = bSums(front_ROI);
    //ROS_INFO("number:%d",number);





/****************************************/
/****************************************/
/*       ROS_ROROT_FOLLOW_STATE         */



    static int state(5);



     switch (state)
  {
    case 0:                          //press the button to passing to next state
     {
         if(BUTTON_ON)
         {
             state++;
             ROS_INFO("!!!start!!!\n\n\n");
         }
         else
         {
             ROS_INFO("waiting for being enabled");
         }

        break;
     }



    case 1:
     {
        ROS_INFO("IN STATE1\n\n\n");



        vel.linear.x = VELOCITY_OF_STARTING_UP;
        vel.linear.y = 0;
        vel.linear.z = 0;
        pub.publish(vel);
        ROS_INFO("starting ZARYA...\n\n");
        ros::Duration(TIME_DURATION_OF_STARTING_UP).sleep();

        state++;

        break;
     }

    case 2:
     {

         ROS_INFO("IN STATE2\n\n\n");

        srv.request.enable = true;

        if(line_following_client.call(srv))
        {
            ROS_INFO("successfully asked line_following_server to work");
            state++;           
        }
        else
        {
            ROS_ERROR("failed to communicate with line_following_server");
        }



        break;
    }

    case 3:
    {
        ROS_INFO("IN STATE3");
        ROS_INFO("line following...\n\n\n");
        ROS_INFO("number of red_pixel : %d",number);
        if(number >= 10000)
        {
            srv.request.enable = false;
            if(line_following_client.call(srv))
            {
                ROS_INFO("successfully stop line_following\n\n\n");
                srv.request.enable = true;
                if(red_point_client.call(srv))
                {
                    ROS_INFO("successfully asked red_pointserver to work");
                    state++;

                }
                else
                {
                    ROS_ERROR("failed to communicate with red_point_server");
                }
            }
            else
            {
                ROS_ERROR("failed to communicate with line_following_server");

            }




        }

        break;
    }
    case 4:
    {
         ROS_INFO("IN STATE 4~");
         ROS_INFO("finding red center......\n\n\n");
         if(incenter)
         {
             ROS_INFO("!!!incenter!!!");
             srv.request.enable = false;
             if(red_point_client.call(srv))
             {
                 ROS_INFO("successfully stop red_point_server");
                 vel.linear.x = 0;
                 vel.linear.y = 0;
                 vel.linear.z = 0;
                 pub.publish(vel);

                 state++;

             }
             else
             {
                 ROS_ERROR("failed to communicate with red_point_server");
             }

         }

         break;
    }
    case 5:
    {
         ROS_INFO("IN STATE5~\n\n\n");
         ros::Duration(1.0).sleep();
         srv.request.enable = true;
         if(number_client.call(srv))
         {
             ROS_INFO("successfully asked number_detect_server to work");
             state++;


         }
         else
         {
             ROS_ERROR("failed to communicate with number_detect_server");
         }




         break;

    }
    case 6:
    {
        ROS_INFO("IN STATE6~");
        ROS_INFO("finding number......\n\n\n");
        if(getnumber_to_punch)
        {
           ROS_INFO("number: %d",number_to_punch_);
           state++;
        }

        break;

    }
    case 7:
    {
        ROS_INFO("IN STATE7~\n\n\n");
        srv.request.enable = true;
        if(line_following_client.call(srv))
        {
            ROS_INFO("successfully acked line_following_server to work");
            state++;
        }
        else
            ROS_ERROR("failed to communicate with line_following_server");


        break;

    }
    case 8:
    {
         ROS_INFO("IN STATE8\n\n\n");
         static int counter(0);
         if(number > 10000 && mode)
         {
             counter++;
             mode = false;
         }
         if(number < 1000)
             mode = true;

         ROS_INFO("counter: %d",counter);

         switch(number_to_punch_)
         {
             case 3:
             case 6:
             case 9:
             {
                 if(counter == 1)
                 {
                     ROS_INFO("STOP!!!");
                     srv.request.enable = false;
                     if(line_following_client.call(srv))
                     {
                         ROS_INFO("successfully stop line_following_server");
                         srv.request.enable = true;
                         if(red_point_client.call(srv))
                         {
                             ROS_INFO("successfully asked red_point_server to work");
                             state++;
                         }
                         else
                             ROS_ERROR("failed to communicate with red_point_server");
                     }
                     else
                         ROS_ERROR("failed to communicate with line_following_server");

                 }
                 break;
             }
             case 2:
             case 5:
             case 8:
             {
                 if(counter == 2)
                 {
                     ROS_INFO("STOP!!!");
                     srv.request.enable = false;
                     if(line_following_client.call(srv))
                     {
                         ROS_INFO("successfully stop line_following_server");
                         srv.request.enable = true;
                         if(red_point_client.call(srv))
                         {
                             ROS_INFO("successfully asked red_point_server to work");
                             state++;
                         }
                         else
                             ROS_ERROR("failed to communicate with red_point_server");
                     }
                     else
                         ROS_ERROR("failed to communicate with line_following_server");
                 }
                 break;
             }
             case 1:
             case 4:
             case 7:
             {
                 if(counter == 3)
                 {
                     ROS_INFO("STOP!!!");
                     srv.request.enable = false;
                     if(line_following_client.call(srv))
                     {
                         ROS_INFO("successfully stop line_following_server");
                         srv.request.enable = true;
                         if(red_point_client.call(srv))
                         {
                             ROS_INFO("successfully asked red_point_server to work");
                             state++;
                         }
                         else
                             ROS_ERROR("failed to communicate with red_point_server");
                     }
                     else
                         ROS_ERROR("failed to communicate with line_following_server");
                 }
                 break;
             }

         default :
             ROS_ERROR("have problem finding number!!!");
         }
         break;


    }
    case 9:
    {
         ROS_INFO("IN STATE9\n\n\n");
         break;

    }



    default:
        ROS_ERROR("ERROR! No case has been satisfied!");//no case has been satisfied ,
        break;
 }














}
void Err_Callback(const zaryabot_bringup::zarya &err)
{
    ROS_INFO("red_err_y: %d",err.error_y);
    ROS_INFO("red_err_x: %d",err.error_x);
    if(abs(err.error_x) <= MAX_ERROR_ACCEPTABLE_X && abs(err.error_y) <= MAX_ERROR_ACCEPTABLE_Y )
        incenter = true;
}
void number_Callback(const zaryabot_bringup::zarya &num)
{
    number_to_punch_ = num.number_to_punch;
    if(number_to_punch_ != 0)
        getnumber_to_punch = true;
}
int bSums(Mat src)
{

    int counter = 0;
    //迭代器访问像素点
    Mat_<uchar>::iterator it = src.begin<uchar>();
    Mat_<uchar>::iterator itend = src.end<uchar>();
    for (; it!=itend; ++it)
    {
        if((*it)>0) counter+=1;//二值化后，像素点是0或者255
    }
    return counter;
}

