#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <zaryabot_bringup/parameterConfig.h>
#include <zaryabot_bringup/working_state.h>

using namespace cv;
using namespace std;


CvPoint Getcenter(Mat src);                             //method of geting center point
void msg_to_cvimg(sensor_msgs::Image cap_msg);
void ConfigCB(zaryabot_bringup::parameterConfig &t ,uint32_t level);    //method of tranfer rosmsg into cvimage
bool working (zaryabot_bringup::working_state::Request &req, zaryabot_bringup::working_state::Response &res);

ros::Publisher vel_pub;
ros::Subscriber cap_sub;

bool start(false);

Mat cap_src;
Mat cap_thre;
Mat cap_gray;                 //Mat declear


Mat ROI1;
Mat ROI2;                     //ROI


CvPoint c_ROI1;
CvPoint c_ROI2;               //center of each ROI

CvPoint c1;
CvPoint c2;                   //center of each ROI in whole image

int c12_x;                    //average of c1 c2 in x direction using for catculating
int c12_y;
int error_x;
double yaw;

double v_offset;
double v_forward;
double v_yaw;                //output parameter


double kpx;
double kdx;
int kpz;                      //value of proportion controller
int kdz;



int main(int argc,char **argv)
{
//setting region



//ros initialize
    ros::init(argc,argv,"vel_publisher");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("line_following",working);
    dynamic_reconfigure::Server<zaryabot_bringup::parameterConfig> server;
    dynamic_reconfigure::Server<zaryabot_bringup::parameterConfig>::CallbackType f;
    f = boost::bind(&ConfigCB, _1,_2);
    server.setCallback(f);

    cap_sub = n.subscribe("/webcam/image",1,msg_to_cvimg);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd/vel",1);
    ros::spin();


}

bool working(zaryabot_bringup::working_state::Request &req, zaryabot_bringup::working_state::Response &res)
{

   start = req.enable;
   if(start)
   {
       ROS_INFO("WORKING~");
   }
   else
   {
       ROS_INFO("SHUT DOWN");
   }

   res.working = start;

}
CvPoint Getcenter(Mat src)
{

  CvPoint center;

  int xc =0 ,yc = 0,sum =0;
  for (int i = 0 ;i < src.rows; i++)
  {
      for(int j = 0;j < src.cols; j++)
      {
          if(src.at<uchar>(i,j) < 125)
          {
              xc = xc + j;
              yc = yc + i;
              sum = sum + 1;
          }
      }


  }
  if(sum == 0)
  {
       center.x = 0;
       center.y = 0;
       return center;
  }
  center.x = xc/sum;
  center.y = yc/sum;
  return center;



}
void msg_to_cvimg(sensor_msgs::Image cap_msg)
{
    if(start)
    {
    cv_bridge::CvImagePtr ipt = cv_bridge::toCvCopy(cap_msg);
    cap_src = ipt->image.clone();

    //ROS_INFO("rows:%d,cols:%d",cap_src.rows,cap_src.cols);
    cvtColor(cap_src,cap_gray,CV_BGR2GRAY);
    threshold(cap_gray,cap_thre,70,255,cv::THRESH_BINARY);


    ROI1 = cap_thre(Rect(0,(cap_thre.rows/10),cap_thre.cols,cap_thre.rows/10));
    ROI2 = cap_thre(Rect(0,(8*cap_thre.rows/10) ,cap_thre.cols,cap_thre.rows/10));

    c_ROI1 = Getcenter(ROI1);
    c_ROI2 = Getcenter(ROI2);
    c1.x = c_ROI1.x;
    c1.y = c_ROI1.y + cap_thre.rows/10;
    c2.x = c_ROI2.x;
    c2.y = c_ROI2.y + 8*cap_thre.rows/10;

    c12_x = (c1.x + c2.x)/2;


    error_x =c12_x - cap_thre.cols/2;
    yaw = atan2((c2.x - c1.x),(c2.y - c1.y));


    static int error_x_lf(error_x);
    static int yaw_lf(yaw);

    v_offset = kpx * error_x + kdx * (error_x - error_x_lf);

    v_yaw = kpz * yaw + kdz * (yaw - yaw_lf);




    geometry_msgs::Twist vel;
    vel.linear.x = v_forward;
    vel.linear.y = v_offset;
    vel.linear.z = 0;

    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = v_yaw;
     //publish messages
    vel_pub.publish(vel);

    error_x_lf = error_x;
    yaw_lf = yaw;
    }

}
void ConfigCB(zaryabot_bringup::parameterConfig &t ,uint32_t level)
{
    kpx = t.kp_offset_line;
    kpz = t.kp_angular_line;
    v_forward = t.velocity_forward_line;
    kdx = t.kd_offset_line;
    kdz = t.kd_angular_line;

}




            

            
            

   
