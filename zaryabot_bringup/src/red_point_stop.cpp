#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<geometry_msgs/Twist.h>
#include<dynamic_reconfigure/server.h>
#include<zaryabot_bringup/parameterConfig.h>
#include<zaryabot_bringup/working_state.h>
#include<zaryabot_bringup/zarya.h>



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
#define max_error 5

using namespace cv;

Mat cap_src;
Mat cap_hsv;
Mat hsv_thre1;
Mat hsv_thre2;
Mat red;

CvPoint red_center;

double kpy;
double kpx;
double kdx;
double kdy;
bool start(false);
bool in_center(false);

void Callback(const sensor_msgs::Image &cap_msg);
void calculate_publish(const CvPoint &red_center);
void ConfigCB(zaryabot_bringup::parameterConfig &t , uint32_t level);
bool working(zaryabot_bringup::working_state::Request &req, zaryabot_bringup::working_state::Response &res);
CvPoint Getcenter(Mat src);

ros::Subscriber subscriber ;
ros::Publisher publisher;
ros::Publisher publisher2;




int main(int argc,char **argv)
{
//ros initializing
    ros::init(argc,argv,"red_center_node");
    ros::NodeHandle nh_;
    ros::ServiceServer service = nh_.advertiseService("red_point",working);

    dynamic_reconfigure::Server<zaryabot_bringup::parameterConfig> server;
    dynamic_reconfigure::Server<zaryabot_bringup::parameterConfig>::CallbackType f;
    f = boost::bind(&ConfigCB ,_1,_2);

    server.setCallback(f);

    subscriber = nh_.subscribe("/webcam/image",1,Callback);
    publisher = nh_.advertise<geometry_msgs::Twist>("/cmd/vel",1);
    publisher2 = nh_.advertise<zaryabot_bringup::zarya>("/cmd/err",1);
    ros::spin();
}

bool working(zaryabot_bringup::working_state::Request &req,zaryabot_bringup::working_state::Response &res)
{

    start = req.enable;

    if(start)
    {
        ROS_INFO("WORKING~");
    }
    else
    {
        ROS_INFO("SHUT DOWN!");
    }
    res.working = start;

}
void ConfigCB(zaryabot_bringup::parameterConfig &t, uint32_t level)
{
    kpy = t.kp_forward_redpoint;
    kpx = t.kp_offset_redpoint;
    kdy = t.kd_forward_redpoint;
    kdx = t.kd_offset_redpoint;

}




void Callback(const sensor_msgs::Image &cap_msg)
{
    if(start) {

     cv_bridge::CvImagePtr ipt = cv_bridge::toCvCopy(cap_msg);
     cap_src = ipt->image.clone();
     //ROS_INFO("%d,%d",cap_src.rows,cap_src.cols);


     cvtColor(cap_src,cap_hsv,CV_BGR2HSV);

   //thresholding
    cv::inRange(cap_hsv,Scalar(H_min1,S_min1,V_min1),Scalar(H_max1,S_max1,V_max1),hsv_thre1);
    cv::inRange(cap_hsv,Scalar(H_min2,S_min2,V_min2),Scalar(H_max2,S_max2,V_max2),hsv_thre2);
     red= Mat::zeros(cap_src.rows,cap_src.cols,CV_8UC1);
     red = hsv_thre1 + hsv_thre2;

     red_center = Getcenter(red);


     calculate_publish(red_center);
    }


}

CvPoint Getcenter(Mat src)
{

  CvPoint center;

  int xc =0 ,yc = 0,sum =0;
  for (int i = 0 ;i < src.rows; i++)
  {
      for(int j = 0;j < src.cols; j++)
      {
          if(src.at<uchar>(i,j) > 125)
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

void calculate_publish(const CvPoint &red_center)
{
    geometry_msgs::Twist  vel;
    zaryabot_bringup::zarya err;
    //ROS_INFO("%d,%d",red_center.x,red_center.y);
    int err_y = cap_src.rows/2 - red_center.y;
    int err_x = red_center.x - cap_src.cols/2;


    static int err_y_lf(3);
    static int err_x_lf(3);

    double v_forward = kpy * err_y  + kdy * (err_y - err_y_lf) ;
    double v_offset = kpx * err_x + kdx * (err_x - err_x_lf);
    int d = err_y - err_y_lf;
   // ROS_INFO("err_y - err_y_lf: %d",d);

    vel.linear.x = v_forward;
    vel.linear.y = v_offset;
    vel.linear.z = 0;
    err.error_x = err_y;
    err.error_y = err_x;

    publisher.publish(vel);
    publisher2.publish(err);

    err_y_lf = err_y;
    err_x_lf = err_x;



}






