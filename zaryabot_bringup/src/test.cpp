#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<ros/ros.h>
using namespace cv;

Mat cap_src;
Mat cap_gray;
Mat cap_thre;
Mat ROI1;
Mat ROI2;
void msgtocv(sensor_msgs::Image cap_msg);
int main(int argc,char **argv)
{
    ros::init(argc,argv,"test");
    ros::NodeHandle nh_;
    ros::Subscriber sub = nh_.subscribe("/webcam/image",1,msgtocv);

    cvtColor(cap_src,cap_gray,CV_BGR2GRAY);
    threshold(cap_gray,cap_thre,125,255,cv::THRESH_BINARY);


    ROI1 = cap_thre(Rect(0,(cap_thre.rows/2) -240,cap_thre.cols,70 ));
    ROI2 = cap_thre(Rect(0,(cap_thre.rows/2) +170,cap_thre.cols,70 ));

    ROS_INFO("%d,%d,%d",ROI1.cols,ROI2.rows,cap_src.type());

    ros::spin();
}
void msgtocv(sensor_msgs::Image cap_msg)
{
    cv_bridge::CvImagePtr ipt = cv_bridge::toCvCopy(cap_msg);
    cap_src = ipt->image.clone();



}
