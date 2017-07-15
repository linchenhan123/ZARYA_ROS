#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>
#include <vector>
using namespace cv;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "opencv_test");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("/webcam/image",30);
    std_msgs::Header header;
    header.seq = 0;
    header.stamp = ros::Time::now();

    Mat image;
    cv::VideoCapture cap(0);


     int iLowH = 170;
     int iHighH = 180;

     int iLowS = 43;
     int iHighS = 255;

     int iLowV = 46;
     int iHighV = 255;
     ros::Rate r(30);
    while (ros::ok())
    {

     cap >> image;
     Mat image_hsv;
     std::vector<Mat> hsvSplit;
     cv::cvtColor(image,image_hsv,CV_BGR2HSV);
     split(image_hsv,hsvSplit);
     cv::equalizeHist(hsvSplit[2],hsvSplit[2]);
     merge(hsvSplit,image_hsv);
     Mat image_thresholded;
     inRange(image_hsv,cv::Scalar(iLowH,iLowS,iLowV),cv::Scalar(iHighH,iHighS,iHighV),image_thresholded);

     Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
     // morphologyEx(image_thresholded, image_thresholded, MORPH_OPEN, element);
      morphologyEx(image_thresholded, image_thresholded, MORPH_CLOSE, element);
     header.seq ++;
     cv_bridge::CvImage cvimage(header,sensor_msgs::image_encodings::MONO8,image_thresholded);
     image_pub.publish(cvimage.toImageMsg());


     r.sleep();

    };



}
