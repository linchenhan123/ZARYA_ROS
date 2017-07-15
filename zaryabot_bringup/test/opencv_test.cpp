#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>


using namespace cv;

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "opencv_test");

  VideoCapture cap(0);

  ros::NodeHandle nh;

  ros::Rate r(10);

  Mat image;

  namedWindow("Test");

  while(ros::ok()) {
    // Read a image
    cap >> image;

    ROS_INFO("Success capture image%d",image.data[0]);

    //imshow("Test",image);

    r.sleep();
  }


  ROS_INFO("Hello world!");
}
