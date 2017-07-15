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
  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub_ = it.advertise("/webcam/image", 10);

  ros::Rate r(10);

  Mat image;
  Mat gray_image;
  //namedWindow("Test");
  std_msgs::Header head;
  head.seq=0;
  while(ros::ok()) {
    // Read a image
    cap >> image;
    cv::cvtColor(image, gray_image, CV_BGR2HSV);
    cv::cvtColor(image, image, CV_BGR2HSV);

    head.seq++;
    head.stamp = ros::Time::now();

    Mat hsv[3];   //destination array
    split(image,hsv);//split source

    cv_bridge::CvImage ros_image(head, sensor_msgs::image_encodings::BGR8, gray_image);
    image_pub_.publish( ros_image.toImageMsg());

    ROS_INFO("Success capture image%d",image.data[0]);

    //imshow("Test",image);

    r.sleep();
  }


  ROS_INFO("Hello world!");
}
