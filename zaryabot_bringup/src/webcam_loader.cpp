#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <std_msgs/Header.h>


namespace enc = sensor_msgs::image_encodings;

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "webcam_loader");

  // ROS main node
  ros::NodeHandle nh("webcam_loader");

  image_transport::ImageTransport it(nh);

  image_transport::Publisher
      image_pub_ =
      it.advertise("/webcam1", 1);


  std_msgs::Header header;
  header.seq = 0;
  header.stamp = ros::Time::now();

  cv::Mat image;

  cv_bridge::CvImage ros_image(header, enc::BGR8, image);

  cv::VideoCapture cap(0);


  ros::Rate r(30);
  while(ros::ok()) {

    ros_image.header.seq++;
    ros_image.header.stamp = ros::Time::now();

    cap >> ros_image.image;

    image_pub_.publish(ros_image.toImageMsg());

    r.sleep();
  }


  ROS_INFO("Hello world!");
}
