#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>

#include <iostream>
#include <stdio.h>
using namespace  std;
using namespace  cv;
std_msgs::Header header;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "renlianwudiban");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/webcam/image",1);

    CascadeClassifier faces_cascade;
    header.seq = 0;
    header.stamp = ros::Time::now();



    cv::VideoCapture cap(0);
    cv::Mat img;
    cv::Mat img_gray;
    ros::Rate r(1);
    while(ros::ok())
    {
        cap >> img;
        cv::cvtColor(img,img_gray,CV_BGR2GRAY);
        equalizeHist( img_gray , img_gray );
        faces_cascade.load("/home/lin/opencv-3.2.0/data/haarcascades/haarcascade_frontalface_alt.xml");
        vector<Rect> faces;
        faces_cascade.detectMultiScale(img_gray,faces,1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(80, 80) );

        for( int i = 0; i<faces.size(); i++ )
        {
            Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
            ellipse( img, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar  ( 255, 0, 255 ), 4, 8, 0 );

            Mat faceROI = img_gray( faces[i] );
        }
        //cv::GaussianBlur(img_gray, gass1, Size(21,21), 5, 5);
        //cv::GaussianBlur(img_gray, gass2, Size(21,21), 15, 15);
        //cv::absdiff(gass1,gass2,gass1);
        //gass1 *= 3;
        //equalizeHist( gass1 , gass1 );

        cv_bridge::CvImage cvimage(header,sensor_msgs::image_encodings::BGR8,img);

        pub.publish(cvimage.toImageMsg());
        header.seq ++;




    }


    ROS_INFO("Hello world!");
}

