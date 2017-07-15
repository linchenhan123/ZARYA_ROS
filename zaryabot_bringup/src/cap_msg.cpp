#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace cv;

Mat cap_src;
Mat cap_dst;
Mat cap_msgimg;
Point2f pt_src[4],pt_dst[4];

int main(int argc, char **argv)
{
//ros initialize
    ros::init(argc, argv, "cap_msg");
    ros::NodeHandle nh_;
    image_transport ::ImageTransport it(nh_);
    image_transport ::Publisher pub = it.advertise("webcam/image",1);
    std_msgs::Header header;
    header.seq = 0;
//camera initialize
    VideoCapture cap(0);
    if(!cap.isOpened())
    {
        ROS_INFO("camera can not open!!! ");
    }

    cap >> cap_src;

    pt_src[0].x = 0;
    pt_src[0].y = 0;
    pt_src[1].x = cap_src.cols-1;
    pt_src[1].y = 0;
    pt_src[2].x = 0;
    pt_src[2].y = cap_src.rows-1;
    pt_src[3].x = cap_src.cols-1;
    pt_src[3].y = cap_src.rows-1;

    pt_dst[0].x = 0;
    pt_dst[0].y = 0;
    pt_dst[1].x = cap_src.cols * 0.80;
    pt_dst[1].y = 0;
    pt_dst[2].x = cap_src.cols * 0.08;
    pt_dst[2].y = cap_src.rows * 0.78;
    pt_dst[3].x = cap_src.cols * 0.72;
    pt_dst[3].y = cap_src.rows * 0.78;

//get warpMatrix from points
    Mat warpMat = cv::getPerspectiveTransform(pt_src,pt_dst);

    ros::Rate looprate(30);
    while(ros::ok())
    {
        cap >> cap_src;
        //do transform
        warpPerspective(cap_src,cap_dst,warpMat,Size(cap_src.rows,cap_src.cols));
        //cut ROI
        cap_msgimg = cap_dst(Rect(55,0,390,375));

        /* So the capture image that publishing as sensor message
           has a width 390 and height 375 */



        //convert Mat format to ros_msg

        cv_bridge::CvImage cvtrap_img(header,sensor_msgs::image_encodings::BGR8,cap_msgimg);
        pub.publish(cvtrap_img.toImageMsg());
        ROS_INFO("%d",cap_msgimg.type());
        looprate.sleep();

    }

}

