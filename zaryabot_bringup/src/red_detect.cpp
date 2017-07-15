#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
using namespace cv;
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

int main(int argc, char **argv)
{   //initializing work
    ros::init(argc, argv, "red_detect");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/webcam/image",1);
    std_msgs::Header header;
    header.seq = 0;
    header.stamp = ros::Time::now();



    //videocapture
    VideoCapture cap(0);
    Mat cap_src;
    Mat cap_ROI;
    Mat cap_hsv;
    Mat hsv_thr1;
    Mat hsv_thr2;
    Mat red = Mat::zeros(cap_src.rows,cap_src.cols,CV_8U);
    std::vector<Mat> hsv_spl;
    int number;

    //loop
    ros::Rate r(1);
    while(ros::ok())
    {
        cap >> cap_src;

        cvtColor(cap_src,cap_hsv,CV_BGR2HSV);

        split(cap_hsv,hsv_spl);
        equalizeHist(hsv_spl[2],hsv_spl[2]);
        merge(hsv_spl,cap_hsv);
        //thresholding
        inRange(cap_hsv,Scalar(165,100,120),Scalar(180,255,255),hsv_thr1);
        inRange(cap_hsv,Scalar(0,100,120),Scalar(10,255,255),hsv_thr2);

        //bonding two red picture thresholeded
        red = hsv_thr1 + hsv_thr2;


        cap_ROI = red(Rect(0,cap_src.rows/2 + 25,cap_src.cols,50));
   

        //show the ROI region in src picture
        cv::rectangle(cap_src,Rect(0,cap_src.rows/2 + 25,cap_src.cols,50),Scalar(255,255,0));

        number = bSums(cap_ROI);

        //output the number we get

        ROS_INFO("the red iterator in range is %d",number);
        cv_bridge::CvImage cvimg(header,sensor_msgs::image_encodings::BGR8,cap_src);
        pub.publish(cvimg.toImageMsg());








    }



}
