#include <iostream>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include "std_msgs/String.h"

void chatterCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]", msg->height);
}

int main(int argc, char* argv[])  {

    ros::init(argc, argv, "image_viewer");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/camera/color/image_raw", 1, chatterCallback);
    //cv::Mat img = cv::imread("/home/mostafa/Downloads/cat.jpg"); //Make sure that the image is there

    //std::cout << img.rows << "; " << img.cols <<  std::endl;

    //cv::namedWindow("img");
    //cv::imshow("img", img);

    ros::spin();
    /*
    ros::Rate r(1);
    while(ros::ok)
    {
        ros::spinOnce();
        r.sleep();
    }   
    */
    return 0; 
}