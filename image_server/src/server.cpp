#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <image_server/ImageAction.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
 
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;
bool _isDraw;
int _x, _y;

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    //cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        //cv_gry_ptr = cv_ptr;
        //cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
        
    if(!_isDraw) return;

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    {
        //ROS_INFO("Drawing circles");
        cv::circle(cv_ptr->image, cv::Point(_x, _y), 50, CV_RGB(255,0,0), 3);
    }

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());       
}

void execute(const image_server::ImageGoalConstPtr& goal,
            actionlib::SimpleActionServer<image_server::ImageAction>* as)
{
    ros::Rate r(1);
    
    image_server::ImageFeedback feedback;
 
    ROS_INFO("Begin to draw circle %d times.", goal->total_seconds);

    for(int i=0; i<goal->total_seconds; i++)
    {
        //ImageConverter::_isDraw = true;
        //ImageConverter::_x = 100 + i * 10;
        //ImageConverter::_y = 100 + i * 10;
        _isDraw = true;
        _x = 100 + i * 10;
        _y = 100 + i * 10;
        feedback.process_seconds = i;
        as->publishFeedback(feedback);
        r.sleep();
    }
    
 
    ROS_INFO("All circels are drew.");
 
    as->setSucceeded();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    //ImageConverter ic;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_(nh_);

    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &imageCb);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    _isDraw = false;
    _x = 0; _y = 0;

//==============================================================
    ros::init(argc, argv, "drawing_server");
    
    ros::NodeHandle n;

    //create an action server，accept action named "draw"
    actionlib::SimpleActionServer<image_server::ImageAction> server(n, "draw", boost::bind(&execute, _1, &server), false);
   
    // start server
    server.start();

    ros::spin();
 
    return 0;
}