#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/algorithm/string.hpp>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

static const int REDUCE_RATIO = 50;    // 1/Ratio speed to publish value
int counter;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  const int REAL_D = 130; //mm, real diameter of the calibration circle

  public:
    double offset_x, offset_y;  // unit: mm
    double ratio;               // pixels/mm
    ImageConverter()
      : it_(nh_), offset_x(0), offset_y(0), ratio(0)
    {
      image_sub_ = it_.subscribe("/camera/color/image_raw", 1,
        &ImageConverter::imageCb, this);
      image_pub_ = it_.advertise("/image_converter/output_video", 1);

      cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      // Draw an example circle on the video stream
      if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      {
        //ROS_INFO("Detecting circles");

        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
  
        // Threshold the HSV image, keep only the red pixels
        cv::Mat lower_red_hue_range;
        cv::Mat upper_red_hue_range;
        cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
        cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

        cv::Mat red_hue_image;
        cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
        cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);
        
        DrawCircleAndLine(red_hue_image, cv_ptr->image);

        //cv_ptr->image = red_hue_image;
        
      }

      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(3);

      // Output modified video stream
      //image_pub_.publish(cv_ptr->toImageMsg());
    }

    void DrawCircleAndLine(const cv::Mat &inputImage, cv::Mat &outputImage)
    {
      vector<Vec3f> circles;
      HoughCircles(inputImage, circles, HOUGH_GRADIENT, 1,
                  inputImage.rows/16,  // change this value to detect circles with different distances to each other
                  100, 30, 30, 300 // change the last two parameters
              // (min_radius & max_radius) to detect larger circles
      );
      ratio = 0;
      offset_x = 0;
      offset_y = 0;
      if (circles.size() > 0)
      {
          Vec3i c = circles[0];
          Point circleCenter = Point(c[0], c[1]);
          // circle center
          circle( outputImage, circleCenter, 1, Scalar(0,0,255), 3, LINE_AA);
          // circle outline
          int radius = c[2];
          circle( outputImage, circleCenter, radius, Scalar(0,255,0), 3, LINE_AA);
          // Draw a line to the center of the image
          Point imageFrameCenter = Point(inputImage.size().width / 2, inputImage.size().height / 2); 
          line(outputImage, imageFrameCenter, circleCenter, Scalar(255,0,0),3,LINE_AA );
          
          // calculate output
          ratio = (double)(2 * radius) / (double)REAL_D;

          if(ratio != 0)
          {
            offset_x = (double)(circleCenter.x - imageFrameCenter.x) / ratio;
            offset_y = (double)(circleCenter.y - imageFrameCenter.y) / ratio;

            std::string s1;
            s1 = to_string(offset_x) + "," + to_string(offset_y);

            Point textCenter = Point(c[0], c[1] - 10);
            putText(outputImage, s1, textCenter, FONT_HERSHEY_DUPLEX, 0.8, CV_RGB(255,255,255), 1.0);
          }
          else
          {
            std::string ss;
            ss == "Ratio is 0";
            Point textCenter = Point(c[0], c[1] - 10);
            putText(outputImage, ss, textCenter, FONT_HERSHEY_DUPLEX, 0.8, CV_RGB(255,0,0), 1.0);
          }
      }
      /*
      for( size_t i = 0; i < circles.size(); i++ )
      {
          Vec3i c = circles[i];
          Point center = Point(c[0], c[1]);
          // circle center
          circle( cv_ptr->image, center, 1, Scalar(0,0,255), 3, LINE_AA);
          // circle outline
          int radius = c[2];
          circle( cv_ptr->image, center, radius, Scalar(0,0,255), 3, LINE_AA);
      }
      */
    }
  };



int main(int argc, char** argv)
{
  counter = 0;

  ros::init(argc, argv, "nOpencv_server");
  ImageConverter ic;

  //========== publisher ==============
  ros::init(argc, argv, "nOffset_publisher");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("tCenter_offset", 1000);
  ros::Rate loop_rate(10);

  ROS_INFO("nOffset_publisher is running...publishing detected circle center offset to [tCenter_offset]");

  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << ic.offset_x << "," << ic.offset_y;
    msg.data = ss.str();

    //ROS_INFO("Center offset: %s (mm)", msg.data.c_str());

    if(counter > REDUCE_RATIO)
    {
      counter = 0;
      chatter_pub.publish(msg);
    }

    ros::spinOnce();

    loop_rate.sleep();
    counter++;
  }

  //ros::spin();
  return 0;
}