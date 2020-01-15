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
  const int CAM_FOCAL = 1.88;    // mm
  const int CAM_PIXEL_SIZE = 1.4;    // um

  public:
    double offset_x, offset_y;  // unit: mm
    double d1;                  // object distance
    double roll, pitch;
    double ratio;               // pixels/mm
    ImageConverter()
      : it_(nh_), offset_x(0), offset_y(0), ratio(0), d1(0)
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
        
        DrawEllipseAndBox(red_hue_image, cv_ptr->image);
      }

      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(3);

      // Output modified video stream
      //image_pub_.publish(cv_ptr->toImageMsg());
    }

    void DrawEllipseAndBox(const cv::Mat &inputImage, cv::Mat &outputImage)
    {
      int thresh = 250;
      const int CSZ_THR = 144;
      Mat canny_output;
      Canny( inputImage, canny_output, thresh, thresh*2 );
      vector<vector<Point> > contours;
      findContours( canny_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
      vector<RotatedRect> minRect( contours.size() );
      vector<RotatedRect> minEllipse( contours.size() );

      if(minEllipse.size() > 0)
      {
        for( size_t i = 0; i < contours.size(); i++ )
        {
            if( contours[i].size() > CSZ_THR )
            {
                minRect[i] = minAreaRect( contours[i] );
                minEllipse[i] = fitEllipse( contours[i] );
            }
        }
        //Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
        for( size_t i = 0; i< contours.size(); i++ )
        {
            if(minEllipse[i].center.x == 0 && minEllipse[i].center.y == 0) continue;

            Scalar color = Scalar(255, 255, 0);
            // contour
            drawContours( outputImage, contours, (int)i, color );
            // ellipse
            ellipse( outputImage, minEllipse[i], color, 2 );
            // rotated rectangle
            Point2f rect_points[4];
            minRect[i].points( rect_points );
            for ( int j = 0; j < 4; j++ )
            {
                line( outputImage, rect_points[j], rect_points[(j+1)%4], color );
            }

            // Draw off center line
            Point circleCenter = minEllipse[i].center;
            Point imageFrameCenter = Point(inputImage.size().width / 2, inputImage.size().height / 2); 
            line(outputImage, imageFrameCenter, circleCenter, Scalar(255,0,0),1,LINE_AA );

            // Calculate output
            double radius = (minEllipse[i].size.width > minEllipse[i].size.height ) ? minEllipse[i].size.width : minEllipse[i].size.height;
            ratio = (double)(2 * radius) / (double)REAL_D;
            d1 = (double)(REAL_D * CAM_FOCAL) /(double)(2 * radius * CAM_PIXEL_SIZE * 0.001);   // convert to mm

            if(ratio != 0)
            {
              offset_x = (double)(circleCenter.x - imageFrameCenter.x) / ratio;
              offset_y = (double)(circleCenter.y - imageFrameCenter.y) / ratio;

              std::string s1;
              s1 = to_string(offset_x) + "," + to_string(offset_y);

              Point textCenter = Point(circleCenter.x, circleCenter.y - 10);
              putText(outputImage, s1, textCenter, FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(255,255,255), 1.0);
            }
            else
            {
              std::string ss;
              ss == "Ratio is 0";
              Point textCenter = Point(circleCenter.x, circleCenter.y - 10);
              putText(outputImage, ss, textCenter, FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(255,0,0), 1.0);
            }

            // Angles

            break;
        }

      }
      else
      { 
        offset_x = 0; offset_y = 0; d1 = 0;
      }

      //outputImage = outputImage;
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
    std::string ss;
    std::string s2;

    std_msgs::String msg;

    ss = to_string(ic.offset_x) + ',' + to_string(ic.offset_y) + ',' + to_string(ic.d1);
    msg.data = ss;

    //ROS_INFO("Center offset: %s (mm)", msg.data.c_str());

    if(counter > REDUCE_RATIO)
    {
      counter = 0;
      s2 = to_string(ic.d1);
      ROS_INFO("Distance %s (mm)",s2.c_str());
      chatter_pub.publish(msg);
    }

    ros::spinOnce();

    loop_rate.sleep();
    counter++;
  }

  //ros::spin();
  return 0;
}