// Include the ROS library
#include <ros/ros.h>

// Include pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <iostream>

// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>

// Topics
static const std::string IMAGE_TOPIC = "/camera/depth/color/points";
static const std::string PUBLISH_TOPIC = "/pcl/point_cloud";

float minX = -0.1, minY = -0.5, minZ = -2.5;
float maxX = +0.1, maxY = +0.5, maxZ = +2.5;

// ROS Publisher
ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PCLPointCloud2* p_cloud = new pcl::PCLPointCloud2();
    pcl::PCLPointCloud2* p_cloud_filtered = new pcl::PCLPointCloud2();

    pcl::PCLPointCloud2ConstPtr cp_cloud(p_cloud);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *p_cloud);

    //pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    
    #pragma region ====================== Cut extra points ==================================
    pcl::PassThrough<pcl::PCLPointCloud2> filter;
    filter.setInputCloud (cp_cloud);
    filter.setFilterFieldName ("x");
    filter.setFilterLimits (minX, maxX);
    filter.setFilterFieldName ("y");
    filter.setFilterLimits (minY, maxY);
    filter.setFilterFieldName ("z");
    filter.setFilterLimits (minZ, maxZ);
    filter.filter(*p_cloud_filtered);

    #pragma endregion ====================== Cut extra points ==================================
    //viewer.addPointCloud(*p_cloud_filtered,"body");
    //viewer.spin();

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(*p_cloud_filtered, output);

    // Publish the data
    pub.publish (output);
}

int main (int argc, char** argv)
{
    // Initialize the ROS Node "roscpp_pcl_example"
    ros::init (argc, argv, "pcl_process");
    ros::NodeHandle nh;

    // Print "Hello" message with node name to the terminal and ROS log file
    ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

    // Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
    ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 1, cloud_cb);

    // Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
    pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);

    // Spin
    ros::spin();

    // Success
    return 0;
}