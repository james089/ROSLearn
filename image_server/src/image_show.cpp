#include <iostream>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include "std_msgs/String.h"
using namespace std;

int main(int argc, char* argv[])  {

    int x;
    char buf[100];
    cin >> x;
    cin.getline(buf, 90);
    cout << buf << endl;

    return 0; 
}