#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <tiburon/hsv_data.h>

using namespace std;
using namespace cv;

ros::Publisher hsv;
int lowH, lowS, lowV, highH, highS, highV;


void sendData( int, void* )
{
    tiburon::hsv_data msg;
    msg.hl = lowH;
    msg.sl = lowS;
    msg.vl = lowV;
    msg.hh = highH;
    msg.sh = highS;
    msg.vh = highV;
    hsv.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"hsvSegmenter");
    ros::NodeHandle nh;
    hsv = nh.advertise<tiburon::hsv_data>("/hsv_data",1);
    namedWindow("Trackbars",CV_WINDOW_AUTOSIZE);
    createTrackbar("Low H", "Trackbars", &lowH, 255, sendData);
    createTrackbar("Low S", "Trackbars", &lowS, 255, sendData);
    createTrackbar("Low V", "Trackbars", &lowV, 255, sendData);
    createTrackbar("High H", "Trackbars", &highH, 255, sendData);
    createTrackbar("High S", "Trackbars", &highS, 255, sendData);
    createTrackbar("High V", "Trackbars", &highV, 255, sendData);
    waitKey(0);
}
