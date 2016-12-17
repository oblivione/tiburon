#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <tiburon/hsv_data.h>

using namespace std;
using namespace cv;

ros::Publisher hsv2,hsv1;
int lowH=25, lowS=30, lowV=30, highH=155, highS=255, highV=255,d=70;
int lowH1=14, lowS1=30, lowV1=30, highH1=155, highS1=255, highV1=255,d1=70;


void sendData( int, void* )
{
    tiburon::hsv_data msg;
    msg.d = (d*1.0)/100.0;
    msg.hl = lowH;
    msg.sl = lowS;
    msg.vl = lowV;
    msg.hh = highH;
    msg.sh = highS;
    msg.vh = highV;
    hsv1.publish(msg);
}
void sendData1( int, void* )
{
    tiburon::hsv_data msg;
    msg.d = (d1*1.0)/100.0;
    msg.hl = lowH1;
    msg.sl = lowS1;
    msg.vl = lowV1;
    msg.hh = highH1;
    msg.sh = highS1;
    msg.vh = highV1;
    hsv2.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"hsvSegmenter");
    ros::NodeHandle nh;
    hsv1 = nh.advertise<tiburon::hsv_data>("/hsv_data_1",1);
    hsv2 = nh.advertise<tiburon::hsv_data>("/hsv_data_2",1);
    namedWindow("Trackbars",CV_WINDOW_AUTOSIZE);
    createTrackbar("D", "Trackbars", &d, 100, sendData);
    createTrackbar("Low H", "Trackbars", &lowH, 255, sendData);
    createTrackbar("Low S", "Trackbars", &lowS, 255, sendData);
    createTrackbar("Low V", "Trackbars", &lowV, 255, sendData);
    createTrackbar("High H", "Trackbars", &highH, 255, sendData);
    createTrackbar("High S", "Trackbars", &highS, 255, sendData);
    createTrackbar("High V", "Trackbars", &highV, 255, sendData);
    namedWindow("Trackbars1",CV_WINDOW_AUTOSIZE);
    createTrackbar("D", "Trackbars1", &d1, 100, sendData1);
    createTrackbar("Low H", "Trackbars1", &lowH1, 255, sendData1);
    createTrackbar("Low S", "Trackbars1", &lowS1, 255, sendData1);
    createTrackbar("Low V", "Trackbars1", &lowV1, 255, sendData1);
    createTrackbar("High H", "Trackbars1", &highH1, 255, sendData1);
    createTrackbar("High S", "Trackbars1", &highS1, 255, sendData1);
    createTrackbar("High V", "Trackbars1", &highV1, 255, sendData1);
    while(1)
        waitKey(1);
}
