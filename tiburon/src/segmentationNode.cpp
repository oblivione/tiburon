#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tiburon/hsv_data.h>

using namespace cv;
using namespace std;

bool imageReceived = false;
Mat inp, out;
int hl=71,sl=38,vl=87,hh=138,sh=255,vh=255;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
   try
   {
     inp = (cv_bridge::toCvShare(msg, "bgr8")->image).clone(); // Don't remove this clone call, else it will not finish copying the whole image and overwrite it prematurely
     //imshow("Img",cv_bridge::toCvShare(msg, "bgr8")->image );
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
   imageReceived=true; 
   
}

void hsvCallback(const tiburon::hsv_data::ConstPtr& hsv)
{
    hl = hsv->hl;
    sl = hsv->sl;
    vl = hsv->vl;
    hh = hsv->hh;
    sh = hsv->sh;
    vh = hsv->vh;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "SegmentationNode");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber imageSub = it.subscribe("auv_cam1",1,imageCallback);
  image_transport::Publisher segPub = it.advertise("bottomCameraSegmented", 1);
  ros::Subscriber hsvData = nh.subscribe("/tiburon/hsv_data",1,hsvCallback);
  sensor_msgs::ImagePtr msg;
  while(1)
  {
    if(imageReceived)
    { 
      //imshow("Inp",inp);
      //waitKey(2);
      cvtColor(inp,inp,CV_BGR2HSV);
     // cout << inp << endl;
      inRange(inp,Scalar(hl,sl,vl),Scalar(hh,sh,vh),out);
      //cout<<"here"<<out.channels()<<endl;
      msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", out).toImageMsg();
      segPub.publish(msg);
      imageReceived = false;
    }
    if((char)waitKey(1)==27)
    {
      break;
    }
    ros::spinOnce();
  }
}
