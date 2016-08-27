#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include <fcntl.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif
int fd;
void frontcallback(const std_msgs::UInt16::ConstPtr& msg)
{
unsigned short target=msg->data*4;
unsigned char command[] = {0x84,2,target & 0x7F,target >> 7 & 0x7F};
if(write(fd,command,sizeof(command))==-1)
{
ROS_INFO("error writing to front thruster");
}
}
void backcallback(const std_msgs::UInt16::ConstPtr& msg)
{
unsigned short target=msg->data*4;
unsigned char command[] = {0x84,3,target & 0x7F,target >> 7 & 0x7F};
if(write(fd,command,sizeof(command))==-1)
{
ROS_INFO("error writing to back thruster");
}
}
void leftcallback(const std_msgs::UInt16::ConstPtr& msg)
{
unsigned short target=msg->data*4;
unsigned char command[] = {0x84,4,target & 0x7F,target >> 7 & 0x7F};
if(write(fd,command,sizeof(command))==-1)
{
ROS_INFO("error writing to left thruster");
}
}
void rightcallback(const std_msgs::UInt16::ConstPtr& msg)
{
unsigned short target=msg->data*4;
unsigned char command[] = {0x84,5,target & 0x7F,target >> 7 & 0x7F};
if(write(fd,command,sizeof(command))==-1)
{
ROS_INFO("error writing to right thruster");
}
}
int main(int argc, char* argv[])
{
ros::init(argc,argv,"thruster_controller");
ros::NodeHandle n;
ros::Subscriber frontsub = n.subscribe("frontpitchspeed",1,frontcallback);
ros::Subscriber backsub = n.subscribe("backpitchspeed",1,backcallback);
ros::Subscriber leftsub = n.subscribe("sideleftspeed",1,leftcallback);
ros::Subscriber rightsub = n.subscribe("siderightspeed",1,rightcallback);
if(argc==1)
{
std::cout<<"Oops there appears to be a problem  with the port"<<std::endl;
return -1;
}
const char *device=(argv[1]);
fd=open(device, O_RDWR | O_NOCTTY);
if(fd==-1)
{
perror(device);
return -2;
}
#ifdef _WIN32
_setmode(fd,_O_BINARY);
#else
struct termios options;
tcgetattr(fd, &options);
options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
options.c_oflag &= ~(ONLCR | OCRNL);
options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
tcsetattr(fd, TCSANOW, &options);
#endif
ros::spin();
close(fd);
return 0;
}
