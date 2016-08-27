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
int reverseThruster1 = 0, reverseThruster2 = 0, reverseThruster3 = 0, reverseThruster4 = 0;
void reverseCallback(const std_msgs::UInt16::ConstPtr& msg)
{
        switch(msg->data)
        {
            case 1:
                reverseThruster1 = 1 - reverseThruster1;
                break;
            case 2:
                reverseThruster2 = 1 - reverseThruster2;
                break;
            case 3:
                reverseThruster3 = 1 - reverseThruster3;
                break;
            case 4:
                reverseThruster4 = 1 - reverseThruster4;
                break;
            default:
                printf("Incorrect!\n");
        }
}

void frontcallback(const std_msgs::UInt16::ConstPtr& msg)
{
    unsigned short target=msg->data*4;
    if(reverseThruster1)
        target=(3000-msg->data)*4;
    unsigned char command[] = {0x84,0,target & 0x7F,target >> 7 & 0x7F};
    if(write(fd,command,sizeof(command))==-1)
    {
        ROS_INFO("error writing to front thruster");
    }
}
void backcallback(const std_msgs::UInt16::ConstPtr& msg)
{
<<<<<<< HEAD
unsigned short target=msg->data*4;
unsigned char command[] = {0x84,2,target & 0x7F,target >> 7 & 0x7F};
if(write(fd,command,sizeof(command))==-1)
{
ROS_INFO("error writing to back thruster");
}
}
void leftcallback(const std_msgs::UInt16::ConstPtr& msg)
{
unsigned short target=msg->data*4;
unsigned char command[] = {0x84,3,target & 0x7F,target >> 7 & 0x7F};
if(write(fd,command,sizeof(command))==-1)
{
ROS_INFO("error writing to left thruster");
}
}
void rightcallback(const std_msgs::UInt16::ConstPtr& msg)
{
unsigned short target=msg->data*4;
unsigned char command[] = {0x84,4,target & 0x7F,target >> 7 & 0x7F};
if(write(fd,command,sizeof(command))==-1)
{
ROS_INFO("error writing to right thruster");
}
=======
    unsigned short target=msg->data*4;
    if(reverseThruster2)
        target=(3000-msg->data)*4;
    unsigned char command[] = {0x84,2,target & 0x7F,target >> 7 & 0x7F};
    if(write(fd,command,sizeof(command))==-1)
    {
        ROS_INFO("error writing to back thruster");
    }
}
void leftcallback(const std_msgs::UInt16::ConstPtr& msg)
{
    unsigned short target=msg->data*4;
    if(reverseThruster3)
        target=(3000-msg->data)*4;
    unsigned char command[] = {0x84,3,target & 0x7F,target >> 7 & 0x7F};
    if(write(fd,command,sizeof(command))==-1)
    {
        ROS_INFO("error writing to left thruster");
    }
}
void rightcallback(const std_msgs::UInt16::ConstPtr& msg)
{
    unsigned short target=msg->data*4;
    if(reverseThruster4)
        target=(3000-msg->data)*4;
    unsigned char command[] = {0x84,4,target & 0x7F,target >> 7 & 0x7F};
    if(write(fd,command,sizeof(command))==-1)
    {
        ROS_INFO("error writing to right thruster");
    }
>>>>>>> 4dc432545db148f634660807e907124336a75a95
}
int main(int argc, char* argv[])
{
    ros::init(argc,argv,"thruster_controller");
    ros::NodeHandle n;
    ros::Subscriber frontsub = n.subscribe("frontpitchspeed",1,frontcallback);
    ros::Subscriber backsub = n.subscribe("backpitchspeed",1,backcallback);
    ros::Subscriber leftsub = n.subscribe("sideleftspeed",1,leftcallback);
    ros::Subscriber rightsub = n.subscribe("siderightspeed",1,rightcallback);
    ros::Subscriber thrusterReverse = n. subscribe("thrusterreverse",1,reverseCallback);
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
