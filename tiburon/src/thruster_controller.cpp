/*
If this code does not work, please check if the Pololu Servo Controller's Serial
mode is set to "USB Dual Port". This can be done by using the provided UI for
servo control by Pololu.
*/

#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include <fcntl.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <fstream>
#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif

int fd, sFront = 0, sBack = 0, sLeft = 0, sRight = 0;
int reverseThruster1 = 0, reverseThruster2 = 1, reverseThruster3 = 1, reverseThruster4 = 1;

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
	// std::ofstream configFile("/home/auv-nitr/Documents/Tiburon/config.txt");
  //       configFile << reverseThruster1;
  //       configFile << reverseThruster2;
  //       configFile << reverseThruster3;
  //       configFile << reverseThruster4;
  //       configFile.close();
}

void frontcallback(const std_msgs::UInt16::ConstPtr& msg)
{
    if(msg->data==1500) sFront = 1;
    printf("here\n");
    unsigned short target=msg->data*4;
    if(reverseThruster1)
        target=(3000-msg->data)*4;
    unsigned char command[] = {0x84,6,target & 0x7F,target >> 7 & 0x7F};
    if(write(fd,command,sizeof(command))==-1)
    {
        ROS_INFO("error writing to front thruster");
    }
}
void backcallback(const std_msgs::UInt16::ConstPtr& msg)
{
    if(msg->data==1500) sBack = 1;
    unsigned short target=msg->data*4;
    if(reverseThruster2)
        target=(3000-msg->data)*4;
    unsigned char command[] = {0x84,7,target & 0x7F,target >> 7 & 0x7F};
    if(write(fd,command,sizeof(command))==-1)
    {
        ROS_INFO("error writing to back thruster");
    }
}
void leftcallback(const std_msgs::UInt16::ConstPtr& msg)
{
    if(msg->data==1500) sLeft = 1;
    unsigned short target=msg->data*4;
    if(reverseThruster3)
        target=(3000-msg->data)*4;
    unsigned char command[] = {0x84,8,target & 0x7F,target >> 7 & 0x7F};
    if(write(fd,command,sizeof(command))==-1)
    {
        ROS_INFO("error writing to left thruster");
    }
}
void rightcallback(const std_msgs::UInt16::ConstPtr& msg)
{
    if(msg->data==1500) sRight = 1;
    unsigned short target=msg->data*4;
    if(reverseThruster4)
        target=(3000-msg->data)*4;
    unsigned char command[] = {0x84,9,target & 0x7F,target >> 7 & 0x7F};
    if(write(fd,command,sizeof(command))==-1)
    {
        ROS_INFO("error writing to right thruster");
    }
}

// void readFile()
// {
//     std::ifstream configFile("/home/auv-nitr/Documents/Tiburon/config.txt");
//     configFile >> reverseThruster1;
//     configFile >> reverseThruster2;
//     configFile >> reverseThruster3;
//     configFile >> reverseThruster4;
//     configFile.close();
// }

int main(int argc, char* argv[])
{
    // readFile();
    ros::init(argc,argv,"thruster_controller");
    ros::NodeHandle n;
    ros::Subscriber frontsub = n.subscribe("frontpitchspeed",1,frontcallback);
    ros::Subscriber backsub = n.subscribe("backpitchspeed",1,backcallback);
    ros::Subscriber leftsub = n.subscribe("sideleftspeed",1,leftcallback);
    ros::Subscriber rightsub = n.subscribe("siderightspeed",1,rightcallback);
    ros::Subscriber thrusterReverse = n.subscribe("thrusterreverse",1,reverseCallback);
    ros::Publisher statusPub = n.advertise<std_msgs::UInt16>("/controllerStatus",1);
    std_msgs::UInt16 msg;
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
    while(1)
    {
        msg.data = sFront + sBack + sLeft + sRight;
        statusPub.publish(msg);
        ros::spinOnce();
    }
    close(fd);
    return 0;
}
