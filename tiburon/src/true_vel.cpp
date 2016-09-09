#include<iostream>
#include<unistd.h>
#include<math.h>
#include<stdlib.h>
//vectornav source header
#include "vectornav.h"
#include "vn100.h"
#include<ros/ros.h>
//adding message type headers
#include <tiburon/sensor_data.h>
#include<tiburon/ins_data.h>
#include<tiburon/YPR.h>
#include<tiburon/delta_velocity_data.h>
#include<cmath>
#include<geometry_msgs/Vector3.h>
#include<std_msgs/Float32.h>
using namespace std;


 ros::Publisher true_velocity;

 	geometry_msgs::Vector3 calculate_velocity;
 	geometry_msgs::Vector3 acquire_velocity;
 	geometry_msgs::Vector3 ypr_value;
 	float dt;

 void callback_1(const tiburon::ins_data::ConstPtr& ypr)
 {
	 ypr_value.x = ypr->YPR.x;
	 ypr_value.y = ypr->YPR.y;
	 ypr_value.z = ypr->YPR.z;
 }

 void callback(const tiburon::delta_velocity_data::ConstPtr& velocity_value)
 {
	 dt = (float) velocity_value -> delta_Time;
	 acquire_velocity.x = velocity_value->delta_Velocity.x;
	 acquire_velocity.y = velocity_value->delta_Velocity.y;
	 acquire_velocity.z = velocity_value->delta_Velocity.z;

	 calculate_velocity.x = acquire_velocity.x - 9.8*sin((ypr_value.y/180)*3.14) * cos((ypr_value.z/180)*3.14) *dt;
	 calculate_velocity.y = acquire_velocity.y + 9.8*cos((ypr_value.y/180)*3.14) * sin((ypr_value.z/180)*3.14) *dt ;
	 calculate_velocity.z = acquire_velocity.z + 9.8* cos((ypr_value.y/180) * 3.14) * cos((ypr_value.z/180)*3.14) * dt;

	 true_velocity.publish(calculate_velocity);

 }
int main(int argc , char **argv)
{
	ros::init(argc, argv,"true_vel");
	ros::NodeHandle n;
	ros::Subscriber velocity_data = n.subscribe<tiburon::delta_velocity_data>("/tiburon/Delta_Velocity", 1, callback);
	ros::Subscriber ypr_data = n.subscribe<tiburon::ins_data>("/tiburon/ins_data",1,callback_1);
	true_velocity = n.advertise<geometry_msgs::Vector3>("/tiburon/true_velocity",1);

	ros::spin();
	return 0;
}
