#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Joy.h> //Run this: rosrun joy joy_node
#include <iostream>
using namespace std;

/*
TO TEST:
sudo apt-get install ros-indigo-joy
cd /dev/input to find the name of attached joystick node --> js0 or js1
sudo jstest /dev/input/js1 (or js0 if that is the device)-->array of axes,buttons

FOR PROPER WORKING OF DEVICE:
sudo chmod a+rw /dev/input/jsX (js0 or js1 etc)

TO START THE JOY NODE:
rosparam set joy_node/dev "/dev/input/jsX"  --> (jsX=js0 or js1 etc)
rosrun joy joy_node
rostopic echo joy //for values of axes,buttons array

http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick for details
http://wiki.ros.org/joy/Tutorials/WritingTeleopNode for beginners
*/


class Thruster
{
	public:
		Thruster();
	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

		ros::NodeHandle nh_;

		//int linear_,angular_;
		int thruster_on,thruster_off,thruster_init;
		int runmode,f;
		//double l_scale_, a_scale_;
		ros::Publisher thruster_but_pub;
		ros::Publisher frontpitchspeedPub;
		ros::Publisher backpitchspeedPub;
		ros::Publisher sideleftspeedPub;
		ros::Publisher siderightspeedPub;
		ros::Publisher runmodePub;
		ros::Subscriber joy_sub_;
};

Thruster::Thruster():
	//linear_(1),
	//angular_(2),
	thruster_off(1),		//B button
	thruster_on(2),			//X button
	thruster_init(3),		//Y button
	runmode(0), 			//A button (numbering acc to jstest)
	f(0)
{
	//nh_.param("axis_linear",linear_,linear_);
	//nh_.param("axis_angular",angular_,angular_);
	//nh_.param("scale_angular",a_scale_,a_scale_);
	//nh_.param("scale_linear",l_scale_,l_scale_);

	//twist_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
	thruster_but_pub = nh_.advertise<std_msgs::UInt16>("thrusterstate",1 ); //on,off,init
	frontpitchspeedPub = nh_.advertise<std_msgs::UInt16>("frontpitchspeed",1 );
	backpitchspeedPub = nh_.advertise<std_msgs::UInt16>("backpitchspeed",1 );
	sideleftspeedPub = nh_.advertise<std_msgs::UInt16>("sideleftspeed",1 );
	siderightspeedPub = nh_.advertise<std_msgs::UInt16>("siderightspeed",1);
	runmodePub = nh_.advertise<std_msgs::UInt16>("runmode",1);


	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy",10,&Thruster::joyCallback,this);
}

void Thruster::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	//geometry_msgs::Twist twist;
	std_msgs::UInt16 msg;
	//twist.angular.z = a_scale_*joy->axes[angular_];
	//twist.linear.x = l_scale_*joy->axes[linear_];

	std_msgs::UInt16 runmode_data;
	//tiburon::joystick_data joy_msg;

	if(joy->buttons[runmode] == 1)
	{
		f++;
		if(f == 2)
		{
		 	f = 0;
		}
		runmode_data.data = f;
		runmodePub.publish(runmode_data);
	}
	if(joy->buttons[thruster_off] == 1)
	{
		msg.data = 3;
		thruster_but_pub.publish(msg);
	}
	else if(joy->buttons[thruster_on] == 1)
	{
		msg.data = 2;
		thruster_but_pub.publish(msg);
	}
	else if(joy->buttons[thruster_init] == 1)
	{
		msg.data = 1;
		thruster_but_pub.publish(msg);
	}

	std_msgs::UInt16 msgf; //front
	std_msgs::UInt16 msgb; //back
	std_msgs::UInt16 msgl; //left
	std_msgs::UInt16 msgr; //right

	/*DEFAULT VALUES:*/
	// update this in joystick ui if changed here
	msgf.data = 1000;  //axes[1] = 1
	msgb.data = 1000;  // axes[1] = -1
	msgl.data = 1500; //axes[2] = 1
	msgr.data = 1500; //axes[2] = -1

	//FOLLOWING IS SUBJECT TO CHANGE DEPENDING ON THE JOYSTICK CONFIGURATION --> the desired index and given range of the axes
	switch(f)
	{
		//RUNMODE: ON --> (FRONT,BACK) AND (LEFT,RIGHT) CHANGE SIMULTANEOUSLY
		case 1:

		if( joy->axes[4] <= 1 && joy->axes[4] >=0)
		{
			msgf.data +=  (int) (joy->axes[4] * 1000);  //map it
			msgb.data = msgf.data;
		}
		else if( joy->axes[4] >= -1 && joy->axes[4] <=0 )
		{
			msgb.data -=  (int) (joy->axes[4] * 1000); //map it
			msgf.data = msgb.data;
		}
		if( joy->axes[2] <=1 && joy->axes[2] >=0)
		{
			msgl.data += (int)(joy->axes[2] * 500); //map it
			msgr.data = msgl.data;
		}
		if( joy->axes[2] >=-1 && joy->axes[2] <=0)
		{
			msgr.data += (int)(joy->axes[2] * 500);//map it
			msgl.data = msgr.data;
		}
		break;

		case 0:

		if( joy->axes[4] <= 1 && joy->axes[4] >=0)
		{
			msgf.data +=  (int) (joy->axes[4] * 1000);  //map it
			//msgb.data = msgf.data;
		}
		else if( joy->axes[4] >= -1 && joy->axes[4] <=0 )
		{
			msgb.data -=  (int) (joy->axes[4] * 1000); //map it
			//msgf.data = msgb.data;
		}
		if( joy->axes[2] <=1 && joy->axes[2] >=0)
		{
			msgl.data += (int)(joy->axes[2] * 500); //map it
			//msgr.data = msgl.data;
		}
		if( joy->axes[2] >=-1 && joy->axes[2] <=0)
		{
			msgr.data += (int)(joy->axes[2] * 500);//map it
			//msgl.data = msgr.data;
		}
		break;

		default:
		cout<<"wrong choice"<<endl;
	}

	frontpitchspeedPub.publish(msgf);
	backpitchspeedPub.publish(msgb);
	siderightspeedPub.publish(msgr);
	sideleftspeedPub.publish(msgl);

	//thruster on: data = 2, off: data=3,initialize : data=1
	//twist_pub_.publish(twist);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"thruster_control_joy");
	cout<<"Joystick node initialized\n";
	Thruster thruster;

	ros::spin();
}
