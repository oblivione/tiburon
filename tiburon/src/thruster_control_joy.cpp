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

int thruster_controller = 0;
int thruster_controller_2 = 0;

class Thruster
{
	public:
		Thruster();

		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		ros::NodeHandle nh_;
		//int linear_,angular_;
		int thruster_button,thruster_init;
		int runmode,f,g,fp,bp,sl,sr;
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
	//thruster_off(1),		
	//thruster_on(2),			
	thruster_button(5),
	thruster_init(4),		//Y button
	runmode(8), 			//A button (numbering acc to jstest)
	f(0),					//toggles runmode (on-off)
	g(0),					//toggles thruster (on-off)
	fp(3),					//Y button for frontpitchspeed
	bp(0),					//A button for backpitchspeed
	sl(2),					//X button for sideleftspeed
	sr(1)					//B button for siderightspeed
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

	if(joy->buttons[thruster_button] == 1)
	{
		g++;
		if(g == 2)        //thruster off
		{
			//cout<<"thruster off\n";
		 	g = 0;
			msg.data = 3;
			thruster_but_pub.publish(msg);
		}
		else if(g == 1)   //thruster on
		{
			//cout<<"thruster on\n";
			msg.data = 2;
			thruster_but_pub.publish(msg);	
		}
	}

	else if(joy->buttons[thruster_init] == 1)
	{
		//cout<<"thruster initialized\n";
		msg.data = 1;
		thruster_but_pub.publish(msg);
		std_msgs::UInt16 msgf; //front
		std_msgs::UInt16 msgb; //back
		std_msgs::UInt16 msgl; //left
		std_msgs::UInt16 msgr; //right
		msgf.data = 1500;  
		msgb.data = 1500; 
		msgl.data = 1500;  
		msgr.data = 1500;  
		frontpitchspeedPub.publish(msgf);
		backpitchspeedPub.publish(msgb);
		siderightspeedPub.publish(msgr);
		sideleftspeedPub.publish(msgl);	
	}

	
	if(joy->buttons[fp] == 1) thruster_controller = 1;
	else if(joy->buttons[bp] == 1) thruster_controller = -1;
	else if(joy->buttons[sr] == 1) thruster_controller = 2;
	else if(joy->buttons[sl] == 1) thruster_controller = -2;
	else thruster_controller = 0;

	if(f == 0)
	{
		if(joy->axes[7] > 0) //up arrow
			thruster_controller_2 = 1;
		else if(joy->axes[7] < 0) //down arrow
			thruster_controller_2 = -1;
		else if(joy->axes[6] > 0) //left arrow
			thruster_controller_2 = -2;
		else if(joy->axes[6] < 0) //right arrow
			thruster_controller_2 = 2;
		else thruster_controller_2 = 0;	
	}

	//if(thruster_controller != 0)
		//cout<<"thruster_controller="<<thruster_controller<<endl;
	//thruster on: data = 2, off: data=3,initialize : data=1
	//twist_pub_.publish(twist);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"thruster_control_joy");
	//cout<<"Joystick node initialized\n";
	Thruster thruster;
	ros::Rate loop_rate(10);

	std_msgs::UInt16 msgf; //front
	std_msgs::UInt16 msgb; //back
	std_msgs::UInt16 msgl; //left
	std_msgs::UInt16 msgr; //right

	/*DEFAULT VALUES:*/
	// update this in joystick ui if changed here
	msgf.data = 1500;  //axes[1] = 1
	msgb.data = 1500;  // axes[1] = -1
	msgl.data = 1500;  //axes[2] = 1
	msgr.data = 1500;  //axes[2] = -1
	
	int prev_msgf = msgf.data;
	int prev_msgb = msgb.data;
	int prev_msgl = msgl.data;
	int prev_msgr = msgr.data;

	while (ros::ok())
	{
		if(thruster_controller == 1) //Y button
		{
			if(thruster.f == 1) //runmode on
			{
				//NOTE: msgf and msgb can't be simply same
				//cout<<"runmode on\n";
				if(prev_msgf != 2000)
				{
					msgf.data = prev_msgf + 2;
					prev_msgf = msgf.data;
				}
				else msgf.data = 2000;
				if(prev_msgb != 2000)
				{
					msgb.data = prev_msgb + 2;
					prev_msgb = msgb.data;
				}
				else msgb.data = 2000;

				//cout<<"msgf="<<msgf.data<<endl;
				//cout<<"msgb="<<msgb.data<<endl;
			}
			else				//runmode off
			{
				//cout<<"runmode off\n";
				if(prev_msgb != 2000)
				{
					msgf.data = prev_msgf + 2;
					prev_msgf = msgf.data;
				}
				else msgf.data = 2000;

				//cout<<"msgf="<<msgf.data<<endl;
			}
		}

		else if(thruster_controller_2 == 1 && thruster.f == 0) //runmode off, up arrow
		{
			//cout<<"runmode off\n";
			if(prev_msgf != 1000)
			{
				msgf.data = prev_msgf - 2;
				prev_msgf = msgf.data;
			}
			else msgf.data = 1000;

			//cout<<"msgf="<<msgf.data<<endl;
		}

		else if(thruster_controller == -1 ) //A button
		{
			if(thruster.f == 1) //runmode on
			{
				//cout<<"runmode on\n";
				if(prev_msgf != 1000)
				{
					msgf.data = prev_msgf - 2;
					prev_msgf = msgf.data;
				}
				else msgb.data = 1000;
				if(prev_msgb != 1000)
				{
					msgb.data = prev_msgb - 2;
					prev_msgb = msgb.data;
				}
				else msgb.data = 1000;

				//cout<<"msgf="<<msgf.data<<endl;
				//cout<<"msgb="<<msgb.data<<endl;
			}
			else				//runmode off
			{
				//cout<<"runmode off\n";
				if(prev_msgb != 2000)
				{
					msgb.data = prev_msgb + 2;
					prev_msgb = msgb.data;
				}
				else msgb.data = 2000;
				//cout<<"msgb="<<msgb.data<<endl;
			}
		}

		else if(thruster_controller_2 == -1 && thruster.f == 0) //runmode off, down arrow
		{
			//cout<<"runmode off\n";
			if(prev_msgb != 1000)
			{
				msgb.data = prev_msgb - 2;
				prev_msgb = msgb.data;
			}
			else msgb.data = 1000;

			//cout<<"msgb="<<msgb.data<<endl;
		}

		else if(thruster_controller == 2)
		{
			if(thruster.f == 1) //runmode on
			{
				//cout<<"runmode on\n";
				if(prev_msgr != 2000)
				{
					msgr.data = prev_msgr + 2;
					prev_msgr = msgr.data;
				}
				else msgr.data = 2000;
				if(prev_msgl != 2000)
				{
					msgl.data = prev_msgl + 2;
					prev_msgl = msgl.data;
				}
				else msgl.data = 2000;

				//cout<<"msgr="<<msgr.data<<endl;
				//cout<<"msgl="<<msgl.data<<endl;
			}
			else				//runmode off
			{
				//cout<<"runmode off\n";
				if(prev_msgr != 2000)
				{
					msgr.data = prev_msgr + 2;
					prev_msgr = msgr.data;
				}
				else msgr.data = 2000;
				
				//cout<<"msgr="<<msgr.data<<endl;
			}
		}

		else if(thruster_controller_2 == 2 && thruster.f == 0) //runmode off, down arrow
		{
			//cout<<"runmode off\n";
			if(prev_msgr != 1000)
			{
				msgr.data = prev_msgr - 2;
				prev_msgr = msgr.data;
			}
			else msgr.data = 1000;

			//cout<<"msgr="<<msgr.data<<endl;
		}

		else if(thruster_controller == -2)
		{
			if(thruster.f == 1) //runmode on
			{
				//cout<<"runmode on\n";
				if(prev_msgr != 1000)
				{
					msgr.data = prev_msgr - 2;
					prev_msgr = msgr.data;
				}
				else msgl.data = 1000;
				if(prev_msgl != 1000)
				{
					msgl.data = prev_msgl - 2;
					prev_msgl = msgl.data;
				}
				else msgl.data = 1000;
				
				//cout<<"msgr="<<msgr.data<<endl;
				//cout<<"msgl="<<msgl.data<<endl;
			}
			else				//runmode off
			{
				//cout<<"runmode off\n";
				if(prev_msgl != 2000)
				{
					msgl.data = prev_msgl + 2;
					prev_msgl = msgl.data;
				}
				else msgl.data = 2000;
				//cout<<"msgl="<<msgl.data<<endl;
			}
		}
		
		else if(thruster_controller_2 == -2 && thruster.f == 0) //runmode off, down arrow
		{
			//cout<<"runmode off\n";
			if(prev_msgl != 1000)
			{
				msgl.data = prev_msgl - 2;
				prev_msgl = msgl.data;
			}
			else msgl.data = 1000;

			//cout<<"msgl="<<msgl.data<<endl;
		}

    	ros::spinOnce();
    	loop_rate.sleep();
		
		thruster.frontpitchspeedPub.publish(msgf);
		thruster.backpitchspeedPub.publish(msgb);
		thruster.siderightspeedPub.publish(msgr);
		thruster.sideleftspeedPub.publish(msgl);	
	}
}
