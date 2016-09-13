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

//Publishers
	ros::Publisher pubsens_data;
	ros::Publisher pubins_data;
	ros::Publisher pubvelocity_data;
	
//Device
	Vn100 vn100;
	std::string imu_frame_id;
	void vnerror_msg(VN_ERROR_CODE vn_error,std::string &msg);
//printing the asynchronous data;
void asyncDataListener(Vn100* vn100,Vn100CompositeData* data)
{
	ros::Time timestamp=ros::Time::now();
	static int seq =0;
	ROS_INFO("\nASYNC DATA:\n"
	         "YPR.Yaw: %+#7.2f\n"
	         "YPR.pitch: %+#7.2f\n"
	         "YPR.roll: %+#7.2f\n",
	         data->ypr.yaw,
	         data->ypr.pitch, 
	         data->ypr.roll);
	ROS_INFO(
	     "\n quaternion.X:  %+#7.2f\n"
	     "   quaternion.Y:  %+#7.2f\n"
	     "   quaternion.Z:  %+#7.2f\n"
	     "   quaternion.W:  %+#7.2f\n", 
	        data->quaternion.x,
	        data->quaternion.y,
	        data->quaternion.z,
	        data->quaternion.w);
	ROS_INFO(
	       "\n            {Value,Voltage}\n"
	       " magnetic X:  %+#7.2f, %+#7.2f\n"
	       " magnetic Y:  %+#7.2f, %+#7.2f\n"
	       " magnetic Z:  %+#7.2f, %+#7.2f\n",
	         data->magnetic.c0,data->magneticVoltage.c0, 
	         data->magnetic.c1,data->magneticVoltage.c1,
	         data->magnetic.c2,data->magneticVoltage.c2);
	ROS_INFO(
	       "\n acceleration X:    %+#7.2f,%+#7.2f\n"
	       "  acceleration Y:     %+#7.2f,%+#7.2f\n"
	       "  acceleration Z:     %+#7.2f,%+#7.2f\n",
	          data->acceleration.c0,
	          data->accelerationVoltage.c0,
	          data->acceleration.c1,
	          data->accelerationVoltage.c1,
	          data->acceleration.c2,
	          data->accelerationVoltage.c2);
	ROS_INFO(
	       "\n                   {Value ,Voltage,Bias,BiasVariance}"
       	"  angularRate X:          %+#7.2f, %+#7.2f, %+#7.2f, %+#7.2f\n"
	  "  angularRate Y:          %+#7.2f, %+#7.2f, %+#7.2f, %+#7.2f\n"
	  "  angularRate Z:          %+#7.2f, %+#7.2f, %+#7.2f, %+#7.2f\n",
		  data->angularRate.c0,
	          data->angularRateVoltage.c0, 
		  data->angularRateBias.c0,
	          data->angularRateBiasVariance.c0, 
		  data->angularRate.c1,    
	          data->angularRateVoltage.c1, 
		  data->angularRateBias.c1, 
	          data->angularRateBiasVariance.c1, 
		  data->angularRate.c2,    
	          data->angularRateVoltage.c2,
		  data->angularRateBias.c2, data->angularRateBiasVariance.c2);
	 ROS_INFO(
		  "\n  Attitude Variance X:    %+#7.2f\n"
		  "  Attitude Variance Y:    %+#7.2f\n"
		  "  Attitude Variance Z:    %+#7.2f\n",
		  data->attitudeVariance.c0, 
		  data->attitudeVariance.c1, 
		  data->attitudeVariance.c2);
	
	 ROS_INFO(
		  "\n  Direction Cosine Matrix:\n"
		  "    %+#7.2f, %+#7.2f, %+#7.2f\n"
		  "    %+#7.2f, %+#7.2f, %+#7.2f\n"
		  "    %+#7.2f, %+#7.2f, %+#7.2f\n",
		  data->dcm.c00, data->dcm.c01, data->dcm.c02,
		  data->dcm.c10, data->dcm.c11, data->dcm.c12,
		  data->dcm.c20, data->dcm.c21, data->dcm.c22);
	
	    ROS_INFO(
		  "\n  Temperature:            %+#7.2f\n"
		  "  Temperature Voltage:    %+#7.2f\n",
		  data->temperature,
		  data->temperatureVoltage);
	
}

void publish_device()
{
	static int seq=0;
	seq++;
	VN_ERROR_CODE vn_error;
	std::string vn_err_msg;
	ros::Time timestamp=ros::Time::now();
	if(pubsens_data.getNumSubscribers()>0)
	{
		ROS_INFO("subscribed");
		VnVector3 Gyro,Mag,Accel;
		float Pressure,Temp;
		vn_error=vn100_getCalibratedSensorMeasurements(&vn100,&Mag,&Accel,&Gyro,&Temp,&Pressure);
		if(vn_error!=VNERR_NO_ERROR)
		{
			vnerror_msg(vn_error,vn_err_msg);
			ROS_INFO("error in reading the sensor data:%s",vn_err_msg.c_str());
		}
		else
		{
			tiburon::sensor_data msg_sensor_data;
			msg_sensor_data.header.seq=seq;
			msg_sensor_data.header.stamp=timestamp;
			msg_sensor_data.header.frame_id=imu_frame_id;
			msg_sensor_data.Mag.x=Mag.c0;
			msg_sensor_data.Mag.y=Mag.c1;
			msg_sensor_data.Mag.z=Mag.c2;
			msg_sensor_data.Accel.x=Accel.c0;
			msg_sensor_data.Accel.y=Accel.c1;
			msg_sensor_data.Accel.z=Accel.c2;
			msg_sensor_data.Gyro.x=Gyro.c0;
			msg_sensor_data.Gyro.y=Gyro.c1;
			msg_sensor_data.Gyro.z=Gyro.c2;
			msg_sensor_data.Temp=Temp;
			msg_sensor_data.Pressure=Pressure;
			pubsens_data.publish(msg_sensor_data);
		}
	}
	if(pubins_data.getNumSubscribers()>0)
	{
		VnYpr YPR;
		VnQuaternion quat_data;
		tiburon::ins_data msg_ins_data;
		vn_error=vn100_getYawPitchRoll(&vn100,&YPR);
		if(vn_error!=VNERR_NO_ERROR)
		{
			vnerror_msg(vn_error,vn_err_msg);
			ROS_INFO("error in reading the ins data:%s",vn_err_msg.c_str());
		}
		else
		{
			vn_error=vn100_getQuaternion(&vn100,&quat_data);
			if(vn_error!=VNERR_NO_ERROR)
			{
				vnerror_msg(vn_error,vn_err_msg);
				ROS_INFO("error in reading the ins data:%s",vn_err_msg.c_str());
			}
			else
			{
				msg_ins_data.header.seq=seq;
				msg_ins_data.header.stamp=timestamp;
				msg_ins_data.header.frame_id=imu_frame_id;
				msg_ins_data.YPR.x=YPR.yaw;
				msg_ins_data.YPR.y=YPR.pitch;
				msg_ins_data.YPR.z=YPR.roll;
				msg_ins_data.quat_data[0]=quat_data.x;
				msg_ins_data.quat_data[1]=quat_data.y;
				msg_ins_data.quat_data[2]=quat_data.z;
				msg_ins_data.quat_data[3]=quat_data.w;
				pubins_data.publish(msg_ins_data);
			}
		}
	}
	if(pubvelocity_data.getNumSubscribers()>0)
	{
		VnVector3 delta_Velocity,delta_Theta;
		float delta_Time;
		vn_error = vn100_getdeltaVelocity(&vn100,&delta_Time,&delta_Theta,&delta_Velocity);
		if(vn_error!=VNERR_NO_ERROR)
		{
			vnerror_msg(vn_error,vn_err_msg);
			ROS_INFO("error in reading the velocity data:%s",vn_err_msg.c_str());
		}
		else
		{
			tiburon::delta_velocity_data velocity_data;
			velocity_data.header.seq = seq;
			velocity_data.header.stamp=timestamp;
			velocity_data.header.frame_id=imu_frame_id;
			velocity_data.delta_Time=delta_Time;
			velocity_data.delta_Theta.x=delta_Theta.c0;
			velocity_data.delta_Theta.y=delta_Theta.c1;
			velocity_data.delta_Theta.y=delta_Theta.c1;
			velocity_data.delta_Velocity.x=delta_Velocity.c0;
			velocity_data.delta_Velocity.y=delta_Velocity.c1;
			velocity_data.delta_Velocity.z=delta_Velocity.c2;
			pubvelocity_data.publish(velocity_data);
			
		}
	}
	

}
//defining timer publication
void publish_timer(const ros::TimerEvent&)
{
	publish_device();
}
//Function defining possible errors in using vectornav ins
void vnerror_msg(VN_ERROR_CODE vn_error,std::string &msg)
{
	switch(vn_error)
	{
		case VNERR_NO_ERROR:
		  msg="No Error";
		  break;
		case VNERR_UNKNOWN_ERROR:
		  msg="UnKnownError";
		  break;
		case VNERR_NOT_IMPLEMENTED:
		  msg="Not implemented";
		  break;
		case VNERR_TIMEOUT:
		  msg="TimeOut";
		  break;
		case VNERR_INVALID_VALUE:
		  msg="Invalid value";
		  break;
		case VNERR_FILE_NOT_FOUND:
		  msg="File not Found";
		  break;
		case VNERR_NOT_CONNECTED:
		  msg="Not Connected";
 		 break;
		default:
		  msg="Undefined error";
	}
}
bool send_data(tiburon::YPR::Request &req,
               tiburon::YPR::Response &res)
{
	if(req.ins=="sendYPR")
	{
		VnYpr ypr;
		vn100_getYawPitchRoll(&vn100,&ypr);
		res.data[0]=ypr.yaw;
		res.data[1]=ypr.pitch;
		res.data[2]=ypr.roll;
		ROS_INFO("sending response YPR:%f,%f,%f",res.data[0],res.data[1],res.data[2]);
	}
	return true;
}
int main(int argc,char** argv)
{
	ros::init(argc,argv,"tiburon"); ////initializing ros node
	ros::NodeHandle n;
	ros::NodeHandle np("~");        //creating public and private nodehandlers to ha                               ndle ros publish services and private parameters
	std::string port;
	int baudrate,publish_rate,async_output_rate,async_output_type;
	std::string enable,headingMode,filteringMode,tuningMode;
	np.param<std::string>("serial_port",port,"/dev/auv_vn100");
	np.param<int>("serial_baud",baudrate,115200);
	np.param<int>("publish_rate",publish_rate,60);
	np.param<int>("async_output_type",async_output_type,0);
	np.param<int>("async_output_rate",async_output_rate,6);//assigning params to                                                            variables
        np.param<std::string>("VPE_enable",enable,"1");
	np.param<std::string>("VPE_headingMode",headingMode,"2");
        np.param<std::string>("VPE_filteringMode",filteringMode,"1");
	np.param<std::string>("VPE_tuningMode", tuningMode,"1");

	pubsens_data  	 	=np.advertise<tiburon::sensor_data> ("sensor_data",1);
	pubins_data    		=np.advertise<tiburon::ins_data> ("ins_data",1);//Initializing
	pubvelocity_data	=np.advertise<tiburon::delta_velocity_data>("Delta_Velocity",1);                                                                   

	ros::ServiceServer service=n.advertiseService("query_ins_data",send_data);
	ROS_INFO("Ready to answer your queries regarding ins data");
	VN_ERROR_CODE vn_err;         //dealing with vectornav errors
	std::string vn_error_msg;
	ROS_INFO("connecting to vn100. port: %s at a baudrate:%d\n",
	                        port.c_str(),
	                        baudrate);
	vn_err=vn100_connect(&vn100,port.c_str(),baudrate);//connecting to vectornav
	if(vn_err!=VNERR_NO_ERROR)    //in case of any error
	{
		vnerror_msg(vn_err,vn_error_msg);
		ROS_FATAL("Could not connect to the sensor on this %s port error:%s\n did you ad           d the user to the dialout group???",
       	        port.c_str(),
	          vn_error_msg.c_str() 
		);
		exit(EXIT_FAILURE);
	}

	//enable vpe funtion 
	
	vn_err = vn100_getVpeControl(&vn100,(unsigned char*) enable.c_str(),(unsigned char*) headingMode.c_str(),(unsigned char*)filteringMode.c_str(),(unsigned char*) tuningMode.c_str());
	if(vn_err!=VNERR_NO_ERROR)
	{
		vnerror_msg(vn_err,vn_error_msg);
		ROS_FATAL("could not able to enable the VPE");	
		exit(EXIT_FAILURE);
	}

	vn_err=vn100_setAsynchronousDataOutputType(&vn100,async_output_type,true);
	if(vn_err!=VNERR_NO_ERROR)
	{
		vnerror_msg(vn_err,vn_error_msg);
		ROS_FATAL("Could not set the output mode error:%s",
                vn_error_msg.c_str());
                exit(EXIT_FAILURE);
	}

//enabling ros timer to publish the data at the particular intervals
	ros::Timer pub_timer;
   	if(async_output_type ==0)
	{
	//publishing loop
		ROS_INFO("publishing at %d Hz\n",publish_rate);
		pub_timer=np.createTimer(ros::Duration(1.0/(double)publish_rate),publish_timer);
	}
	else
	{
		if (async_output_rate==1|async_output_rate==2|async_output_rate==4|async_output_rate==5|async_output_rate==10|async_output_rate==20|async_output_rate==25|async_output_rate==40|async_output_rate==50|async_output_rate==100|async_output_rate==200)
		{
			ROS_INFO("asynchronous output is subscribed at %d Hz\n",async_output_rate);
		}
		else
		{
			ROS_ERROR("Invalid Async rate %d Hz\n Valid rates:{1,2,4,5,10,25,40,50,100,200}",async_output_rate);
		}
	}
	vn100_setAsynchronousDataOutputFrequency(&vn100,async_output_rate,true);
	vn100_registerAsyncDataReceivedListener(&vn100,&asyncDataListener);
	ros::spin();
	vn100_disconnect(&vn100);
	return 0;
}
