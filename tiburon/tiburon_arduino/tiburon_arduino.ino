
/*
2 MOTHERBOARD_SHORT
8 BATTERY1_RELAY_ON
9 BATTERY2_RELAY_ON
*/
#define thrusterrelay_1 8              
#define thrusterrelay_2 9             
//#define nano_relay 12                  
#define motherboard_on 2 

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <Servo.h> 
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>
#include <std_msgs/Float64.h>

ros::NodeHandle  nh;

MS5803 sensor(ADDRESS_HIGH);
Servo front,back,left,right;
int i=0,j=0,k=0,l=0;
float temperature_c, temperature_f;
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;
float array[4];

double base_altitude = 1655.0; 

void fpsCb( const std_msgs::UInt16& msg)
{
  front.writeMicroseconds(msg.data);
}
void bpsCb(const std_msgs::UInt16& msg)
{
  back.writeMicroseconds(msg.data);
}
void lsCb(const std_msgs::UInt16& msg)
{
  left.writeMicroseconds(msg.data);
}
void rsCb(const std_msgs::UInt16& msg)
{
  right.writeMicroseconds(msg.data);
}
void tsCb(const std_msgs::UInt16& msg)
{
  if(msg.data==1)
  {
    
  }
  else if(msg.data==2)
  {
    digitalWrite(thrusterrelay_1,HIGH);
    digitalWrite(thrusterrelay_2,HIGH);
    
  }
  else if(msg.data==3)
  {
   
    digitalWrite(thrusterrelay_1,LOW);
     digitalWrite(thrusterrelay_2,LOW);
    
  }
}

std_msgs::Float64 depth_msg;

ros::Subscriber<std_msgs::UInt16> fpssub("frontpitchspeed", &fpsCb );
ros::Subscriber<std_msgs::UInt16> bpssub("backpitchspeed",&bpsCb);
ros::Subscriber<std_msgs::UInt16> lssub("sideleftspeed",&lsCb);
ros::Subscriber<std_msgs::UInt16> rssub("siderightspeed", &rsCb);
ros::Subscriber<std_msgs::UInt16> tstate("thrusterstate",&tsCb);
ros::Publisher  pub_depth("/depth_value",&depth_msg);


void setup()
{
    
   nh.initNode();
   Wire.begin();
   
   //pinMode(nano_relay,OUTPUT);
   pinMode(motherboard_on,OUTPUT);

   pinMode(thrusterrelay_1,OUTPUT);
   pinMode(thrusterrelay_2,OUTPUT);

   sensor.reset();
   sensor.begin();
    
   nh.subscribe(fpssub);
   nh.subscribe(bpssub);
   nh.subscribe(lssub);
   nh.subscribe(rssub);
   nh.subscribe(tstate);
   nh.advertise(pub_depth);
   pressure_baseline = sensor.getPressure(ADC_4096);
   
   delay(1000);


  digitalWrite(thrusterrelay_1,LOW);
  digitalWrite(thrusterrelay_2,LOW);

//  digitalWrite(nano_relay,HIGH);
  
  
  digitalWrite(motherboard_on,HIGH);
  //delay(1000);
 // digitalWrite(motherboard_on,LOW);
}

void loop()
{  
  pressure_abs = sensor.getPressure(ADC_4096);
  depth_msg.data = pressure_abs;
  pub_depth.publish(&depth_msg);
  nh.spinOnce();
  delay(1);
}
  
  
// Thanks to Mike Grusin for letting me borrow the functions below from 
// the BMP180 example code. 

 double sealevel(double P, double A)
// Given a pressure P (mbar) taken at a specific altitude (meters),
// return the equivalent pressure (mbar) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
	return(P/pow(1-(A/44330.0),5.255));
}


double altitude(double P, double P0)
// Given a pressure measurement P (mbar) and the pressure at a baseline P0 (mbar),
// return altitude (meters) above baseline.
{
	return(44330.0*(1-pow(P/P0,1/5.255)));
}
