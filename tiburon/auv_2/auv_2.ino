#define thrusterrelay 11
#define thrusterrelay1 10
//#define frontrelay 12
//#define backrelay 8
//#define leftrelay 7
//#define rightrelay 4
//#define obcrelay 9
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <Servo.h> 
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>
#include <std_msgs/Float64.h>
//#include <geometry_msgs/Vector3.h>
//#include <geometry_msgs/Vector3.h>MS5803 sensor(ADDRESS_HIGH);


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
  //digitalWrite(13,HIGH);
  front.writeMicroseconds(msg.data);
  //delay(1000);
  //digitalWrite(13,LOW);
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
    //digitalWrite(13,HIGH);
   // front.writeMicroseconds(1000);
   // back.writeMicroseconds(1000);
  //  left.writeMicroseconds(1500);
   // right.writeMicroseconds(1500);
  //    delay(100);
  //digitalWrite(13,LOW);
  }
  else if(msg.data==2)
  {
   /* digitalWrite(13,LOW);
    digitalWrite(12,HIGH);
    digitalWrite(8,HIGH);
    digitalWrite(7,HIGH);*/
    digitalWrite(11,HIGH);
    digitalWrite(10,HIGH);
    
  }
  else if(msg.data==3)
  {
    /*digitalWrite(13,LOW);
    digitalWrite(12,LOW);
    digitalWrite(8,LOW);
    digitalWrite(7,LOW);*/
    digitalWrite(11,LOW);
     digitalWrite(10,LOW);
  //digitalWrite(13,LOW);
  }
}

std_msgs::Float64 depth_msg;
//std_msgs::Float64 c1_msg;
//std_msgs::Float64 c2_msg;
//std_msgs::Float64 c3_msg;
//geometry_msgs::Vector3 depth_msg;
ros::Subscriber<std_msgs::UInt16> fpssub("frontpitchspeed", &fpsCb );
ros::Subscriber<std_msgs::UInt16> bpssub("backpitchspeed",&bpsCb);
ros::Subscriber<std_msgs::UInt16> lssub("sideleftspeed",&lsCb);
ros::Subscriber<std_msgs::UInt16> rssub("siderightspeed", &rsCb);
ros::Subscriber<std_msgs::UInt16> tstate("thrusterstate",&tsCb);
//ros::Subscriber<std_msgs::UInt16> threv("thrusterreverse",&trsCb);
ros::Publisher  pub_depth("/depth_value",&depth_msg);
//ros::Publisher   b_l_1("/voltage_c1",&depth_msg);
//ros::Publisher   b_l_2("/voltage_c2",&depth_msg);
//ros::Publisher   b_l_3("/voltage_c3",&depth_msg);
//ros::Subscriber<std_msgs::UInt16> mstate("motherboardstate",&msCb);


//ros::ServiceServer<depth_srv::Request, depth_srv::Response> server("depth_srv",&callback);


void setup()
{ 
   nh.initNode();
   Wire.begin();
 // front.attach(6);
 // back.attach(9);
 // left.attach(3);
 // right.attach(5);
 pinMode(12,OUTPUT);
 pinMode(4,OUTPUT);
pinMode(thrusterrelay,OUTPUT);
pinMode(thrusterrelay1,OUTPUT);
/*pinMode(frontrelay,OUTPUT);
pinMode(backrelay,OUTPUT);
pinMode(leftrelay,OUTPUT);
pinMode(rightrelay,OUTPUT);
pinMode(13,OUTPUT);*/

//pinMode(obcrelay,OUTPUT);

sensor.reset();
sensor.begin();
    
nh.subscribe(fpssub);
nh.subscribe(bpssub);
nh.subscribe(lssub);
nh.subscribe(rssub);
nh.subscribe(tstate);
//nh.subscribe(threv);
nh.advertise(pub_depth);
//nh.advertise(b_l_1);
//nh.advertise(b_l_2);

 pressure_baseline = sensor.getPressure(ADC_4096);
 
//nh.subscribe(mstate);
digitalWrite(thrusterrelay,LOW);
/*digitalWrite(frontrelay,LOW);
digitalWrite(backrelay,LOW);
digitalWrite(leftrelay,LOW);
digitalWrite(rightrelay,LOW);*/
//digitalWrite(obcrelay,HIGH);
digitalWrite(12,HIGH);
digitalWrite(4,HIGH);
delay(1000);
digitalWrite(4,LOW);
}

void loop()
{  
  
  /* array[0] = analogRead(A3);
   array[1] = analogRead(A6);
   arrray[2] = analogRead(A7);*/
  
 // c1_msg.data= a;
  //c1_msg.data= b;
  //c1_msg.data= c;
  
  pressure_abs = sensor.getPressure(ADC_4096);
 
  depth_msg.data = pressure_abs;

 pub_depth.publish(&depth_msg);
 // b_l_1.publish(&depth_msg);
   // b_l_2.publish(&c2_msg);
     // b_l_3.publish(&c3_msg);
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
