
/*
2 MOTHERBOARD_SHORT
 8 BATTERY1_RELAY_ON
 9 BATTERY2_RELAY_ON
 */
#define thrusterrelay_1 8              
#define thrusterrelay_2 9             
//#define nano_relay 12                  
#define motherboard_on 2 
#define ledPin 13

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>
#include <std_msgs/Float64.h>
//#define notConnected
ros::NodeHandle  nh;

MS5803 sensor(ADDRESS_HIGH);
int i=0,j=0,k=0,l=0,ledState = 1;
unsigned long prev,current;
float temperature_c, temperature_f;
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;
float array[4];
int depthSensorState = 0;
double base_altitude = 1655.0; 

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
ros::Subscriber<std_msgs::UInt16> tstate("thrusterstate",&tsCb);
ros::Publisher  pub_depth("/depth_value",&depth_msg);


void setup()
{
  pinMode(motherboard_on,OUTPUT);
  delay(1000);
  digitalWrite(motherboard_on,HIGH); 
  nh.initNode();
  Wire.begin();
#ifdef notConnected
  Serial.begin(9600);
#endif
  //pinMode(nano_relay,OUTPUT);


  pinMode(thrusterrelay_1,OUTPUT);
  pinMode(thrusterrelay_2,OUTPUT);
#ifdef notConnected
  Serial.println("Hello!");
#endif
  sensor.reset();
#ifdef notConnected
  Serial.println(depthSensorState);
  Serial.println("Hi Again!");
#endif
  depthSensorState = sensor.begin();
#ifdef notConnected
  Serial.println(depthSensorState);
  Serial.println("Hello Again!");
#endif
  nh.subscribe(tstate);
  nh.advertise(pub_depth);
  pressure_baseline = sensor.getPressure(ADC_4096);

  delay(1000);


  digitalWrite(thrusterrelay_1,LOW);
  digitalWrite(thrusterrelay_2,LOW);

  //  digitalWrite(nano_relay,HIGH);




  //delay(1000);
  // digitalWrite(motherboard_on,LOW);
  prev = millis();

}

void loop()
{  
  current = millis();
  if(current-prev>1000)
  {
    ledState = 1-ledState;
    digitalWrite(ledPin,ledState);
    prev = current;
  }
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

