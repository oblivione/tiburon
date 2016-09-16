
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
unsigned long prev,current,prevCallbackTime;
bool connected = 0;
float temperature_c, temperature_f;
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;
float array[4];
double base_altitude = 1655.0;

int depthSensorOn = 0, prevDepthSensorOn = 0;


void checkConnectionCallback(const std_msgs::UInt16& msg)
{
  connected = 1;
  prevCallbackTime = millis();
}

void depthSensorCallback(const std_msgs::UInt16& msg)
{
  prevDepthSensorOn = depthSensorOn;
  depthSensorOn = msg.data;
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
ros::Subscriber<std_msgs::UInt16> tstate("thrusterstate",&tsCb);
ros::Publisher  pub_depth("/depth_value",&depth_msg);
ros::Subscriber<std_msgs::UInt16> depthSensor("/depthSensorState",&depthSensorCallback );
ros::Subscriber<std_msgs::UInt16> connectionChecker("/tiburonConnection",&checkConnectionCallback );

void setup()
{
  pinMode(motherboard_on,OUTPUT);
  delay(1000);
  digitalWrite(motherboard_on,HIGH);
  nh.initNode();
  Wire.begin();

  pinMode(thrusterrelay_1,OUTPUT);
  pinMode(thrusterrelay_2,OUTPUT);
  nh.subscribe(depthSensor);
  nh.subscribe(tstate);
  nh.advertise(pub_depth);
  // pressure_baseline = sensor.getPressure(ADC_4096);
  digitalWrite(thrusterrelay_1,LOW);
  digitalWrite(thrusterrelay_2,LOW);

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
  if(prevDepthSensorOn == 0 && depthSensorOn == 1)
  {
    sensor.reset();
    sensor.begin();
  }
  if(depthSensorOn == 1)
  {
    pressure_abs = sensor.getPressure(ADC_4096);
    depth_msg.data = pressure_abs;
    pub_depth.publish(&depth_msg);
  }
  if(prevDepthSensorOn == 1 && depthSensorOn == 0)
  {
    sensor.reset();
  }
  if(connected && millis()-prevCallbackTime>5000)
  {
    connected = 0;
    digitalWrite(thrusterrelay_1,LOW);
    digitalWrite(thrusterrelay_2,LOW);
  }
  nh.spinOnce();
  delay(1);
}
