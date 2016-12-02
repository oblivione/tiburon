

/*
2 MOTHERBOARD_SHORT
 8 BATTERY1_RELAY_ON
 10 BATTERY2_RELAY_ON
 */
#define thrusterrelay_1 8
#define thrusterrelay_2 10
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
int ledState = 1;
unsigned long prev,current,prevCallbackTime;
bool connectedTiburon = 0;
int depthSensorOn = 0, prevDepthSensorOn = 0, globalStatus = 0;


void checkConnectionCallback(const std_msgs::UInt16& msg)
{
  connectedTiburon = 1;
  globalStatus = 1;
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
  else if(msg.data==2 && connectedTiburon)
  {
    digitalWrite(thrusterrelay_1,HIGH);
    digitalWrite(thrusterrelay_2,HIGH);
    globalStatus = 2;

  }
  else if(msg.data==3)
  {

    digitalWrite(thrusterrelay_1,LOW);
    digitalWrite(thrusterrelay_2,LOW);

  }
}

std_msgs::Float64 depth_msg;
std_msgs::UInt16 uint16_msg;
ros::Subscriber<std_msgs::UInt16> tstate("thrusterstate",1,&tsCb);
ros::Publisher  pub_depth("/depth_value",1);
ros::Subscriber<std_msgs::UInt16> depthSensor("/depthSensorState",1,&depthSensorCallback );
ros::Subscriber<std_msgs::UInt16> connectionChecker("/tiburonConnection",1,&checkConnectionCallback );
ros::Publisher statusConnection("/auvStatus",1);
ros::Publisher underwater("/underwaterStatus",1);

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
  nh.advertise(statusConnection);
  nh.advertise(underwater);
  nh.subscribe(connectionChecker);
  // pressure_baseline = sensor.getPressure(ADC_4096);
  digitalWrite(thrusterrelay_1,LOW);
  digitalWrite(thrusterrelay_2,LOW);

  prev = millis();

}

void loop()
{
  uint16_msg.data =globalStatus;
  statusConnection.publish(&uint16_msg);
  uint16_msg.data = digitalRead(6)*10 + digitalRead(7);
  underwater.publish(&uint16_msg);
  current = millis();
  if(current-prev>1000)
  {
    ledState = 1-ledState;
    digitalWrite(ledPin,ledState);
    prev = current;
  }
  if(prevDepthSensorOn == 0 && depthSensorOn == 1)
  {
    globalStatus = 3;
    sensor.reset();
    sensor.begin();
  }
  if(depthSensorOn == 1)
  {
    depth_msg.data = sensor.getPressure(ADC_4096);
    pub_depth.publish(&depth_msg);
  }
  if(prevDepthSensorOn == 1 && depthSensorOn == 0)
  {
    sensor.reset();
  }
  if(connectedTiburon && millis()-prevCallbackTime>5000)
  {
    connectedTiburon = 0;
    digitalWrite(thrusterrelay_1,LOW);
    digitalWrite(thrusterrelay_2,LOW);
  }
  nh.spinOnce();
  delay(1);
}
