#define thrusterrelay_1 8
#define thrusterrelay_2 10
#define motherboard_on 2
#define ledPin 13

#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>

MS5803 sensor(ADDRESS_HIGH);

int ledState = 1;
unsigned long prev,current,prevCallbackTime;
bool connectedTiburon = 0;
double pressure_abs;
int depthSensorOn = 0, prevDepthSensorOn = 0, globalStatus = 0;
String str,temp;
char c;
void checkConnectionCallback(int data)
{
  connectedTiburon = 1;
  prevCallbackTime = millis();
}

void depthSensorCallback(int data)
{
  prevDepthSensorOn = depthSensorOn;
  depthSensorOn = data;
}

void tsCb(int data)
{
  if(data==1)
  {

  }
  else if(data==2 && connectedTiburon)
  {
    globalStatus = 2;
    digitalWrite(thrusterrelay_1,HIGH);
    digitalWrite(thrusterrelay_2,HIGH);
  }
  else if(data==3)
  {
    digitalWrite(thrusterrelay_1,LOW);
    digitalWrite(thrusterrelay_2,LOW);
  }
}
void setup()
{
  Serial.begin(9600);
  pinMode(motherboard_on,OUTPUT);
  delay(1000);
  digitalWrite(motherboard_on,HIGH);
  Wire.begin();

  pinMode(thrusterrelay_1,OUTPUT);
  pinMode(thrusterrelay_2,OUTPUT);
  digitalWrite(thrusterrelay_1,LOW);
  digitalWrite(thrusterrelay_2,LOW);

  prev = millis();
  
  globalStatus = 1;

}

void loop()
{
  Serial.println("S"+String(globalStatus)+"#");
  Serial.println("U"+String(digitalRead(6)*10 + digitalRead(7))+"#");
  str = "";
  if(Serial.available())
  {
    c = ' ';
    while(c!='\n')
    {
      c = Serial.read();
      if((c>='0' && c<='9') || (c>='A' && c<='Z') || c=='#')
        str += c;
    }
    for(int i=0;i<str.length();i++)
    {
      if(str[i]=='T')
      {
        i++;
        temp = "";
        for(;;i++)
          if(str[i]!='#') temp+=str[i];
          else break;
        tsCb(temp.toInt());
      //Serial.println(temp.toInt());
      }
      if(str[i]=='P')
      {
        i++;
        temp = "";
        for(;;i++)
          if(str[i]!='#') temp+=str[i];
          else break;
        //Serial.println("     "+temp.toInt());
        depthSensorCallback(temp.toInt());
        
      }
      if(str[i]=='C')
      {
        i++;
        temp = "";
        for(;;i++)
          if(str[i]!='#') temp+=str[i];
          else break;
        checkConnectionCallback(temp.toInt());
        //Serial.println("C     "+temp.toInt());
      }
    }
  }
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
    pressure_abs = sensor.getPressure(ADC_4096);
    Serial.println("D"+String(pressure_abs)+"#");
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
}
