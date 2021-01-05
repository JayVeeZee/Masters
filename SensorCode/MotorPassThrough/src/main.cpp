#include "ODriveArduino.h"
#include <Arduino.h>
#include <Encoder.h>
#include <math.h>

template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

#define encA 18
#define encB 19
#define PPR 2048*4
#define loopTime 500

Encoder Enc(encA, encB);

ODriveArduino odrive(Serial3);

float EncPos, EncPosPrev, EncVel;
uint32_t timeLast=0;

bool commsWait(void);
float getCmdVal(void);
void writeBytes(float data);

bool light = false;
bool fullread = false;

void setup() {
  //Serial.begin(115200);
  //Serial.println("setup");
  delay(2000);
  Enc.write(0);

  Serial3.begin(115200);
  int motornum = '0'-'0';
  int requested_state;

  requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
  odrive.run_state(motornum, requested_state, true);
  requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  odrive.run_state(motornum, requested_state, true);
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive.run_state(motornum, requested_state, true);
  
  Serial2.begin(250000);
  Serial2.flush();

  odrive.SetCurrent(0,0);
  //Serial.println("setup done");
}

void loop() 
{
  /*while(Serial2.available()==0)
  {
  }*/
  //odrive.SetCurrent(0,0);
  //Serial.println(EncPos);
  timeLast = micros();

  EncPosPrev = EncPos;
  int32_t EncCTS = Enc.read();
  /*if (EncCTS > 2048*4)
  {
    EncCTS = EncCTS - 2048*4;
  }*/
  EncPos = (EncCTS*M_PI/(PPR*5))*2;
  //EncVel = (EncPos-EncPosPrev)/loopTime;
  //Serial.println(EncCTS);
  if(Serial2.available())
  {
    uint8_t dat = Serial2.read();
    bool readTest = true;
    switch(dat)
    {
      case 0: //position control
      {
        readTest = commsWait();
        if (readTest == false)
        {
          float cmdVal = getCmdVal();
          if(fullread)
          {
            odrive.SetPosition(0,cmdVal);
            //Serial.print("P: ");
            //Serial.println(cmdVal);
          }
        }
        break;
      }
      case 1://vel control
      {
        readTest = commsWait();
        if (readTest == false)
        {
          float cmdVal = getCmdVal();
          if(fullread)
          {
            odrive.SetVelocity(0,cmdVal);
            //Serial.print("V: ");
            //Serial.println(cmdVal);
          }
        }
        break;
      }
      case 2://current control
      {
        readTest = commsWait();
        if (readTest == false)
        {
          float cmdVal = getCmdVal();
          if(fullread)
          {
            odrive.SetCurrent(0,cmdVal);
            //Serial.print("C: ");
            //Serial.println(cmdVal);
          }
        }
        break;
      }
      case 3://read pos
      {
        /*float Pos = odrive.GetPosition(0);
        Serial.print("Pos: ");Serial.println(Pos);
        writeBytes(Pos);*/
        writeBytes(EncPos);
        break;
      }
      case 4:// read vel
      {
        /*float Vel = odrive.GetVelocity(0);
        Serial.print("Vel: ");Serial.println(Vel);
        writeBytes(Vel);*/
        //writeBytes(EncVel);
        break;
      }
      /*
      case 5: //read real curr
      {
        float RCurr = odrive.GetCurrentMeasured(0);
        Serial.print("RCurr: ");Serial.println(RCurr);
        writeBytes(RCurr);
        break;
      }
      case 6: //read cmd curr
      {
        float CCurr = odrive.GetCurrentCommand(0);
        Serial.print("CCurr: ");Serial.println(CCurr);
        writeBytes(CCurr);
        break;
      }*/
      default:
        Serial2.flush();
        break;
    }
  }

  while(micros()-timeLast<loopTime)
  {

  }
}
bool commsWait()
{
  uint32_t currtime = micros();
  bool comTimeout = false;
  while(Serial2.available()==0 && comTimeout==false)
  {
    if (micros() - currtime > 2000)
    {
      comTimeout = true;
      //Serial.println("timeout");
      Serial2.flush();
    }
  }
  return comTimeout;
}

float getCmdVal()
{
  bool readtest = commsWait();
  
  fullread = false;
  byte bufTemp[4] = {0};
  if (readtest == 0)
  {
    for(int i=0;i<4;i++)
    {
      bool readtest = commsWait();
      if (readtest == false)
      {
        bufTemp[i] = Serial2.read();
        if (i == 3)
        {
          fullread = true;
          //Serial.println("Full Read");
        }
      }
      else
      {
        break;
      }
    }
  }
  float x = *((float*)(bufTemp));
  //Serial.println(x);
  /*long readDat = 0;
  if (readtest == false)
  {
    readDat = Serial2.read();
    fullread = true;
    Serial.print("read data: ");Serial.println(readDat);
  }
  else
  {
    fullread = false;
  }
  float x = readDat/100000;*/
  return x;
}

void writeBytes(float data)
{
  byte* dataByte = (byte*) &data;
  byte buffer[4] = {0};

  memcpy(buffer, dataByte, 4);

  for(byte i=0;i<sizeof(buffer);i++)
  {
    Serial2.write(buffer[i]);
  }
}