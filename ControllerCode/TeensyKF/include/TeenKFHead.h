#include <Arduino.h>
#include <QuadEncoder.h>
#include <ADC.h>
#include <ADC_util.h>
#include <SoftwareSerial.h>
#include <math.h>
#include "ODriveArduino.h"

#include <BasicLinearAlgebra.h>
using namespace BLA;

template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

//ODriveArduino odrive(Serial3);

bool Debug = false;
int state = 0;

#define PPR 4096
#define YConversion 0.3       //Gear Ratio on Z of 18/60
#define XConversion 0.6       //Fix later
#define LBoom 2.045

#define loopTime 500     
#define max_length 0.135

#define fireHeightBoom 125
#define fireHeightLeg 0.05

#define Ard_baud 500000

// PIN MAPPING
#define RX_serial_PC 0          //0              
#define TX_serial_PC 1          //1
#define X_enc_A 2               //2
#define X_enc_B 3               //3
#define Y_enc_A 4               //4
#define Y_enc_B 5               //5
#define Z_enc_A 6               //6
#define Z_enc_B 7               //7
#define solenoidPin 8           //8
#define KILL 9                  //9
#define INIT 10                 //10
#define foot_sensor 11          //11
                                //12
#define DebugLED 13             //13
#define TX_motor 14             //14-A0    SERIAL 3 motor comms TX 
#define RX_motor 15             //15-A1    SERIAL 3 motor comms RX
#define leg_length_pin A2       //16-A2 
                                //17-A3    
                                //18-A4
#define START 19                //19-A5
#define TX_LiDAR 20             //20-A6    
#define RX_LiDAR 21             //21-A7   
                                //22-A8
                                //23-A9


// comms to pc
int PC_baud=500000;

// comms to motor
#define motor_baud 250000
int motor_pos;
int motor_vel;
int motor_current;
bool commsWait();

// ADC 420 pin for leg length
ADC *adc = new ADC();
float leg_length;
float leg_length_old;
float leg_velocity;
float adc_offset;
float adc_range;

// encode variables
float boom_X_current_pos;
float boom_X_old_pos=0;
int32_t boom_X_CTS;

float boom_Y_current_pos;
float boom_Y_old_pos=0;
int32_t boom_Y_CTS;

float boom_Z_current_pos;
float boom_Z_old_pos=0;
int32_t boom_Z_CTS;

QuadEncoder boom_X_enc(1, X_enc_A, X_enc_B, 0);//Enc 1, Phase A (pin0), PhaseB(pin1), Pullups Req(0)
QuadEncoder boom_Y_enc(2, Y_enc_A, Y_enc_B, 0);//Enc 2, Phase A (pin2), PhaseB(pin3), Pullups Req(0)           
//QuadEncoder boom_Z_enc(3, Z_enc_A, Z_enc_B, 0);//Enc 3, Phase A (pin4), PhaseB(pin5), Pullups Req(0)
float boom_X_current_vel;
float boom_Y_current_vel;
float boom_Z_current_vel;

// LiDAR variables
float Lidar;
float LidarOffset;

// MPU Variables
float Ax;
float Az;
uint16_t timeMPULast;

byte bufTemp[13] = {0};
byte message[7] = {0};

// KF Vars
#define R 0.000625
#define dAmax 3000
int KFcnt = 0;
int i = 0;
int temp, tempP;
uint16_t timePrev, timeNow;
float deltaT = (1/2000);

boolean AxUP = false;

BLA::Matrix<3,3> F;
BLA::Matrix<3,3> Q;
BLA::Matrix<3> X_bar;
BLA::Matrix<3> X_bartemp;
BLA::Matrix<1> B;
BLA::Matrix<3> u;
BLA::Matrix<3,3> P;
BLA::Matrix<3,3> P_bar;
BLA::Matrix<1,3> H;
BLA::Matrix<3> Xm;

float Position = 0;

void updateF(float Fup);
void updateQ(float axCovA,float axCovB,float axCovC);

void writeBytes(int32_t data);

void send_pc_data()
{
  if(Debug)
  {
    //Serial.print("Current position boom X: "); Serial.println(boom_X_current_pos);
    //Serial.print("Current position boom Y: "); 
    //Serial.println(boom_Y_current_pos);
    float dat = Lidar;
    //Serial.println(leg_length*1000);
    //Serial.print("Ax: ");Serial.println(Ax);
    //Serial.print("Az: ");Serial.println(Az);
    if(dat > 300 || dat < 0)
    {
      //Serial.println(dat);
    }
    Serial.println(boom_X_current_pos);
    //Serial.println(dat);
    //Serial.printf("Current position boom Z: %ld\r\n", boom_Z_current_pos);
    //Serial.printf("Current position leg angle: %ld\r\n", leg_angle_current_pos);
    /*
    Serial.print("Current ADC 420 value: "); Serial.println(leg_length);
    Serial.print("Current KF y: "); Serial.println(X_bar(0));
    Serial.print("Current KF dy: "); Serial.println(X_bar(1));
    Serial.print("Current KF ddy: "); Serial.println(X_bar(2));
    
    Serial.print("Lidar: "); Serial.println(Lidar);
    Serial.print("Ax: "); Serial.println(Ax);
    Serial.print("Az: "); Serial.println(Az);
    */
    //Serial.printf("Current motorData: %ld\r\n", motor_data.torque);
    //Serial.println();
  }
  else
  {
    
    int32_t ypos = boom_Y_current_pos*1000;
    writeBytes(ypos);
    int32_t yvel = boom_Y_current_vel*1000;
    writeBytes(yvel);

    int32_t xpos = boom_X_current_pos*1000;
    writeBytes(xpos);
    //int32_t xvel = boom_X_current_vel*1000;
    //writeBytes(xvel);
    int32_t enc = Position*1000;
    writeBytes(enc);
    
    int32_t legLenWrite = leg_length*1000000;
    writeBytes(legLenWrite);
    int32_t legVelWrite = leg_velocity*1000000;
    writeBytes(legVelWrite);
    
    int32_t lidarWrite = (Lidar-LidarOffset)*1000000;
    writeBytes(lidarWrite);
    
    int32_t AxWrite = Ax*1000;
    writeBytes(AxWrite);
    int32_t AzWrite = Az*1000;
    writeBytes(AzWrite);

    int32_t XmatDat0 = X_bar(0)*1000;
    writeBytes(XmatDat0);
    int32_t XmatDat1 = X_bar(1)*1000;
    writeBytes(XmatDat1);
    int32_t XmatDat2 = X_bar(2)*1000;
    writeBytes(XmatDat2);
    
  }
}
void writeBytes(int32_t data)
{
  byte* dataByte = (byte*) &data;
  byte buffer[4] = {0};

  memcpy(buffer, dataByte, 4);

  for(byte i=0;i<sizeof(buffer);i++)
  {
    Serial.write(buffer[i]);
  }
}

void get_motor_pos()
{
  
}

void get_leg_length()
{
  // read leg length 420 ADC
  uint16_t adc_reading = analogRead(leg_length_pin);
  
  // convert to leg length
  leg_length=(adc_reading-adc_offset)*max_length/adc_range;
  
  //clip for noise
  if(leg_length>max_length)
  {
    leg_length=max_length;
    adc_range = adc_reading;
  }
  else if(leg_length<0)
  {
    leg_length=0;
  }
  if(leg_length>=0.124)
  {
    leg_velocity=0;
  }
  else
  {
    leg_velocity = (leg_length-leg_length_old)*loopTime;
  }
  leg_length_old = leg_length;
}
void get_encoder_values()
{
  //int micro_delay=1000;

  // get encoder values
  boom_X_old_pos = boom_X_current_pos;
  boom_Y_old_pos = boom_Y_current_pos;
  //boom_Z_old_pos = boom_Z_enc.read()/PPR*M_PI;
  
  boom_X_CTS = boom_X_enc.read();
  boom_Y_CTS = boom_Y_enc.read();
  if (boom_Y_CTS > 2048)
  {
    boom_Y_CTS = boom_Y_CTS-4096;
  }
  if (boom_X_CTS > 2048)
  {
    boom_X_CTS = boom_X_CTS-4096;
  }

  //boom_Y_current_pos = LBoom*sin(boom_Y_CTS*2*M_PI*YConversion/PPR);
  boom_Y_current_pos = (boom_Y_CTS*M_PI/PPR);
  boom_X_current_pos = (LBoom)*(boom_X_CTS*M_PI*(44/28)/PPR)/0.6;
  //boom_Z_current_pos = boom_Z_enc.read()/PPR*M_PI;

  // calculate encoder vel
  boom_X_current_vel=(boom_X_current_pos-boom_X_old_pos)*loopTime;
  boom_Y_current_vel=(boom_Y_current_pos-boom_Y_old_pos)*loopTime;
  //boom_Z_current_vel=(boom_Z_current_pos-boom_Z_old_pos)*1000000/micro_delay;
  //Serial.write("Got Encoder");
} 
int read_Ard()
{
  int dat = -1;
  while(Serial5.available())
  {
    //Serial.println("READ");
    tempP = temp;
    temp = Serial5.read();
    bufTemp[i] = temp;
    if (temp == 88 && tempP == 88)
    {
      if (i-7<0)
      {
        memcpy(message, bufTemp + 13 + (i-7), 7-i);
        memcpy(message+7-i, bufTemp, i);
      }
      else
      {
        memcpy(message, bufTemp +(i-7), 7);
      }

      byte buftemp[4] = {0};
      memcpy(buftemp,message+1,sizeof(buftemp));
      float x = *((float*)(buftemp));
      
      if (message[0] == 0)
      {
        if (x<300 && x>-0.1)
        {
          Lidar = x;
          Lidar = Lidar/100;
          dat=0;
          //Serial.println(Lidar);
        }
        else
        {
          Serial5.flush();
        }
      }
      else if(message[0] == 1)
      {
        if (Ax<100)
        {
          Ax = x;
          //dat=1;
          if (dat != 0)
          {
            dat = 1;
          }
          AxUP = true;
          }
          //Serial.println(Ax);
      }
      else if(message[0] == 2)
      {
        if (Az<16.5)
        {
          Az = x;
          if (dat != 0 || dat != 1)
          {
            dat = 2;
          }
        }
      }
      timeNow = micros();
      int count = 0;
      for(uint8_t j=0; j<sizeof(message); j++)
      {
        if (message[j]&0b1)
        {
          count++;
        }
      }
      if (count%2 != 1)
      {
       //Serial.println("Bad parity");
      }
    }
      
    i++;
    if (i>13)
    {
      i=0;
    }
  }
  return dat;
}
//////////////////////////////////////////////////////
/////// SETUP FUNCTIONS //////////////////////////////
//////////////////////////////////////////////////////
void setup_420()
{
  // setup ADC for 420 leg length
  pinMode(leg_length_pin, INPUT);
  analogReadRes(12);          // set ADC resolution to this many bits
  analogReadAveraging(50);    // average this many readings
}
void setup_encoders()
{
  boom_X_enc.setInitConfig();  //
  boom_X_enc.EncConfig.revolutionCountCondition = ENABLE;
  boom_X_enc.EncConfig.enableModuloCountMode = ENABLE;
  boom_X_enc.EncConfig.positionModulusValue = PPR; 
  boom_X_enc.EncConfig.positionInitialValue = 0;
  boom_X_enc.init();
  
  boom_Y_enc.setInitConfig();  //
  boom_Y_enc.EncConfig.revolutionCountCondition = ENABLE;
  boom_Y_enc.EncConfig.enableModuloCountMode = ENABLE;
  boom_Y_enc.EncConfig.positionModulusValue = PPR; 
  boom_Y_enc.EncConfig.positionInitialValue = 0;
  boom_Y_enc.init();
  /*
  boom_Z_enc.setInitConfig();  //
  boom_Z_enc.EncConfig.revolutionCountCondition = ENABLE;
  boom_Z_enc.EncConfig.enableModuloCountMode = ENABLE;
  boom_Z_enc.EncConfig.positionModulusValue = 1024; 
  boom_Z_enc.EncConfig.positionInitialValue = 0;
  boom_Z_enc.init();
  */
}
void setup_GPIO()
{
  //setup KILL switch
  pinMode(KILL, INPUT_PULLUP);
  //setup INIT switch
  pinMode(INIT, INPUT_PULLUP);
  //setup START switch
  pinMode(START, INPUT_PULLUP);
  //setup debug LED
  pinMode(DebugLED, OUTPUT);
  //setup solenoid
  pinMode(solenoidPin, OUTPUT);
  digitalWrite(solenoidPin, LOW);
  //setup foot sensor
  pinMode(foot_sensor, INPUT_PULLUP);
}

void setup_motor_comms()
{
  /*
  Serial3.begin(motor_baud);
  int motornum = '0'-'0';
  int requested_state;

  requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
  if(Debug){Serial.println("Motor Calibration");}
  odrive.run_state(motornum, requested_state, true);
  
  requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  if(Debug){Serial.println("Encoder Calibration");}
  odrive.run_state(motornum, requested_state, true);

  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(Debug){Serial.println("Closed Loop Control Test");}
  odrive.run_state(motornum, requested_state, true);*/
  Serial3.begin(motor_baud);
  Serial3.flush();
}
void setup_Ard_comms()
{
  Serial5.begin(Ard_baud);
  Serial5.flush();
}
void setup_pc_comms()
{
  Serial.begin(PC_baud);
}

void setup_KF()
{
  F.Fill(0);
  Q.Fill(0);
  X_bar.Fill(0);
  X_bartemp.Fill(0);
  B.Fill(0);
  P.Fill(0);
  H.Fill(0);
  Xm.Fill(0);

  B(0)=1;

  P <<  0.002,  0,      0,
        0,      0.001,  0,
        0,      0,      0.001;

  H(0,0) = 1;      H(0,1)=0;    H(0,2)=0;

  int temp = read_Ard();
  float LidarTot = 0;
  
  for(int j=0; j<10; j++)
  {
    while (temp!=0)
    {
      temp = read_Ard();
    }
  }
  for(int k=0; k<100; k++)
  {
    while (temp!=0)
    {
      temp = read_Ard();
    }
    LidarTot += Lidar;
  }
  LidarOffset = LidarTot/100;
  
  //LidarOffset = 0.25;
  Xm(0)=(Lidar-LidarOffset);
  Xm(1)=0;
  Xm(2)=0;

  updateF(0);
  updateQ(0,0,0);
}

//////////////////////////////////////////////////////
////////// KF FUNCTIONS //////////////////////////////
//////////////////////////////////////////////////////
void updateQ(float axCovA, float axCovB, float axCovC)
{
  /*Q <<  pow(deltaT,4)/4,  pow(deltaT,3)/2, pow(deltaT,2)/2,
        pow(deltaT,3)/2,  pow(deltaT,2),   deltaT,
        axCovA,           axCovB,          axCovC;*/
  Q <<  1.5625e-14,       6.25e-11,        1.25e-7,
        6.25e-11,         1.25e-7,         0.0005,
        axCovA,           axCovB,          axCovC;
}
void updateF(float Fup)
{
  /*F <<  1,  deltaT, pow(deltaT,2)/2,
        0,  1,      deltaT,
        0,  0,      Fup;*/
  F <<  1,   0.0005,    1.25e-7,
        0,   1,         0.0005,
        0,   0,         Fup;
}

void predictKF()
{
  //----------------------------Predict--------------------------
  float AXtemp = -Ax*9.81;
  u(0) = 0;
  u(1) = 0;
  u(2) = AXtemp;
  X_bar = F*Xm + u;

  BLA::Matrix<3,3> P_bartemp;
  BLA::Matrix<3,3> Ft = ~F;
  
  P_bar = F * P * Ft;
  float daTemp = 0;
  float axCovA, axCovB, axCovC;
  if (Ax > 14 || AXtemp>-5)
  {
    axCovA = pow(deltaT,2)/2;
    axCovB = deltaT;
    axCovC = 1;
    daTemp = 3000;//100000;
  }
  else
  {
    axCovA=0;
    axCovB=0;
    axCovC=1.26736e-7/3000;
    daTemp = 500;//3000;
  }
  updateQ(axCovA, axCovB, axCovC);
  Q = Q*daTemp;//dAmax;
  P_bar += Q;
}
void updateKF(int dat)
{
  //--------------------------------Update------------------------------
  BLA::Matrix<3,3> Eye3 = {1, 0,  0,
                         0, 1,  0,
                         0, 0,  1};
  
  if (dat==0 && (Lidar-LidarOffset)>-0.2)//KFcnt<4)
  {
    BLA::Matrix<1> y = {Lidar-LidarOffset};
    y-= H*(X_bar);
    BLA::Matrix<3> K;
    BLA::Matrix<3> Ht = ~H;

    BLA::Matrix<1> temp = H*P_bar*Ht;
    temp+=R;
    temp = temp.Inverse();
    K = P_bar*Ht;
    K = K*temp;
      
    P = (Eye3 - K*H)*P_bar;
    Xm = X_bar + K*y;
    KFcnt=0;
  }
  else
  {
    Xm = X_bar;
    P = Eye3*P_bar;

    KFcnt++;
  }
}

float motorCom (uint8_t cmdType)
{
  Serial3.write(cmdType);
  
  bool readtest = commsWait();
  
  byte bufTemp[4] = {0};
  if (readtest == 0)
  {
    for(int i=0;i<4;i++)
    {
      bool readtest = commsWait();
      if (readtest == 0)
      {
        bufTemp[i] = Serial3.read();
      }
      else
      {
        break;
      }
    }
  }
  
  float x = *((float*)(bufTemp));
  return x;
}

float motorCom (uint8_t cmdType, float cmdVal)
{
  Serial3.write(cmdType);
  float tempSend = cmdVal;
  byte* dataByte = (byte*) &tempSend;
  byte buffer[4] = {0};

  memcpy(buffer, dataByte, 4);

  Serial3.write(buffer,4);
  return 1;
}

bool commsWait()
{
  uint16_t currtime = micros();
  bool comTimeout = false;
  while(Serial3.available()==0 && comTimeout==false)
  {
    if (micros() - currtime > 2000)
    {
      comTimeout = true;
      //Serial.println("timeout");
    }
  }
  return comTimeout;
}