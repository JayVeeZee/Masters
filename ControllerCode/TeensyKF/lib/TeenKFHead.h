#include <Arduino.h>
#include <QuadEncoder.h>
#include <ADC.h>
#include <ADC_util.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

bool Debug = false;

#define PPR 4096
#define YConversion 0.3       //Gear Ratio on Z of 18/60
#define XConversion 0.6       //Fix later
#define LBoom 2045

#define loopTime 500     
#define max_length 125

#define fireHeightBoom 125
#define fireHeightLeg 50

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
int PC_baud=1000000;

// comms to motor
int motor_baud=115200;
int motor_pos;
int motor_vel;
int motor_current;

// ADC 420 pin for leg length
ADC *adc = new ADC();
uint16_t leg_length;
uint16_t leg_length_old;
int16_t leg_velocity;
uint16_t adc_offset;
uint16_t adc_range;

// encode variables
int32_t boom_X_current_pos;
int32_t boom_X_old_pos=0;
int32_t boom_Y_current_pos;
int32_t boom_Y_old_pos=0;
int32_t boom_Y_CTS;
int32_t boom_Z_current_pos;
int32_t boom_Z_old_pos=0;
//QuadEncoder boom_X_enc(1, X_enc_A, X_enc_B, 0);//Enc 1, Phase A (pin0), PhaseB(pin1), Pullups Req(0)
QuadEncoder boom_Y_enc(2, Y_enc_A, Y_enc_B, 0);//Enc 2, Phase A (pin2), PhaseB(pin3), Pullups Req(0)           
//QuadEncoder boom_Z_enc(3, Z_enc_A, Z_enc_B, 0);//Enc 3, Phase A (pin4), PhaseB(pin5), Pullups Req(0)
int32_t boom_X_current_vel;
int32_t boom_Y_current_vel;
int32_t boom_Z_current_vel;

// LiDAR variables
float Lidar;
float LidarOffset;

// MPU Variables
float Ax;
float Az;
uint16_t timeMPULast;

// KF Vars
#define R 6.25
#define dAmax 3000
byte bufTemp[13] = {0};
byte message[7] = {0};

int i = 0;
int temp, tempP;
uint16_t timePrev, timeNow;
float deltaT = (1/2500);

boolean AxUP = false;

BLA::Matrix<3,3> F;
BLA::Matrix<3,3> Q;
BLA::Matrix<3> X_bar;
BLA::Matrix<3> X_bartemp;
BLA::Matrix<3> B;
BLA::Matrix<3,3> P;
BLA::Matrix<3,3> P_bar;
BLA::Matrix<1,3> H;
BLA::Matrix<3> Xm;

void updateF(float Fup);
void updateQ(float axCov);

void send_pc_data()
{
  if(Debug)
  {
    //Serial.print("Current position boom X: "); Serial.println(boom_X_current_pos);
    //Serial.print("Current position boom Y: "); Serial.println(boom_Y_current_pos);
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
    Serial.println();
  }
  else
  {
    //Serial.write(boom_X_current_pos); Serial.print(";");
    Serial.print(boom_Y_current_pos);   Serial.print(";");
    //Serial.write(boom_Z_current_pos); Serial.printf(";");
    //Serial.write(motor_pos);          Serial.printf(";");
    Serial.print(leg_length);           Serial.print(";");
    //Serial.write(boom_X_current_vel); Serial.printf(";");
    //Serial.print(boom_Y_current_vel);   Serial.print(";");
    //Serial.write(boom_Z_current_vel); Serial.printf(";");
    //Serial.write(motor_vel);          Serial.printf(";");
    //Serial.print(leg_velocity);         Serial.print(";");
    Serial.print(Lidar);                Serial.print(";");
    
    //Serial.print(X_bar(0));             Serial.print(";");
    //Serial.print(X_bar(1));             Serial.print(";");
    //Serial.print(X_bar(2));             Serial.print(";");
    
    Serial.print(Ax);                   Serial.print(";");
    Serial.print(Az);                   Serial.println(";");
    //Serial.write(motor_torque);       Serial.printf(";");
    //Serial.write(motor_current);      Serial.println(); 
  }
}
void send_motor_data()
{
  // convert torque to current
  motor_current=0;//TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  Serial3.print("c 0 ");
  Serial3.write(motor_current);
}
void get_motor_data()
{
  Serial3.print("f 0");

  //delay to wait for incoming data
  int i=0;
  while(Serial3.available()<=0 && i<100)
  {
    //delayMicroSeconds(10);
    i++;
  }
  
  String inString="";
  if(Serial3.available()>0)
  {
    int inChar=Serial.read();
    while(inChar != ' ')
    {
      inString +=(char)inChar;
      inChar=Serial.read();
    }
    motor_pos=inString.toFloat();
    inChar=Serial.read();
    inString="";
    while(inChar != '\n')
    {
      inString +=(char)inChar;
      inChar=Serial.read();
    }
    motor_vel=inString.toFloat();

    // TODO convert encoder counts to pos and vel TODO!!!!!!!!!!!!!!
  }
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
  if(leg_length>=124)
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
  //boom_X_old_pos = boom_X_current_pos;
  boom_Y_old_pos = boom_Y_current_pos;
  //boom_Z_old_pos = boom_Z_enc.read()/PPR*M_PI;
  
  //boom_X_current_pos = boom_X_enc.read()/PPR*M_PI;
  boom_Y_CTS = boom_Y_enc.read();
  //Serial.print("Boom Counts: ");Serial.println(boom_Y_CTS);
  if (boom_Y_CTS > 2048)
  {
    boom_Y_CTS = boom_Y_CTS-4096;
  }
  boom_Y_current_pos = LBoom*sin(boom_Y_CTS*2*M_PI*YConversion/PPR);
  //boom_Z_current_pos = boom_Z_enc.read()/PPR*M_PI;
  
  // calculate encoder vel
  //boom_X_current_vel=(boom_X_current_pos-boom_X_old_pos)*loopTime;
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
        Lidar = x/100;
        //Serial.print("Lidar:");Serial.println(Lidar);
        dat=0;
      }
      else if(message[0] == 1)
      {
        Ax = x;
        //Serial.print("Ax:");Serial.println(Ax);
        dat=1;
        if (dat != 0)
        {
          dat = 1;
        }
        AxUP = true;
      }
      else if(message[0] == 2)
      {
        Az = x;
        //Serial.print("Az: ");Serial.println(x);
        if (dat != 0 || dat != 1)
        {
          dat = 2;
        }
      }
      timeNow = micros();
      int count = 0;
      for(int j=0; j<sizeof(message); j++)
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
  /*boom_X_enc.setInitConfig();  //
  boom_X_enc.EncConfig.revolutionCountCondition = ENABLE;
  boom_X_enc.EncConfig.enableModuloCountMode = ENABLE;
  boom_X_enc.EncConfig.positionModulusValue = 1024; 
  boom_X_enc.EncConfig.positionInitialValue = 0;
  boom_X_enc.init();
  */
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
  Serial3.begin(motor_baud);
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

  B(0)=0;
  B(1)=0;
  B(2)=1;

  P <<  0.002,  0,      0,
        0,      0.001,  0,
        0,      0,      0.001;

  H(0,0) = 1;      H(0,1)=0;    H(0,2)=0;

  int temp = read_Ard();
  while (temp!=0)
  {
    temp = read_Ard();
    //Serial.println(temp);
  }
  LidarOffset = Lidar;
  Xm(0)=(0);
  Xm(1)=0;
  Xm(2)=0;

  updateF(0);
  updateQ(1);
}

//////////////////////////////////////////////////////
////////// KF FUNCTIONS //////////////////////////////
//////////////////////////////////////////////////////
void updateQ(float axCov)
{
  Q <<  pow(deltaT,4)/4,  pow(deltaT,3)/2, pow(deltaT,2)/2,
        pow(deltaT,3)/2,  pow(deltaT,2),   deltaT,
        0,                0,               axCov;
}
void updateF(float Fup)
{
  F <<  1,  deltaT, 0.5*pow(deltaT,2),
        0,  1,      deltaT,
        0,  0,      Fup;
}

void predictKF()
{
  //----------------------------Predict--------------------------
  float AXtemp = Ax*9.81;
  X_bar = F*Xm + B*(AXtemp);
  
  
  float Fup=0;
  if(AxUP)
  {
    Fup=0;
    AxUP = false;
  }
  else
  {
    Fup=1;
  }
  updateF(Fup);
  //F(2,2)=Fup;
  BLA::Matrix<3,3> P_bartemp;
  BLA::Matrix<3,3> Ft = ~F;
  //Multiply(P,Ft,P_bartemp);
  //Multiply(F,P_bartemp,P_bar);
  P_bar = F * P * Ft;
  
  float axCov;
  if (Ax > 15)
  {
    axCov = 3000; //Check this
  }
  else
  {
    axCov=0.00034/3000;
  }
  updateQ(axCov);
  Q = Q*dAmax;
  P_bar += Q;
}
void updateKF(int dat)
{
  //--------------------------------Update------------------------------
  BLA::Matrix<3,3> Eye3 = {1, 0,  0,
                         0, 1,  0,
                         0, 0,  1};
  switch(dat)
  {
    case -1:
      break;
    case 0:
      //Lidar data
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
      break;
    case 1:
      Xm = X_bar;
      P = Eye3*P_bar;
  }
}