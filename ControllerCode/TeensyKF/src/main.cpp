#include <QuadEncoder.h>
#include <ADC.h>
#include <ADC_util.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <TeenKFHead.h>
#include <BasicLinearAlgebra.h>

//---------------------------------------------------------------------------------------------------
//----------------------------------Defines for useful constants-------------------------------------
//---------------------------------------------------------------------------------------------------
void init_params();
void wait_control_loop();
void run_controller();
void horzontal_controller();
void testPos();
//-------------------------------------control loop variables----------------------------------------
#define loopTime 500         //Gives control loop 2kHz Frequency
unsigned long previousTime = 0;
unsigned long evalTime = 500;
int motor_torque;
int solenoid = LOW;

bool doneInit = false;
bool started = false;
bool dead = false;

//#define fireAng 5
float fireAng = 10;
int stepCnt =0;
int32_t l = 0;
float motorPosition = 0;

//-------------------------------------------------------------------------------------
//----------------------------------SetUp Functions------------------------------------
//-------------------------------------------------------------------------------------

void setup() {  
  Debug = true;
  if(Debug)
  {
    setup_pc_comms();
    Serial.println("COMMS GO");
    setup_GPIO();
    Serial.println("GPIO GO");
    setup_motor_comms();
    Serial.println("MOTOR COMMS GO");
    setup_encoders();
    Serial.println("ENCODER GO");
    setup_420();
    Serial.println("420 GO");
    setup_Ard_comms();
    Serial.println("ARD COMMS GO");
    digitalWrite(DebugLED, Debug);
  }
  else
  {
    digitalWrite(DebugLED, !Debug);
    setup_GPIO();
    setup_motor_comms();
    setup_pc_comms();
    setup_encoders();
    setup_420();
    setup_Ard_comms();
    digitalWrite(DebugLED, Debug);
  }
}

//-------------------------------------------------------------------------------
//----------------------------------Main Loop------------------------------------
//-------------------------------------------------------------------------------

void loop() 
{
  //------------------Wait for initialisation-------------------------
  //Holds till initalise button is pressed and then initialise the system
  while(doneInit == false)
  {
    //Serial.println("I");
    if(digitalRead(INIT) == LOW)
    {
      float lidarInit = 0;
      digitalWrite(DebugLED, !Debug);
      init_params();
      for(int k=0;k<100;k++)
      {
        get_encoder_values();
        get_leg_length();
        int ReadStat = -1;
        while(ReadStat==-1)
        {
          ReadStat=read_Ard();
        }
        predictKF();
        updateKF(ReadStat);
        wait_control_loop();
      }
      for (int h=0;h<10;h++)
      {
        while(X_bar(0)<-0.001 || X_bar(0)>0.001)
        {
          init_params();
          for(int k=0;k<100;k++)
          {
            get_encoder_values();
            get_leg_length();
            int ReadStat = -1;
            while(ReadStat==-1)
            {
              ReadStat=read_Ard();
            }
            predictKF();
            updateKF(ReadStat);
            wait_control_loop();
          }
        }
        lidarInit = lidarInit + LidarOffset;
      }
      LidarOffset = lidarInit/10;
      /*digitalWrite(solenoidPin, HIGH);
      delay(8000);
      while(X_bar(0)<-0.001 || X_bar(0)>0.001)
      {
        setup_KF();
        for(int k=0;k<10;k++)
        {
          int ReadStat = -1;
          while(ReadStat==-1)
          {
            ReadStat=read_Ard();
          }
          predictKF();
          updateKF(ReadStat);
          wait_control_loop();
        }
      }
      LidarOffset = ((LidarOffset-0.125)+lidarInit)/2;*/
      
      doneInit = true;
      digitalWrite(DebugLED, Debug);
      motorCom(0,0);
      digitalWrite(solenoidPin, LOW);
    }
  }

  //--------------------Hold for system start-------------------------
  //Holds in the while loop till the start button is pressed
  while(started == false)
  {
    if(digitalRead(START) == LOW)
    {
      started = true;
      Serial5.flush();
      digitalWrite(DebugLED, !Debug);
    }
    
  }
  
  //---------------------Get Data--------------------------------
  //Reads data from the various sensors+
  get_encoder_values();
  get_leg_length();
  int ReadStat = -1;
  uint32_t timeread = micros();
  while(ReadStat==-1 && ((micros()-timeread)<450))
  {
    ReadStat=read_Ard();
  }
  
  //-------------------Run KalmanFilter--------------------------
  predictKF();
  updateKF(ReadStat);
  //----------------------Controller-----------------------------
  //check if control killed:
  if(digitalRead(KILL)== LOW)
  {
    //torque=0;
    solenoid=LOW;
    dead=true;
    
    motorCom(0,0);
    
    int32_t dat = 10;
    for(int i=0; i<100; i++)
    {
      writeBytes(dat);
    }

    //int32_t dat = 123456;
    //writeBytes(dat);
  }
  else
  {
    //run_controller();
    //horzontal_controller();
    //testPos();
  }
  while (dead)
  {
    //motorCom(2,0);
    digitalWrite(DebugLED, Debug);
    digitalWrite(solenoidPin, LOW);
    //motorCom(2,0);
  }

  //comms to motor
  if (l>4)
  {
    motorCom(0,motorPosition);
    //motorCom(2,0);
    float tempPosition = motorCom(3);
    if (-1.5<tempPosition<1.5)
    {
      tempPosition = -tempPosition;
    }
    else
    {
      tempPosition = Position;
    }
    Position = tempPosition;
    l=0;
  }
  else
  {
    l++;
  }

  //toggle solenoid
  digitalWrite(solenoidPin,solenoid);
  
  //send data to PC
  send_pc_data();
  
  // wait for control loop to be finished
  wait_control_loop();
  
}

//---------------------------------------------------------------------------------
//----------------------------------Conteroller------------------------------------
//---------------------------------------------------------------------------------
void testPos()
{
  switch (state)
  {
  case 0:
    motorPosition = -fireAng*(4096/360)*6;
    if ((Position<-fireAng*(PI/180)*0.9))
    {
      state++;
      int32_t dat = 35;
      for(int i=0; i<12; i++)
      {
        writeBytes(dat);
      }
      delay(1000);
    }
    break;
  case 1:
    solenoid = LOW;
    motorPosition = fireAng*(4096/360)*6;
    if ((Position>fireAng*(PI/180)*0.9))
    {
      state=0;
      delay(1000);
    }
    break;
  default:
    dead = true;
    break;
  }
}
void horzontal_controller()
{
  switch (stepCnt)
  {
  case 0:
    fireAng = 10;
    break;
  case 1:
    fireAng = 10;
    break;
  case 2:
    fireAng = 0;
    break;
  case 3:
    fireAng = 0;
    break;
  default:
    fireAng = 0;
    break;
  }
  
  switch (state)
  {
  case 0:
    motorPosition = -fireAng*(4096/360)*6;
    if ((Position<-fireAng*(PI/180)*0.8) && (leg_length < 0.115))
    {
      state++;
      int32_t dat = 35;
      for(int i=0; i<12; i++)
      {
        writeBytes(dat);
      }
    }
    break;
  case 1:
    solenoid = HIGH;
    if (X_bar(0)>=0.14 && leg_length>0.1)// && X_bar(1)>0.5)
    {
      state++;
    }
    break;
  case 2:
    solenoid = LOW;
    motorPosition = fireAng*(4096/360)*6;
    if (leg_length<0.1)
    {
      state=0;
      stepCnt++;
    }
    break;
  default:
    dead = true;
    break;
  }
}

void run_controller()
{
  float firePos = X_bar(0);//+0.007*X_bar(1)-0.00024;
  
  if(leg_length>0.120)
  {
    firePos = X_bar(0)+0.007*X_bar(1)-0.00024;
  }
  else
  {
    firePos = leg_length+0.007*X_bar(1)-0.00024;
  }
  
  if(firePos<=0.2)
  {
    if(solenoid == LOW)
    {
      if (Debug) 
      {
        Serial.println("Fire");
      }
      else
      {
        int32_t dat = 35;
        for(int i=0; i<12; i++)
        {
          writeBytes(dat);
        }
      }
    }
    solenoid=HIGH;
  }
  else if (X_bar(0)>=0.14 && X_bar(1)>0.5)
  {
    solenoid = LOW;
  }
  else
  {
    solenoid = solenoid;
  }
  motorPosition=0;
}

//-----------------------------------------------------------------------------------------
//----------------------------------Auxiliary Functions------------------------------------
//-----------------------------------------------------------------------------------------

void wait_control_loop()
{
  evalTime = previousTime + loopTime;
  while (micros() < evalTime) 
  {
     
  }
  previousTime=micros();
}
void init_params()
{
  //INIT PARAMS
  if(Debug)
  {
    Serial.println("Initialising");
  
    boom_Y_enc.write(0);
    boom_X_enc.write(0);
    Serial.println("Encoder Initialised");
  
    Serial5.flush();
    Serial.println("Ard Initialised");

    Serial3.flush();
    Serial.println("Motor Initialised");
    
    setup_KF();
    Serial.println("KF Initialised");

    //digitalWrite(solenoidPin,LOW);
    //delay(2000);

    adc_range=3745;
    adc_offset=analogRead(leg_length_pin);
    Serial.println("4-20 Initialised");
    
    Serial.println("Initialisation Done");
    
  }
  else
  {
    boom_Y_enc.write(0);
    boom_X_enc.write(0);
    
    Serial5.flush();
    Serial3.flush();
    
    setup_KF();
    
    //digitalWrite(solenoidPin,LOW);
    //delay(2000);
    
    adc_range=3745;
    adc_offset=analogRead(leg_length_pin);
    /*int32_t dat = 1;
    for (int i=0; i<12; i++)
    {
      dat = dat*i;
      writeBytes(dat);
    }*/
  }
}
