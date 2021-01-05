#include <Arduino.h>
#include "quaternionFilters.h"
#include "MPU9250.h"
#include <Wire.h>
#include <LIDARLite.h>

//------------------------MPU ---------------------------------------
#define AHRS false         // Set to false for basic data read
#define SerialDebug false  // Set to true to get Serial output for debugging
#define intPin 7
#define SPIspeed 1000000
#define SPIport SPI
#define CSpin   10
#define MPU9250_ADDRESS 9001 // Needs to be here but unused

MPU9250 myIMU(CSpin, SPIport, SPIspeed);

//------------------------Lidar ---------------------------------------
LIDARLite lidar1;
uint8_t dist = 0; 
//------------------------Comms ---------------------------------------
int loopCnt = 0;

int wait = 0;

#define Lp 1
#define Axp 0
#define Azp 1
#define Lw 0
#define Axw 1
#define Azw 2

float Azoff = 0;
uint8_t runCnt = 0;
unsigned long previousTime = 0;
//------------------------Functions ---------------------------------------
void setUpMPU();
void readMPU();
void setUpLidarLite();
int distanceFast(bool biasCorrection, int Addr_read);
void writeBytesFloats(uint8_t sensor, float data, uint8_t par);

//------------------------Setup ---------------------------------------
void setup() {
  //Serial.begin(9600);
  Serial.begin(500000);
  setUpMPU();
  //Serial.println("MPU DONE");
  setUpLidarLite();
  //Serial.println("Lidar Done");
  
  //pinMode(8,OUTPUT);
  //digitalWrite(8, HIGH);
  // put your setup code here, to run once:
  for(int i =0; i<10; i++)
  {
    readMPU();
    Azoff = Azoff + myIMU.az;
  }
  Azoff = Azoff/10;
}

//------------------------Main ---------------------------------------
void loop() {

  if (runCnt < 5)
  {
    readMPU();
    //Serial.print("Ax;");  
    //Serial.println(myIMU.ax);
    writeBytesFloats(Axw, myIMU.ax, Axp);
    //Serial.print("Az;");  
    //Serial.println(myIMU.az);
    //
    float azDat = myIMU.az - Azoff;
    writeBytesFloats(Azw, myIMU.az, Azp);//<---- This was commented out
    //
    runCnt++;
  }
  else
  {
    if (loopCnt<5)
    {
      dist = distanceFast(false, LIDARLITE_ADDR_DEFAULT);
      loopCnt++;
    }
    else
    {
      dist = distanceFast(true, LIDARLITE_ADDR_DEFAULT);
      loopCnt=0;
    }
    //Serial.print("Dist;");
    //Serial.println(dist);
    writeBytesFloats(Lw, dist, Lp);

    runCnt = 0;
  }
  //wait_control_loop(500);
}

//------------------------------SetUpMPU--------------------------------------------
void setUpMPU()
{
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);

  SPIport.begin();
  
  myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250); // Kick the hardware

  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (SerialDebug)
  {
    Serial.print(F("MPU9250 I AM 0x"));
    Serial.print(c, HEX);
    Serial.print(F(" I should be 0x"));
    Serial.println(0x71, HEX);
  }

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);

    if (SerialDebug)
    {
      Serial.println(F("MPU9250 is online..."));
      Serial.print(F("x-axis self test: acceleration trim within : "));
      Serial.print(myIMU.selfTest[0],1); Serial.println("% of factory value");
      Serial.print(F("y-axis self test: acceleration trim within : "));
      Serial.print(myIMU.selfTest[1],1); Serial.println("% of factory value");
      Serial.print(F("z-axis self test: acceleration trim within : "));
      Serial.print(myIMU.selfTest[2],1); Serial.println("% of factory value");
      Serial.print(F("x-axis self test: gyration trim within : "));
      Serial.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
      Serial.print(F("y-axis self test: gyration trim within : "));
      Serial.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
      Serial.print(F("z-axis self test: gyration trim within : "));
      Serial.print(myIMU.selfTest[5],1); Serial.println("% of factory value");
    }

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  
    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    //Serial.println("MPU9250 initialized for active data mode....");

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
  } 
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
}

//-------------------------------ReadMPU--------------------------------------------
void readMPU()
{
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);

    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes - myIMU.accelBias[2];

  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  myIMU.updateTime();

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        // Print acceleration values in milligs!
        Serial.print("X-acceleration: "); Serial.print(1000 * myIMU.ax);
        Serial.print(" mg ");
        Serial.print("Y-acceleration: "); Serial.print(1000 * myIMU.ay);
        Serial.print(" mg ");
        Serial.print("Z-acceleration: "); Serial.print(1000 * myIMU.az);
        Serial.println(" mg ");

        // Print gyro values in degree/sec
        Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
        Serial.println(" degrees/sec");

        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
        //Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
        //Serial.println(" degrees C");
      }

      myIMU.count = millis();
      //digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // update LCD once per half-second independent of read rate
    if (myIMU.delt_t > 5)
    {
      if(SerialDebug)
      {
        Serial.print("ax = ");  Serial.print((int)1000 * myIMU.ax);
        Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
        Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
        Serial.println(" mg");

        Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
        Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
        Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
        Serial.println(" deg/s");
      }
    } // if (myIMU.delt_t > 500)
  } // if (AHRS)
}

//-------------------------------SetUpLidar--------------------------------------------
void setUpLidarLite()
{
  lidar1.begin(0, true); // Set configuration to default and I2C to 400 kHz
  lidar1.write(0x02, 0x0d); // Maximum acquisition count of 0x0d. (default is 0x80)
  lidar1.write(0x04, 0b00000100); // Use non-default reference acquisition count
  lidar1.write(0x12, 0x03);
}

//-------------------------------ReadLidar--------------------------------------------
int distanceFast(bool biasCorrection, int Addr_read)
{
  //If passed True does bias correction  if false reads without bias correction 
  //Should pass true approx every 10 readings

  // Read distance. The approach is to poll the status register until the device goes
  // idle after finishing a measurement, send a new measurement command, then read the
  // previous distance data while it is performing the new command.
  byte isBusy = 1;
  int distance;
  int loopCount;
  
  // Poll busy bit in status register until device is idle
  while(isBusy)
  {
    // Read status register
    Wire.beginTransmission(Addr_read);
    Wire.write(0x01);
    Wire.endTransmission();
    Wire.requestFrom(Addr_read, 1);
    isBusy = Wire.read();
    isBusy = bitRead(isBusy,0); // Take LSB of status register, busy bit

    loopCount++; // Increment loop counter
    // Stop status register polling if stuck in loop
    if(loopCount > 9999)
    {
      break;
    }
  }

  // Send measurement command
  Wire.beginTransmission(Addr_read);
  Wire.write(0X00); // Prepare write to register 0x00
  if(biasCorrection == true)
  {
    Wire.write(0X04); // Perform measurement with receiver bias correction
  }
  else
  {
    Wire.write(0X03); // Perform measurement without receiver bias correction
  }
  Wire.endTransmission();

  // Immediately read previous distance measurement data. This is valid until the next measurement finishes.
  // The I2C transaction finishes before new distance measurement data is acquired.
  // Prepare 2 byte read from registers 0x0f and 0x10
  Wire.beginTransmission(Addr_read);
  Wire.write(0x8f);
  Wire.endTransmission();

  // Perform the read and repack the 2 bytes into 16-bit word
  Wire.requestFrom(Addr_read, 2);
  distance = Wire.read();
  distance <<= 8;
  distance |= Wire.read();

  // Return the measured distance
  return distance;
}

//-------------------------------WriteData--------------------------------------------
void writeBytesFloats(uint8_t sensor, float data, uint8_t par)
{
  byte* dataByte = (byte*) &data;
  byte buffer[8] = {0};

  byte* sensorByte = (byte*) &sensor;

  int count = par;
  for (int i=0; i<4; i++)
  {
    if (((dataByte[i]))&(1))
    {
      count++;
    }
  }
  
  count = count%2;
  byte* parity = (byte*) &count; 
  byte* close = "XX";
  memcpy(buffer, sensorByte, 1);
  memcpy(buffer+1, dataByte, 4);
  memcpy(buffer+5, parity, 1);
  memcpy(buffer+6, close, 2);

  for(byte i=0;i<sizeof(buffer);i++)
  {
    Serial.write(buffer[i]);
  }
}

void wait_control_loop(uint16_t loopTime)
{
  unsigned long evalTime = previousTime + loopTime;
  while (micros() < evalTime) 
  {
     
  }
  previousTime=micros();
}