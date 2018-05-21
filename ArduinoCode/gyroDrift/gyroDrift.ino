/*
 * Author : kunchala Anil
 * Email : anilkunchalaece@gmail.com
 * Date : 21 May 2018
 * 
 * This sketch is used to calculate angle from MPU6050 GyroScope Angular Rate
 * Ingridient of sketch is
 *     Angle = prevAngle + presentAngularRate * dt
 *     prevAngle = Angle
 *     
 *     dt -> time elapsed from the last angular rate reading
 *     
 *     Offset Value is used to slow the Gyro Drift
 *     
 *  For discussion on the code ref : http://forum.arduino.cc/index.php?topic=548675
 */
#include <Wire.h>
const float GYRO_SENSITIVITY_SCALE_FACTOR = 131.0; //for 200 degrees / sec
const int MPU6050_ADDR = 0b1101000;
const byte PWR_REG_ADDR = 0x6B;
const byte GYRO_CONFIG_REG_ADDR = 0x1B;
const byte GYRO_READ_START_ADDR = 0x43;
int16_t gyroX;
float rotX; // It is Angular Rate
float prevRotX=0;
unsigned long prevTime=0,currentTime;

float offsetVal;
const int noOfSamplesForOffset = 1000;

void setup() {
 Wire.begin();
 Serial.begin(115200);
 configureMPU();

  float totalVal = 0;
  for (int i=0; i< noOfSamplesForOffset; i++){
    readGyroX();
    Serial.println(rotX);
    totalVal = totalVal + (rotX);
  }//end of for
  Serial.println(totalVal);
 offsetVal = totalVal / (noOfSamplesForOffset);
 Serial.print("offset Val");
 Serial.println(offsetVal);
 //while(1);
}//end of setup

void loop() {
   
  if(prevTime == 0){
    readGyroX();
    prevTime = millis(); 
  }else{
    readGyroX();
   
    currentTime = millis();
    double dt = (currentTime - prevTime)/1000.0;
    prevTime = currentTime;
    rotX =  prevRotX + (rotX-offsetVal)* dt;
    prevRotX = rotX;
    Serial.println(rotX);
  }//end of ifElse
  delay(100);
}//end of loop

void readGyroX(){
   Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_READ_START_ADDR);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR,2,true);  
  gyroX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  rotX = gyroX / GYRO_SENSITIVITY_SCALE_FACTOR;
}//end of readGyroX Fcn

void configureMPU(){
  //Power Register
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_REG_ADDR);//Access the power register
  Wire.write(0b00000000);
  Wire.endTransmission();

  //Gyro Config
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_CONFIG_REG_ADDR);
  Wire.write(0b00000000);
  Wire.endTransmission();
}//end of setUpMPU Fcn



