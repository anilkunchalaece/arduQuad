/*
 * Author : kunchala Anil
 * Email : anilkunchalaece@gmail.com
 * Date : 22 May 2018
 * 
 * This sketch is used to calculate angle from MPU6050 Accelerometer values
 * crust of the sketch is 
 *        float pitch = atan(accX/sqrt(pow(accY,2) + pow(accZ,2)));
 *         float roll = atan(accY/sqrt(pow(accX,2) + pow(accZ,2)));
 * 
 * Check following https://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf
 * i didnt understand the math behind it
 */
 
#include <Wire.h>
#include <math.h>

const long ACCR_SENSITIVITY_SCALE_FACTOR = 16384.0; // for 2g
const int MPU6050_ADDR = 0b1101000;

const byte PWR_REG_ADDR = 0x6B;

const byte ACCR_CONFIG_REG_ADDR = 0x1C;
const byte ACCR_READ_START_ADDR = 0x3B;

int16_t accX,accY,accZ;
double angleX,angleY; 
double aX,aY,aZ;

void setup() {
 Wire.begin();
 Serial.begin(115200);
 configureMPU();
}//end of setup

void loop() {
  readAccrX();
// apply trigonometry to get the pitch and roll:
float pitch = atan(accX/sqrt(pow(accY,2) + pow(accZ,2)));
float roll = atan(accY/sqrt(pow(accX,2) + pow(accZ,2)));
//convert radians into degrees
pitch = pitch * (180.0/PI);
roll = roll * (180.0/PI);
  

  Serial.print(pitch);
  Serial.print("   ");
  Serial.println(roll);
  delay(100);
}//end of loop

void readAccrX(){
   Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCR_READ_START_ADDR);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR,6,true);  
  accX=Wire.read()<<8|Wire.read(); 
  accY=Wire.read()<<8|Wire.read(); 
  accZ=Wire.read()<<8|Wire.read(); 
//  Serial.println(accX);   
//  aX = accX / ACCR_SENSITIVITY_SCALE_FACTOR;
//  aY = accY / ACCR_SENSITIVITY_SCALE_FACTOR;
//  aZ = accZ / ACCR_SENSITIVITY_SCALE_FACTOR;

}//end of readGyroX Fcn

void configureMPU(){
  //Power Register
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_REG_ADDR);//Access the power register
  Wire.write(0b00000000);
  Wire.endTransmission();

  //Accr Config
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCR_CONFIG_REG_ADDR);
  Wire.write(0b00000000);
  Wire.endTransmission();
}//end of setUpMPU Fcn

