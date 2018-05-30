/*
 * Author : Kunchala Anil
 * Email : anilkunchalaece@gmail.com 
 * Date : 19 May 2018
 * 
 * This sketch is used to read raw data(accelrometer and gyro) from mpu6050
 * 
 * The Following registers have to be configured / used to get the data from mpu6050
 * 
 *          Register Name              Address (in HEX)       VALUE
 *     Power Management Register         6B                 0b00000000
 *     GYRO Configuration                1B                 0b00000000
 *     ACCELEROMTER Config               1C                 0b00000000
 *     Gyro Readings                   43,44,45,46,47,48    43 X_OUT[15:8] , 44 X_OUT[7:0] , Y , Z
 *     Acceleromter Readings           3B,3C,3D,3E,3F,40    3B ACC_X_OUT[15:8] , 3C ACC_X_OUT[7:0] , Y , Z
 *     
 * Upon power up MPU6050 will go to sleep, we need to set the sleep bit to 0 to put the system in wake mode
 * Gyro/Acceler Config is used to set the Full scale setting for Gyro/Accelerometer. selected Full Scale are 200 degrees / seconds for gyro and 2g for accelerometers 
 * For selected FS Sensitivity scale factor's are 131 for gyro and 16384 for accelrometer.
 * 
 * For more info see 
 *      Data sheet - https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 *      Register Description - https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 * 
 */

#include <Wire.h>

const byte GYRO_SENSITIVITY_SCALE_FACTOR = 131; //for 200 degrees / sec
const long ACCR_SENSITIVITY_SCALE_FACTOR = 16384; // for 2g

const int MPU6050_ADDR = 0b1101000;
const byte PWR_REG_ADDR = 0x6B;
const byte GYRO_CONFIG_REG_ADDR = 0x1B;
const byte ACCR_CONFIG_REG_ADDR = 0x1C;

const byte ACCR_READ_START_ADDR = 0x3B;
const byte GYRO_READ_START_ADDR = 0x43;

int16_t accrX,accrY,accrZ;
float gForceX,gForceY,gForceZ;

int16_t gyroX,gyroY,gyroZ;
float rotX,rotY,rotZ;

float prevRotX=0, prevRotY=0, prevRotZ=0;

int16_t Tmp;

unsigned long prevTime,currentTime;

void setup() {
 Wire.begin();
 Serial.begin(115200);
 setUpMPU();
 prevTime = millis();
}//end of setup

void loop() {
  readMPURegisters();
  printData();
  //delay(100);
}//end of loop


/*
 * this function is used to 
 *        - put the MPU out of sleep mode
 *        - set the GYRO Full Scale Range
 *        - set the Acceleromter Full Scale Range
 */
void setUpMPU(){
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

  //Accr Config
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCR_CONFIG_REG_ADDR);
  Wire.write(0b00000000);
  Wire.endTransmission();
}//end of setUpMPU Fcn

/*
 * we can request total 14 registers from mpu with starting address 0x3B(i.e ACCR_XOUT_H)
 * the following data will be
 * 1. 0x3B - ACCR_X_H   
 * 2. 0x3C - ACCR_X_L 
 * 3. 0x3D - ACCR_Y_H
 * 4. 0x3E - ACCR_Y_L
 * 5. 0x3F - ACCR_Z_H
 * 6. 0x40 - ACCR_Z_L
 * 7. 0x41 - TEMP_H
 * 8. 0x42 - TEMP_L 
 * 9. 0x43 - GYRO_X_H
 * 10. 0x44 - GYRO_X_L
 * 11. 0x45 - GYRO_Y_H
 * 12. 0x46 - GYRO_Y_L
 * 13. 0x47 - GYRO_Z_H
 * 14. 0x48 - GYRO_Z_L
 * 
 *  I shamelessly copied it from example in  https://playground.arduino.cc/Main/MPU-6050  
 */
void readMPURegisters(){
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCR_READ_START_ADDR);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU6050_ADDR,14,true);  // request a total of 14 registers
  accrX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  accrY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accrZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  currentTime = millis();
//  Serial.print("cT");
//  Serial.print(currentTime);
//  Serial.print("pT");
//  Serial.println(prevTime);
//  Serial.println((currentTime-prevTime)/1000.0);
  double dt = (currentTime- prevTime)/1000.0;
  prevTime = currentTime;
  processAccrData();
  processGyroData(dt);
  delay(100);
}//end of readMPURegisters

void processAccrData(){
  gForceX = accrX / ACCR_SENSITIVITY_SCALE_FACTOR;
  gForceY = accrY / ACCR_SENSITIVITY_SCALE_FACTOR;
  gForceZ = accrZ / ACCR_SENSITIVITY_SCALE_FACTOR;
}//end of processAccrData Fcn

void processGyroData(double dt){
//  Serial.print("dt");
//  Serial.println(dt);
  rotX = gyroX / GYRO_SENSITIVITY_SCALE_FACTOR;
  rotX =prevRotX + (rotX * dt);
  prevRotX = rotX;
    
  rotY = gyroY / GYRO_SENSITIVITY_SCALE_FACTOR;
  rotY = prevRotY + rotY * dt;
  prevRotY = rotY;
  
  rotZ = gyroZ / GYRO_SENSITIVITY_SCALE_FACTOR;
  rotZ = prevRotZ + rotZ * dt;
  prevRotZ = rotZ;
   
}//end of processGyroData Fcn

void printData(){
//  Serial.print("Gyro  ");
//  Serial.print("X - ");
  Serial.println(accrX);
//  Serial.print(" Y - ");
//  Serial.print(rotY);
//  Serial.print(" Z - ");
//  Serial.println(rotZ);

//  Serial.print("Accr  ");
//  Serial.print("X - ");
//  Serial.print(gForceX);
//  Serial.print(" Y - ");
//  Serial.print(gForceY);
//  Serial.print(" Z - ");
//  Serial.println(gForceZ);
}//end of printData Fcn
