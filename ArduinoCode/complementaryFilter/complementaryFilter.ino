/*
 * Author : Kunchala Anil
 * Email : anilkunchalaece@gmail.com
 * Date : 31 May 2018
 * MPU 6050 IMU using complementary filter
 * 
 * The Completementary filter magic is given
 *  angle = 0.98(angle + gyrData * dt) + 0.02*accData
 *  
 *  I have no idea how it is derived
 *  
 *  Accelrometer angle can be calculated using 
 *  
 *  float pitch = atan(accX/sqrt(pow(accY,2) + pow(accZ,2)));
 *  float roll = atan(accY/sqrt(pow(accX,2) + pow(accZ,2)));
 *  
 * 
 */

 #include <Wire.h>
 #include <math.h>
 
const float GYRO_SENSITIVITY_SCALE_FACTOR = 131.0; //for 200 degrees / sec
const long ACCR_SENSITIVITY_SCALE_FACTOR = 16384.0; // for 2g

const int MPU6050_ADDR = 0b1101000;
const byte PWR_REG_ADDR = 0x6B;
const byte GYRO_CONFIG_REG_ADDR = 0x1B;
//const byte GYRO_READ_START_ADDR = 0x43;
const byte ACCR_CONFIG_REG_ADDR = 0x1C;
const byte ACCR_READ_START_ADDR = 0x3B;

int16_t accX,accY,accZ,gyroX,gyroY,gyroZ,tmp;
double rotX,rotY; //for Gyro's
double angleX,angleY;//for Accer
double compAngleX,compAngleY;//for complimentary filter

unsigned long prevTime=0,currentTime;

//Gyro offset variables
float gyroOffsetValX=0,gyroOffsetValY=0;
float accrOffsetValX=0,accrOffsetValY = 0;

const int noOfSamplesForOffset = 400;

const float radToDegreeConvert = 180.0/PI;

void setup() {
Wire.begin();
Serial.begin(115200); 
configureMPU();
calculateOffsets();
prevTime = millis();
}//end of setUp

void loop() {
  readMPU();
  calculateAngles();
  printValues();
}//end of loop

void configureMPU(){
  //Power Register
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_REG_ADDR);//Access the power register
  Wire.write(0b00000000);
  Wire.endTransmission();

  //Gyro Config
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCR_CONFIG_REG_ADDR);
  Wire.write(0b00000000);
  Wire.endTransmission();

  //Accr Config
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCR_CONFIG_REG_ADDR);
  Wire.write(0b00000000);
  Wire.endTransmission();
}//end of setUpMPU Fcn

void readMPU(){
//  Serial.println("begin tx");
   Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCR_READ_START_ADDR);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
//  Serial.println("request from");
  Wire.requestFrom(MPU6050_ADDR,14,true);
//  Serial.println("request completed");
  accX=Wire.read()<<8|Wire.read(); 
  accY=Wire.read()<<8|Wire.read(); 
  accZ=Wire.read()<<8|Wire.read(); 
  tmp = Wire.read()<<8|Wire.read();
  gyroX = Wire.read()<<8|Wire.read();
  gyroY = Wire.read()<<8|Wire.read();
  gyroZ = Wire.read()<<8|Wire.read();

  //apply scale factor for gyro reading
  rotX = gyroX / GYRO_SENSITIVITY_SCALE_FACTOR;
  rotY = gyroY / GYRO_SENSITIVITY_SCALE_FACTOR;

//  Serial.println("read mpu completed ");

}//end of readGyroX Fcn

/*
 * This Fcn will calculate offset and set initial value of gyro 
 * complementary values
 */

void calculateOffsets(){
  float gyroTotalValX = 0, gyroTotalValY = 0;
  float accrTotalValX = 0, accrTotalValY = 0;
  for (int i=0 ; i < noOfSamplesForOffset ; i++){
    readMPU();
    gyroTotalValX = gyroTotalValX + rotX;
    gyroTotalValY = gyroTotalValY + rotY;
    //angles from accr
    angleX = atan(accX/sqrt(pow(accY,2) + pow(accZ,2))) * radToDegreeConvert;
    angleY = atan(accY/sqrt(pow(accX,2) + pow(accZ,2))) * radToDegreeConvert;
    accrTotalValX = accrTotalValX + angleX;
    accrTotalValY = accrTotalValY + angleY;
    Serial.println("..........");
  }//end of for loop
  gyroOffsetValX = gyroTotalValX / noOfSamplesForOffset;
  gyroOffsetValY = gyroTotalValY / noOfSamplesForOffset;

  accrOffsetValX = accrTotalValX / noOfSamplesForOffset;
  accrOffsetValY = accrTotalValY / noOfSamplesForOffset;

  Serial.print("Gyro Offset X Y");
  Serial.print(gyroOffsetValX);
  Serial.print(" ");
  Serial.println(gyroOffsetValY);

  Serial.print("Accr Offset X Y");
  Serial.print(accrOffsetValX);
  Serial.print(" ");
  Serial.println(accrOffsetValY);
  
  //read again to get current values
  readMPU();
  angleX = atan(accX/sqrt(pow(accY,2) + pow(accZ,2))) * radToDegreeConvert;
  angleY = atan(accY/sqrt(pow(accX,2) + pow(accZ,2))) * radToDegreeConvert;

  //apply offsets to accr values
  angleX = angleX - accrOffsetValX;
  angleY = angleY - accrOffsetValY;

  //set initial values for gyro and complementary filter
  rotX = angleX;
  rotY = angleY;
  compAngleX = angleX;
  compAngleY = angleY;
  
}//end of calculateGyroOffsets Fcn

/*
 * This Fcn will calculate the angles using complementary filter
 */

void calculateAngles(){
  //angles from accr
  angleX = atan(accX/sqrt(pow(accY,2) + pow(accZ,2)))*radToDegreeConvert;
  angleY = atan(accY/sqrt(pow(accX,2) + pow(accZ,2)))*radToDegreeConvert;

  //apply offsets to accr values
  angleX = angleX - accrOffsetValX;
  angleY = angleY - accrOffsetValY;
  
  currentTime = millis();
  double dt = (currentTime - prevTime)/1000;
  prevTime = currentTime;
  //The Mighty Complementary filter
   compAngleX = 0.99 * (compAngleX + (rotX - gyroOffsetValX) * dt) + 0.01 * angleX; 
   compAngleY = 0.99 * (compAngleY + (rotY - gyroOffsetValY) * dt) + 0.01 * angleY;
}//end of calculateAngles Fcn

void printValues(){
  Serial.print("X ");
  Serial.print(compAngleX);
  Serial.print(" Y ");
  Serial.println(compAngleY);
}//end of printValues Fcn

