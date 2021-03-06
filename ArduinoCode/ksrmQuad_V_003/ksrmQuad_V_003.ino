/*
   Author : Kunchala Anil
   Email : anilkunchalaece@gmail.com
   Date : 06 June 2018

   This is a rewrite of ksrmQuad_V_002 Sketch with PID Implementation with fixed time period
      reason - using servo library work at 50Hz to drive ESC - this is quite low
               we can increase this up to 250HZ. esc needs pulse every 4ms


     Motor Directions for Quad + Configuration

                            Arduino Pin
             m0 Front - CW      4
             m1 Right - CCW     5
             m2 Back  - CW      6
             m3 Left  - CCW     7

     MPU 6050 Connections
             SCL  - A5
             SDA  - A4
             INT  -  2 -- we are not gonna use this

      Rx Connections
      Enable PCMSK0 Register  - PCMSK0 - portB (D8-D13) (PCINT0  - PCINT6)
                    Arduino Pin   PCINTx
            ch1   -    8          PCINT0   - Roll
            ch2   -    9          PCINT1   - Pitch
            ch3   -    10         PCINT2   - Throttle
            ch4   -    11         PCINT3   - Yaw


     Arm - DisArm Mechanisms
      To Arm    - Throttle Min && Yaw Max
      To DisArm - Throttle Min && Roll Max

*/

#include<Servo.h> //used to drive bldc motors
// There may be some conflicts with Software Serial Library and Servo
// So if you are planning for more serial devices go for Arduino Mega
// I use bluetooth to get flight parameters - it will be used with Hardware Serial

#include<Wire.h> //used for MPU6050
#include<math.h> //used in accr angle calculation check accrTest for more info

//#define TUNING // used for tuning purpose

//tuning variables
#ifdef TUNING
#define MAX_DATA_LENGTH 100
char startingChar = '<';
char endingChar = '>';
boolean storeRecvData = false;
char recvDataBuffer[MAX_DATA_LENGTH];
char strtokDelimiter[] = ","; //used to parse the data
int index = 0;
double pp, pd, pi, rp, rd, ri, yp, yd, yi; //pid constants for pitch,roll,yaw [double since library only supports double]
#endif

//used to send bluetooth debug info
#define DATA_INTERVAL 150
unsigned long btDataStartMillis = millis();

//MotorConnections
#define m0 4
#define m1 5
#define m2 6
#define m3 7

// Date : 20Mar - Changing the Motor value from Angle(i.e servo.write) to microSeconds(servo.writeMicroSeconds)
#define motorArmValue 1200 // 60
#define motorMinValue 1200 //65
#define motorMaxValue 1800 //130
#define motorArmDelay 2000 //wait after motors armed

#define throttleMinValue 1200
#define throttleMaxValue 1800

//PID Constants


#define pitchPVal 2.7
#define pitchDVal 120
#define pitchIVal 0.0

#define rollPVal 2.7
#define rollDVal 120
#define rollIVal 0

#define yawPVal 0
#define yawDVal 0
#define yawIVal 0

#define pidOutputMax 400 //used to remove integral windup - dont know how to choose this value

//Flight Parameters
#define pitchMin -30
#define pitchMax 30
#define rollMin -30
#define rollMax 30
#define yawMin -90
#define yawMax 90

#define ch0Max 1800
#define ch0Min 1200
#define ch1Max 1800
#define ch1Min 1200

#define chMid 1500


//variables used for MPU6050 angle calculation
const float GYRO_SENSITIVITY_SCALE_FACTOR = 131.0; //for 200 degrees / sec
const long ACCR_SENSITIVITY_SCALE_FACTOR = 16384.0; // for 2g

//MPU6050 i2c address
const int MPU6050_ADDR = 0b1101000;
//MPU6050 Register Addresses
const byte PWR_REG_ADDR = 0x6B;
const byte GYRO_CONFIG_REG_ADDR = 0x1B;
//const byte GYRO_READ_START_ADDR = 0x43;
const byte ACCR_CONFIG_REG_ADDR = 0x1C;
const byte ACCR_READ_START_ADDR = 0x3B;

const float radToDegreeConvert = 180.0 / PI;

int16_t accX, accY, accZ, gyroX, gyroY, gyroZ, tmp; //used to store the data from MPU registers
double rotX, rotY; //for Gyro's
double angleX, angleY; //for Accer
double compAngleX, compAngleY; //for complimentary filter
double prevCompAngleX, prevCompAngleY; //used to remove small(+- 0.04) variance error

unsigned long prevTime = 0, currentTime=0,loopTimer=0; //to calculate dt for gyro integration
unsigned long now, difference;

//offset variables
float gyroOffsetValX = 0, gyroOffsetValY = 0;
float accrOffsetValX = 0, accrOffsetValY = 0;

const int noOfSamplesForOffset = 400;

//variables used in pinChange ISR
volatile boolean recvPCInt = false;
volatile boolean armMotors = false;
boolean motorsArmed = false;

//servo instansces for motors
Servo motor0;
Servo motor1;
Servo motor2;
Servo motor3;

// PID Variables
float pidPitchIn, pidPitchOut, pidPitchSetPoint = 0; 
float pidRollIn, pidRollOut, pidRollSetPoint = 0;
float pidYawIn, pidYawOut, pidYawSetPoint = 0;

float pitchError,rollError,yawError = 0;
float pitchPrevError,rollPrevError,yawPrevError=0;
float pitchErrorSum,rollErrorSum,yawErrorSum = 0;//for Integral coefficient
float pitchErrorDelta,rollErrorDelta,yawErrorDelta = 0;//for Derivative coefficient


//variables to store the motor values calculated using pid and throttle
int m0Value, m1Value, m2Value, m3Value;

//used to store the pwm duration
volatile unsigned long pwmDuration[4];
volatile unsigned long pwmStart[4];

//pinDeclaration for Rx
const byte rxCh[] = {8, 9, 10, 11};
const byte noOfChannels = sizeof(rxCh);

//portStatus
volatile byte prevPortState[] = {0, 0, 0, 0};
volatile byte presentPortState[4];

//Interrupt Service Routine will fire when for PinChange in PortB
ISR(PCINT0_vect) {
  recvPCInt = true;
  for (int ch = 0; ch < noOfChannels ; ch++) {
    presentPortState[ch] = digitalRead(rxCh[ch]);
  }//end of for looptr

  for (int c = 0; c < noOfChannels ; c++) {
    if (prevPortState[c] == 0 & presentPortState[c] == 1) {
      //if previous state is 0 and present state is 1 (Raising Edge) then take the time stamp
      pwmStart[c] = micros();
      prevPortState[c] = 1; //update the prevPort State
    } else if (prevPortState[c] == 1 & presentPortState[c] == 0) {
      //if previous state is 1 and present state is 0 (Falling Edge) then calculate the width based on the change
      pwmDuration[c] = micros() - pwmStart[c];
      prevPortState[c] = 0; //update Present PortState
    }
  }//end of for loop

  //ARM motors
  if (pwmDuration[2] < 1250 && pwmDuration[3] > 1700) {
    //Throttle Min And Yaw Max
    armMotors = true;
  }

  //DISARM Motors
  if (pwmDuration[2] < 1250 && pwmDuration[0] > 1700) {
    //Throttle Min and Roll Max
    armMotors = false;
  }
}//end of ISR Fcn



void updateMotors() {

  int throttle = map(pwmDuration[2], throttleMinValue, throttleMaxValue, motorMinValue, motorMaxValue);
  m0Value = throttle - pidPitchOut; //- pidYawOut;
  m1Value = throttle - pidRollOut; //+ pidYawOut;
  m2Value = throttle + pidPitchOut; //- pidYawOut;
  m3Value = throttle + pidRollOut; //+ pidYawOut;

//  //keep the motors running
//  if (m0Value < motorArmValue) m0Value = motorArmValue + 50;
//  if (m1Value < motorArmValue) m1Value = motorArmValue + 50;
//  if (m2Value < motorArmValue) m2Value = motorArmValue + 50;
//  if (m3Value < motorArmValue) m3Value = motorArmValue + 50;

  //dont send values more than motorMaxValue - ESC may get into trouble
  if (m0Value > motorMaxValue) m0Value = motorMaxValue;
  if (m1Value > motorMaxValue) m1Value = motorMaxValue;
  if (m2Value > motorMaxValue) m2Value = motorMaxValue;
  if (m3Value > motorMaxValue) m3Value = motorMaxValue;

//  while(micros() - loopTimer < 4000) {
//    //wait until 4000 microseconds passed
//    //i.e send ESC pulse every 4000 micro seconds
//  }
//  //update the loopTimer
//  loopTimer = micros();
//  
//  setAllMotorOutputsToHigh();
//    //wait until all the outputs are low
//  while(digitalRead(m0) || digitalRead(m1) || digitalRead(m2) || digitalRead(m3)){
//     unsigned long currentTime = micros();
//    if(m0Value <= currentTime )  digitalWrite(m0,LOW);//if time is complete lower the pulse
//    if(m1Value <= currentTime ) digitalWrite(m1,LOW);
//    if(m2Value <= currentTime ) digitalWrite(m2,LOW);
//    if(m3Value <= currentTime ) digitalWrite(m3,LOW);
//  }//end of while

    // Refresh rate is 250Hz: send ESC pulses every 4000µs
    while ((now = micros()) - loopTimer < 4000);

    // Update loop timer
    loopTimer = now;

    // Set pins #4 #5 #6 #7 HIGH
    PORTD |= B11110000;

    // Wait until all pins #4 #5 #6 #7 are LOW
    while (PORTD >= 16) {
        now        = micros();
        difference = now - loopTimer;

        if (difference >= m0Value) PORTD &= B11101111; // Set pin #4 LOW
        if (difference >= m1Value) PORTD &= B11011111; // Set pin #5 LOW
        if (difference >= m2Value) PORTD &= B10111111; // Set pin #6 LOW
        if (difference >= m3Value) PORTD &= B01111111; // Set pin #7 LOW
}

}//end of updateMotors Fcn


void initializeMotors() {
    pinMode(m0,OUTPUT);
    pinMode(m1,OUTPUT);
    pinMode(m2,OUTPUT);
    pinMode(m3,OUTPUT);
}//end of initializeMotors Fcn

//this function is used as part of arming, disarming and updating motors
void setAllMotorOutputsToHigh(){
  digitalWrite(m0,HIGH);
  digitalWrite(m1,HIGH);
  digitalWrite(m2,HIGH);
  digitalWrite(m3,HIGH);
}//end of setAllMotorsOutputsToHigh Fcn


void armAllMotors() {
  setAllMotorOutputsToHigh();
  //wait until all the outputs are low
  while(digitalRead(m0) || digitalRead(m1) || digitalRead(m2) || digitalRead(m3)){
    unsigned long currentTime = micros();
    if(motorArmValue <= currentTime ){
      digitalWrite(m0,LOW);
      digitalWrite(m1,LOW);
      digitalWrite(m2,LOW);
      digitalWrite(m3,LOW);
    }//end of IF
  }//end of while
 
 delay(motorArmDelay);
 Serial.println(F("Motors Armed"));
 }//end of armAllMotors Fcn


void disArmMotors() {
  setAllMotorOutputsToHigh();
  //wait until all the outputs are low
  while(digitalRead(m0) || digitalRead(m1) || digitalRead(m2) || digitalRead(m3)){
    //Serial.println("im struck");
    unsigned long currentTime = micros();
    if((motorArmValue - 100) <= currentTime ){
      digitalWrite(m0,LOW);
      digitalWrite(m1,LOW);
      digitalWrite(m2,LOW);
      digitalWrite(m3,LOW);
    }//end of IF
  }//end of while
 Serial.println("motors are disarmed");
//  Serial.println(F("motors are disarmed "));
}//end of disArmMotors Fcn

//get register values from mpu6050 and calculate angles based on complementary filter
void updateAnglesFromMPU() {
  readMPU();
  calculateAngles();
}//end of updateAnglesFromMPU Fcn


void setPointUpdate() {
  if (pwmDuration[0] > chMid - 20  && pwmDuration[0] < chMid + 20) {
    pidRollSetPoint = 0;
  } else {
    //ref : https://arduino.stackexchange.com/questions/9219/why-is-the-constrain-function-used-after-the-map-function
    pidRollSetPoint = map(pwmDuration[0], ch0Min, ch0Max, rollMin, rollMax);
    pidRollSetPoint = constrain(pidRollSetPoint, rollMin, rollMax);
  }
  if (pwmDuration[1] > chMid - 20 && pwmDuration[1] < chMid + 20) {
    pidPitchSetPoint = 0;
  } else {
    pidPitchSetPoint = map(pwmDuration[1], ch1Min, ch1Max, pitchMin, pitchMax);
    pidPitchSetPoint = constrain(pidPitchSetPoint, pitchMin, pitchMax);
  }
}//end of setPointUpdate Fcn


void computePID() {
  
  //update setpoint according to throttle
  setPointUpdate();
    
  //calculate errors
  pitchError = compAngleY - pidPitchSetPoint;
  rollError = compAngleX - pidRollSetPoint;

  //calculate sum of errors
  pitchErrorSum = pitchError + pitchPrevError;
  rollErrorSum = rollError + rollPrevError;

  //calculate delta of errors
  pitchErrorDelta = pitchError - pitchPrevError;
  rollErrorDelta = rollError - rollPrevError;

  //save current error as prevError for Next iteration
  pitchPrevError = pitchError;
  rollPrevError = rollError;


 //PID Calculations
 pidPitchOut = (pitchError * pitchPVal) + (pitchErrorSum * pitchIVal) + (pitchErrorDelta * pitchDVal);
 pidRollOut = (rollError * rollPVal) + (rollErrorSum * rollIVal) + (rollErrorDelta * rollDVal); 

//constrining the pid output 
 if ( pidPitchOut > pidOutputMax){
    pidPitchOut = pidOutputMax;
 }else if (pidPitchOut < -pidOutputMax){
    pidPitchOut = -pidOutputMax;
 }//end of ifElse

if(pidRollOut > pidOutputMax){
    pidRollOut = pidOutputMax;
}else if(pidRollOut < -pidOutputMax){
  pidRollOut = -pidOutputMax;
}//end of ifElse

}//end of computePID Fcn



#ifdef TUNING
void processReceivedData() {
  char * strtokIndex ; //this is used by strtok() as index
  strtokIndex = strtok(recvDataBuffer, strtokDelimiter); // get the first part - pp
  pp = atof(strtokIndex); // convert to float and store
  strtokIndex = strtok(NULL, strtokDelimiter); // this continues where the previous call left off - get pd
  pd = atof(strtokIndex);
  strtokIndex = strtok(NULL, strtokDelimiter); // get pi
  pi = atof(strtokIndex);
  strtokIndex = strtok(NULL, strtokDelimiter); // get rp
  rp = atof(strtokIndex);
  strtokIndex = strtok(NULL, strtokDelimiter); // get rd
  rd = atof(strtokIndex);
  strtokIndex = strtok(NULL, strtokDelimiter); // get ri
  ri = atof(strtokIndex);
  strtokIndex = strtok(NULL, strtokDelimiter); // get yp
  yp = atof(strtokIndex);
  strtokIndex = strtok(NULL, strtokDelimiter); // get yd
  yd = atof(strtokIndex);
  strtokIndex = strtok(NULL, strtokDelimiter); // get yi
  yi = atof(strtokIndex);
//here there is no auto tune method - we can use these to send individual pid calculations
  Serial.print(F("@pp")); Serial.print(pp); Serial.print(F(">"));
  Serial.print(F("@pi")); Serial.print(pi); Serial.print(F(">"));
  Serial.print(F("@pd")); Serial.print(pd); Serial.print(F(">"));
  Serial.print(F("@rp")); Serial.print(rp); Serial.print(F(">"));
  Serial.print(F("@ri")); Serial.print(ri); Serial.print(F(">"));
  Serial.print(F("@rd")); Serial.print(rd); Serial.print(F(">"));
  Serial.print(F("@yp")); Serial.print(yp); Serial.print(F(">"));
  Serial.print(F("@yi")); Serial.print(yi); Serial.print(F(">"));
  Serial.print(F("@yd")); Serial.print(yd); Serial.print(F(">"));
  Serial.println("");
  if (pp > 0 && pd > 0)  
  {
    setModifiedTunings();//add tuning parameters to PID if all the pitch pid parameters are positive
    Serial.println("modified tunings applied");
  }
}//end of processReceivedData

void setModifiedTunings() {
  pitchController.SetTunings(pp, pi, pd);
  rollController.SetTunings(rp, ri, rd);
  //  yawController.SetTunings(yp, yi, yd);
}//end of setModifiedTunings


void checkForBTInput() {
  if (Serial.available()) {
    //Serial.write(BTSerial.read());
    char recvChar = Serial.read();
    if (recvChar == startingChar) {
      storeRecvData = true;
      index = 0; //set the index back to starting
    }//end of if

    if (recvChar == endingChar) {
      storeRecvData = false;
      recvDataBuffer[index] = 0; //null terminating the string
      //Serial.println(recvDataBuffer);
      processReceivedData();
    }//end of if

    if (storeRecvData == true) {
      if (recvChar != startingChar) {
        recvDataBuffer[index] = recvChar; //store the received char in buffer
        index = index + 1; //increment the index
      }//end of if. Dumb Workaroung :)
    }

  }//end of If
}//end of checkForBTInput
#endif //of define TUNING



void sendBTOutput() {
  if (millis() - btDataStartMillis > DATA_INTERVAL) {
    btDataStartMillis = millis();
    //    Serial.print(F("y")); Serial.print(pidYawIn);Serial.print(F(">"));
    Serial.print(F("p")); Serial.print(compAngleY); Serial.print(F(">")); //pidPitchIn
    Serial.print(F("r")); Serial.print(compAngleX); Serial.print(F(">")); //PidRollIn
    Serial.print(F("ps")); Serial.print(pidPitchSetPoint); Serial.print(F(">"));
    Serial.print(F("rs")); Serial.print(pidRollSetPoint); Serial.print(F(">"));
    Serial.print(F("ys")); Serial.print(pidYawSetPoint); Serial.print(F(">"));
    Serial.print(F("po")); Serial.print(pidPitchOut); Serial.print(F(">"));
    Serial.print(F("ro")); Serial.print(pidRollOut); Serial.print(F(">"));
    //    Serial.print(F("yo")); Serial.print(pidYawOut);Serial.print(F(">"));
    Serial.print(F("m0-")); Serial.print(m0Value); Serial.print(F(">"));
    Serial.print(F("m1-")); Serial.print(m1Value); Serial.print(F(">"));
    Serial.print(F("m2-")); Serial.print(m2Value); Serial.print(F(">"));
    Serial.print(F("m3-")); Serial.print(m3Value); Serial.print(F(">"));
    Serial.print(F("c0")); Serial.print(pwmDuration[0]); Serial.print(F(">"));
    Serial.print(F("c1")); Serial.print(pwmDuration[1]); Serial.print(F(">"));
    Serial.print(F("c2")); Serial.print(pwmDuration[2]); Serial.print(F(">"));
    Serial.print(F("c3")); Serial.print(pwmDuration[3]); Serial.print(F(">"));
    Serial.println("");
  }//end of IF
}//end of sendBTOutput Fcn


void initReceiver() {
  cli(); //Clear all interrupts
  PCICR |= 1 << PCIE0; //Enable port B Registers i.e D8-D13
  PCMSK0 |= 1 << PCINT3 | 1 << PCINT2 | 1 << PCINT1 | 1 << PCINT0 ; // Pin11,10,9,8
  sei(); //enable all interrupts

  for (int i = 0 ; i < noOfChannels ; i++) {
    pinMode(rxCh[i], INPUT);
    digitalWrite(rxCh[i], HIGH); //enable pullup
  }//end of for loop
}//end of initReceiver Fcn


void configureMPU() {
  //Power Register
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_REG_ADDR);//Access the power register
  Wire.write(0b00000000);//check datasheet
  Wire.endTransmission();

  //Gyro Config
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCR_CONFIG_REG_ADDR);
  Wire.write(0b00000000);//check data sheet for more info
  Wire.endTransmission();

  //Accr Config
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCR_CONFIG_REG_ADDR);
  Wire.write(0b00000000);//check datasheet for more info
  Wire.endTransmission();
}//end of setUpMPU Fcn

void readMPU() {
  //  Serial.println("begin tx");
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCR_READ_START_ADDR);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  //  Serial.println("request from");
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  //  Serial.println("request completed");
  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
  tmp = Wire.read() << 8 | Wire.read();
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();

  //apply scale factor for gyro reading
  rotX = gyroX / GYRO_SENSITIVITY_SCALE_FACTOR;
  rotY = gyroY / GYRO_SENSITIVITY_SCALE_FACTOR;

  //  Serial.println("read mpu completed ");

}//end of readGyroX Fcn

/*
   This Fcn will calculate offset and set initial value of gyro
   complementary values
*/

void calculateOffsets() {
  float gyroTotalValX = 0, gyroTotalValY = 0;
  float accrTotalValX = 0, accrTotalValY = 0;
  for (int i = 0 ; i < noOfSamplesForOffset ; i++) {
    readMPU();
    gyroTotalValX = gyroTotalValX + rotX;
    gyroTotalValY = gyroTotalValY + rotY;
    //angles from accr
    angleX = atan(accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * radToDegreeConvert;
    angleY = atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * radToDegreeConvert;
    accrTotalValX = accrTotalValX + angleX;
    accrTotalValY = accrTotalValY + angleY;
    Serial.println("..........");
  }//end of for loop
  gyroOffsetValX = gyroTotalValX / noOfSamplesForOffset;
  gyroOffsetValY = gyroTotalValY / noOfSamplesForOffset;

  accrOffsetValX = accrTotalValX / noOfSamplesForOffset;
  accrOffsetValY = accrTotalValY / noOfSamplesForOffset;

  //read again to get current values
  readMPU();
  angleX = atan(accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * radToDegreeConvert;
  angleY = atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * radToDegreeConvert;

  //apply offsets to accr values
  angleX = angleX - accrOffsetValX;
  angleY = angleY - accrOffsetValY;

  //set initial values for gyro and complementary filter
  rotX = angleX;
  rotY = angleY;
  compAngleX = angleX;
  compAngleY = angleY;

  //used to remove small variances
  prevCompAngleX = angleX;
  prevCompAngleY = angleY;

}//end of calculateGyroOffsets Fcn

/*
   This Fcn will calculate the angles using complementary filter
*/

void calculateAngles() {
  //angles from accr
  angleX = atan(accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * radToDegreeConvert;
  angleY = atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * radToDegreeConvert;

  //apply offsets to accr values
  angleX = angleX - accrOffsetValX;
  angleY = angleY - accrOffsetValY;

  currentTime = millis();
  double dt = (currentTime - prevTime) / 1000;
  prevTime = currentTime;
  //The Mighty Complementary filter
  compAngleX = 0.99 * (compAngleX + (rotX - gyroOffsetValX) * dt) + 0.01 * angleX;
  compAngleY = 0.99 * (compAngleY + (rotY - gyroOffsetValY) * dt) + 0.01 * angleY;

  //remove small variations
  //iam getting + or - 0.04 angle difference even when sensor is stable to remove it 
  //we are doing this
  //REMOVING VARIANCE FILTER TO CHECK YMFC TUNING
//  if (compAngleX < prevCompAngleX + 0.02 && compAngleX > prevCompAngleX - 0.02) 
//  {
//    compAngleX = prevCompAngleX;
//  }
//
//  if (compAngleY < prevCompAngleY + 0.02 && compAngleY > prevCompAngleY - 0.02)
//  {
//    compAngleY = prevCompAngleY;
//  }

  //store the angle values
  prevCompAngleX = compAngleX;
  prevCompAngleY = compAngleY;
      
}//end of calculateAngles Fcn


/*
   In this fuction we configure mpu6050 and calculate offsets i.e calibration
*/
void initMPU6050() {
  configureMPU();
  calculateOffsets();
}//end of initMPU6050 Fcn

void setup() {
  initReceiver();
  Serial.begin(38400); //HC-05 is using hardware serial , 38400 is HC-05 Default Baud Rate
  initMPU6050();
  initializeMotors();
  prevTime = millis(); //used to calculate dt for gyro integration
  loopTimer = micros() ; // used to calculate the loop timer [do we need it?]
//  pinMode(12,OUTPUT); //to check execution time
//  digitalWrite(12,LOW);
}//end of setup Fcn

void loop() {
//   digitalWrite(12,!digitalRead(12));// to check execution time
  updateAnglesFromMPU(); 
   if (armMotors == true && motorsArmed == false) {
    armAllMotors();
    motorsArmed = true;
  }

  if (armMotors == false) {
    disArmMotors();
    motorsArmed = false;
  }

  if (armMotors == true) {
    computePID();
    updateMotors();
  }//end of armMotors
#ifdef TUNING
  checkForBTInput();
#endif
  sendBTOutput();

}//end of loop Fcn
