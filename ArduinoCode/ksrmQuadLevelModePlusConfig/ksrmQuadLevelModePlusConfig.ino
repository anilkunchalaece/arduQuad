/*
   Author :Kunchala Anil
   Email : anilkunchalaece@gmail.com
   Date : 20 June 2018

   Quadcopter flightcontroller in the Level Mode - This is a continuation from rate mode sketch ksrmQuadRateMode
   which i get a moderate takeoff
   ref - http://ardupilot.org/dev/docs/apmcopter-programming-attitude-control-2.html and ymfc-al code
   angle error (difference between target error and actual angle) is converted in to desired rotation rate followed by
   previous rate pid from ksrmQuadRateMode

   The input pulses from RC receiver are converted to the +/- rate of rotation.
   The pilot input acts as the SetPoint to roll,pitch and yaw rate PID's. Measured input to PID comes from
   gyroscope

   Motor directions for Quad + configuration

                5 (cw)
                  |
                  |
    (ccw) 6 ------|------ (ccw)4
                  |
                  |
                7 (cw)


                            Arduino Pin
       m0 right       - ccw      4
       m1 top         - cw       5
       m2 left        - ccw      6
       m3 bottom      - cw       7

    MPU6050 Connections
       SCL   - A5
       SDA   - A4

    Rx connections
    Enable PCMSK0 Register - PCMSK0 - portB (D8-D13) (PCINT0-PCINT6)
            ArduinoPin     PCINTx
        ch1  - 8        PCINT0 - Roll
        ch2  - 9        PCINT1 - Pitch
        ch3  - 10       PCINT2 - Throttle
        ch4  - 11       PCINT3 - YAW

    Arm and Disarm mechanism
      To Arm    - Throttle Min && Yaw Max
      To DisArm - Throttle Min && Roll Max

*/

// Dont use servo - it is limited to 50Hz and conflict with Software serial
// If you are planning for more serial devices go for arduino Mega
// I use blutooth to get flight parameters - it will be used with hardware serial

#include<PID_v1.h>
#include<Wire.h>
#include<math.h>

//used to send debug info via bluetooth
#define DEBUG

#define DATA_INTERVAL 200 //send debug info every 200 milli seconds
unsigned long btDataStartMillis;

//motor connections
#define m0 4
#define m1 5
#define m2 6
#define m3 7

//receiver connections
#define ch0 8
#define ch1 9
#define ch2 10
#define ch3 11

#define motorMinValue 1150
#define motorMaxValue 1800

//variables used for MPU6050 Angle calculation
const float GYRO_SENSITIVITY_SCALE_FACTOR = 65.5; //for 500 degrees/sec full scale

//MPU6050 I2C address
const int MPU6050_ADDR = 0b1101000;
//MPU6050 Register Addresses
const int PWR_REG_ADDR = 0x6B;
const int GYRO_CONFIG_REG_ADDR = 0x1B;
const int GYRO_CONFIG_REG_VALUE = 0x08;//for 500 degrees / sec
const int ACCR_CONFIG_REG_ADDR = 0x1C;
const int ACCR_CONFIG_REG_VALUE = 0x10; //for +/- 8g
const int ACCR_READ_START_ADDR = 0x3B;

//variables used for angular rate and angle calculations
const float radToDegreeConvert = 180.0 / PI;

int16_t accX, accY, accZ, gyroX, gyroY, gyroZ, tmp; //used to store the data from MPU6050 registers
float rotX, rotY, rotZ; //used to store the angular rotation from gyro
float accAngleX = 0.0, accAngleY = 0.0, accrAngleZ = 0.0;
float compAngleX = 0.0, compAngleY = 0.0;

float pitchAngle = 0.0, rollAngle = 0.0;

float pitchAngleAdjust = 0.0, rollAngleAdjust = 0.0;


unsigned long prevTime = 0, currentTime = 0; //used to calculate the dt for gyro integration
unsigned long loopTimer = 0, now = 0, difference = 0; //used to calculate the loop execution time

//gyro offset variables
float gyroOffsetValX = 0, gyroOffsetValY = 0, gyroOffsetValZ = 0;
float accrOffsetValX = 0, accrOffsetValY = 0;

const int noOfSamplesForOffset = 400;

//variables used in pinChange ISR
volatile boolean armMotors = false; //used to arm and disarm quad
volatile int pwmDuration[4]; //used to store pwm duration
volatile unsigned long pwmStart[4];

//port status
volatile byte prevPortState[] = {0, 0, 0, 0};
volatile byte presentPortState[4];

//pinDeclarations for Rx
const byte rxCh[] = {ch0, ch1, ch2, ch3};
const byte noOfChannels = sizeof(rxCh);

//PID Constants //0.5
const float pitchRatePGain = 1.0;
const float pitchRateIGain = 0.0;
const float pitchRateDGain = 0.0;

const float rollRatePGain = 0.0;
const float rollRateIGain = 0.0;
const float rollRateDGain = 0.0;

const float yawRatePGain = 0.0;
const float yawRateIGain = 0.0;
const float yawRateDGain = 0.0;

int pidMax = 400;

// PID variables
float pidPitchRateIn = 0.0, pidPitchRateOut = 0.0, pidPitchRateSetPoint = 0.0;
float pidRollRateIn = 0.0, pidRollRateOut = 0.0, pidRollRateSetPoint = 0.0;
float pidYawRateIn = 0.0, pidYawRateOut = 0.0, pidYawRateSetPoint = 0.0;

float pitchError = 0.0, rollError = 0.0, yawError = 0.0;
float pitchPrevError = 0.0, rollPrevError = 0.0, yawPrevError = 0.0;
float pitchErrorSum = 0.0, rollErrorSum = 0.0, yawErrorSum = 0.0; //for integral coefficient
float pitchErrorDelta = 0.0 , rollErrorDelta = 0.0 , yawErrorDelta = 0.0; //for differntial coeffcient

//variables to store the motor values calculated using pid and throttle
int m0Value, m1Value, m2Value, m3Value;

//Interrupt Service Routine will fire when for PinChange in PortB
ISR(PCINT0_vect) {
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
  if (pwmDuration[2] < 1250 && pwmDuration[0] > 1600) {
    //Throttle Min and Roll Max
    armMotors = false;
  }
}//end of ISR Fcn


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


void updateMotors() {

  int throttle = pwmDuration[2];
  m0Value = throttle - pidRollRateOut + pidYawRateOut; //right
  m1Value = throttle  - pidPitchRateOut - pidYawRateOut; //top
  m2Value = throttle + pidRollRateOut + pidYawRateOut; //left
  m3Value = throttle + pidPitchRateOut  - pidYawRateOut; //bottom

  //dont send values more than motorMaxValue - ESC may get into trouble
  if (m0Value > motorMaxValue) m0Value = motorMaxValue;
  if (m1Value > motorMaxValue) m1Value = motorMaxValue;
  if (m2Value > motorMaxValue) m2Value = motorMaxValue;
  if (m3Value > motorMaxValue) m3Value = motorMaxValue;


  //dont send values less than motorMinValue - Keep motors running
  if (m0Value < motorMinValue) m0Value = motorMinValue + 20;
  if (m1Value < motorMinValue) m1Value = motorMinValue + 20;
  if (m2Value < motorMinValue) m2Value = motorMinValue + 20;
  if (m3Value < motorMinValue) m3Value = motorMinValue + 20;

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
  }//end of while loop

}//end of updateMotors Fcn


void initializeMotors() {
  pinMode(m0, OUTPUT);
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
}//end of initializeMotors Fcn

/*
   This function will convert Rx inputs into angular rate
   which are used as SetPoints for PID
*/

void setPointUpdate() {
  pidRollRateSetPoint = 0.0;
  pidPitchRateSetPoint = 0.0;
  pidYawRateSetPoint = 0.0;

  if (pwmDuration[0] > 1508) {
    pidRollRateSetPoint = pwmDuration[0] - 1508;
  } else if (pwmDuration[0] < 1492) {
    pidRollRateSetPoint = pwmDuration[0] - 1492;
  }

  rollAngleAdjust = rollAngle * 15; // convert 0 - 30 to 0 -450
  pidRollRateSetPoint = pidRollRateSetPoint - rollAngleAdjust; //calculate angleError
  pidRollRateSetPoint = pidRollRateSetPoint / 3.0 ;// the max roll rate is aprox 164 degrees per second (500/3 = 165 d/s )

  if (pwmDuration[1] > 1508) {
    pidPitchRateSetPoint = pwmDuration[1] - 1508;
  } else if (pwmDuration[1] < 1492) {
    pidPitchRateSetPoint = pwmDuration[1] - 1492;
  }


  pitchAngleAdjust = pitchAngle * 15;
  pidPitchRateSetPoint = pidPitchRateSetPoint - pitchAngleAdjust;
  pidPitchRateSetPoint = pidPitchRateSetPoint / 3.0;


  if (pwmDuration[3] > 1508) {
    pidYawRateSetPoint = pwmDuration[3] - 1508;
  } else if (pwmDuration[3] < 1492) {
    pidYawRateSetPoint = pwmDuration[3] - 1492;
  }

  pidYawRateSetPoint = pidYawRateSetPoint / 3.0;

}//end of SetPointUpdate Fcn


void initPID() {
  //nothing to initialize here
}//end of initPID Fcn

/*
   In this fuction we configure mpu6050 and calculate offsets i.e calibration
*/
void initMPU6050() {
  configureMPU();
  calculateOffsets();
}//end of initMPU6050 Fcn

void calculateOffsets() {
  float gyroTotalValX = 0, gyroTotalValY = 0, gyroTotalValZ = 0;
  float accrTotalValX = 0, accrTotalValY = 0;

  for (int i = 0 ; i < noOfSamplesForOffset ; i++) {
    readMPU();
    delayMicroseconds(20);
    gyroTotalValX = gyroTotalValX + rotX;
    gyroTotalValY = gyroTotalValY + rotY;
    gyroTotalValZ = gyroTotalValZ + rotZ;

    accAngleY = atan(accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * radToDegreeConvert;
    accAngleX = atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * radToDegreeConvert;
    accrTotalValX = accrTotalValX + accAngleX;
    accrTotalValY = accrTotalValY + accAngleY;

    //    Serial.println("..........");
  }//end of for loop
  gyroOffsetValX = gyroTotalValX / noOfSamplesForOffset;
  gyroOffsetValY = gyroTotalValY / noOfSamplesForOffset;
  gyroOffsetValZ = gyroTotalValZ / noOfSamplesForOffset;

  accrOffsetValX = accrTotalValX / noOfSamplesForOffset;
  accrOffsetValY = accrTotalValY / noOfSamplesForOffset;

  //read again to get current values
  readMPU();
  accAngleY = atan(accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * radToDegreeConvert;
  accAngleX = atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * radToDegreeConvert;

  //apply offsets to accr values
  accAngleX = accAngleX - accrOffsetValX;
  accAngleY = accAngleY - accrOffsetValY;

  //set initial values for gyro and complementary filter
  rotX = accAngleX;
  rotY = accAngleY;

  compAngleX = accAngleX;
  compAngleY = accAngleY;


}//end of calculateGyroOffsets Fcn


void readMPU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCR_READ_START_ADDR);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  while (Wire.available() < 14) {
    //wait until data is available
#ifdef DEBUG
    Serial.println(F("MPU6050 waiting"));
#endif
  }
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
  rotZ = gyroZ / GYRO_SENSITIVITY_SCALE_FACTOR;

}//end of readGyroX Fcn

/*
   This function will apply complementary filer to gyro angular rates and
   assign them as PID inputs
*/
void calculateRotationRates() {
  //apply the offset
  rotX = rotX - gyroOffsetValX;
  rotY = rotY - gyroOffsetValY;
  rotZ = rotZ - gyroOffsetValZ;
  //complemenrary filter for gyro readings
  pidPitchRateIn = (pidPitchRateIn * 0.8) + (rotX * 0.2);
  pidRollRateIn = (pidRollRateIn * 0.8) + (rotY * 0.2);
  pidYawRateIn = (pidYawRateIn * 0.8) + (rotZ * 0.2);
}//end of calculateRotationRates Fcn

/*
   This Fcn will calculate the angles using complementary filter
*/

void calculateAngles() {
  //angles from accr
  accAngleY = atan(accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * radToDegreeConvert;
  accAngleX = atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * radToDegreeConvert;

  //apply offsets to accr values
  accAngleX = accAngleX - accrOffsetValX;
  accAngleY = accAngleY - accrOffsetValY;


  //double dt = 0.004 = 1/250

  //The Mighty Complementary filter
  compAngleX = 0.98 * (compAngleX + rotX * 0.004) + 0.02 * accAngleX;
  compAngleY = 0.98 * (compAngleY + rotY * 0.004) + 0.02 * accAngleY;

  // assigning pitch and roll
  pitchAngle = compAngleX;
  rollAngle = compAngleY;

  //   //if imu yawed convert roll to pitch //0.004 = 1/250
  //copied from ymfc-al code
  pitchAngle = pitchAngle - rollAngle * sin(rotZ * 0.004 *  ((3.142 * PI) / 180));
  rollAngle = rollAngle + pitchAngle * sin(rotZ * 0.004 * ((3.142 * PI) / 180));
}//end of calculateAngles Fcn


//get register values from mpu6050 and calculate angles based on complementary filter
void updateAnglesFromMPU() {
  readMPU();
  calculateRotationRates();
  calculateAngles();
}//end of updateAnglesFromMPU Fcn

void configureMPU() {
  //Power Register
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_REG_ADDR);//Access the power register
  Wire.write(0b00000000);//check datasheet
  Wire.endTransmission();

  //Gyro Config
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCR_CONFIG_REG_ADDR);
  Wire.write(ACCR_CONFIG_REG_VALUE);//check data sheet for more info
  Wire.endTransmission();

  //Accr Config
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_CONFIG_REG_ADDR);
  Wire.write(GYRO_CONFIG_REG_VALUE);//check datasheet for more info
  Wire.endTransmission();
}//end of setUpMPU Fcn


void computePID() {
  //input updated in updateAngles Fcn
  //update SetPoint according to throttle
  setPointUpdate();

  //calculate errors
  pitchError = pidPitchRateIn - pidPitchRateSetPoint;
  rollError = pidRollRateIn - pidRollRateSetPoint;
  yawError = pidYawRateIn - pidYawRateSetPoint;

  //calculate sum of errors
  pitchErrorSum = pitchError + pitchPrevError;
  rollErrorSum = rollError + rollPrevError;
  yawErrorSum = yawError + yawPrevError;

  //calculate delta of errors
  pitchErrorDelta = pitchError - pitchPrevError;
  rollErrorDelta = rollError - rollPrevError;
  yawErrorDelta = yawError - yawPrevError;

  //save current error as prev error for next iteration
  pitchPrevError = pitchError;
  rollPrevError = rollError;
  yawPrevError = yawError;

  //PID Calculation
  pidPitchRateOut = (pitchError * pitchRatePGain) + (pitchErrorSum * pitchRateIGain) + (pitchErrorDelta * pitchRateDGain);
  pidRollRateOut = (rollError * rollRatePGain) + (rollErrorSum * rollRateIGain) + (rollErrorDelta * rollRateDGain);
  pidYawRateOut = (yawError * yawRatePGain) + (yawErrorSum * yawRateIGain) + (yawErrorDelta * yawRateDGain);

  if (pidPitchRateOut > pidMax) pidPitchRateOut = pidMax;
  else if (pidPitchRateOut < (pidMax * -1)) pidPitchRateOut = pidMax * -1;

  if (pidRollRateOut > pidMax) pidRollRateOut = pidMax;
  else if (pidRollRateOut < (pidMax * -1)) pidRollRateOut = pidMax * -1;

  if (pidYawRateOut > pidMax) pidYawRateOut = pidMax;
  else if (pidYawRateOut < (pidMax * -1) ) pidYawRateOut = pidMax * -1;

}//end of computePID Fcn

void resetPID() {

}//end of resetPID Fcn

void sendBTOutput() {

  if (millis() - btDataStartMillis > DATA_INTERVAL) {
    btDataStartMillis = millis();
    Serial.print(F("y")); Serial.print(pidYawRateIn); Serial.print(F(">"));
    Serial.print(F("p")); Serial.print(pidPitchRateIn); Serial.print(F(">"));
    Serial.print(F("r")); Serial.print(pidRollRateIn); Serial.print(F(">"));
    Serial.print(F("ps")); Serial.print(pidPitchRateSetPoint); Serial.print(F(">"));
    Serial.print(F("rs")); Serial.print(pidRollRateSetPoint); Serial.print(F(">"));
    Serial.print(F("ys")); Serial.print(pidYawRateSetPoint); Serial.print(F(">"));
    Serial.print(F("po")); Serial.print(pidPitchRateOut); Serial.print(F(">"));
    Serial.print(F("ro")); Serial.print(pidRollRateOut); Serial.print(F(">"));
    Serial.print(F("yo")); Serial.print(pidYawRateOut); Serial.print(F(">"));
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

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW); //used to indicate FC is ready
  initReceiver();
#ifdef DEBUG
  Serial.begin(115200); //HC-05 is using hardware serial, 38400 is HC-05 default baud rate
  btDataStartMillis = millis();
  Serial.println("DEBUG mode ON");
#endif
  initMPU6050();
  initPID();
  initializeMotors();
  //prevTime = millis(); //used to calculate dt for gyro integration
  loopTimer = micros();//used to keep track of loop time
  digitalWrite(13, HIGH); //FC is Ready
}//end of setup Fcn

void loop() {
  updateAnglesFromMPU();
  if (armMotors == true) {
    computePID();
    updateMotors();
  }//end of armMotors
#ifdef DEBUG
  sendBTOutput();
#endif
}//end of loop Fcn
