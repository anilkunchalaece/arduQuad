/*
  Author : Kunchala Anil
  email : anilkunchalaece@gmail.com
  Date : 15 Mar 2018


  Basic lift with PID of KSRM Quad


*/
/*
   Connect Channel 3 to pin 10 i.e PCINT2
   Arm the all the Motors
   use Servo Write to Send the control signal to ESC

        To Arm ESC use value 60
        Max Speed Can be Achieved using 130

        Pulse Width when Throttle at Minimum position is : 1150
        Pulse width when Throttle at Maximum Posistion is : 1800

        we need to map value from 1200-1800 to 60 to 130

 *    
 *    Motor Directions For Quad + Configuration 
 *    
 *    m0 Front - CW
 *    m1 Right - CCW
 *    m2 Back - CW
 *    m3 Left - CCW

*/


//PCMSKx - Pin Change Mask Register
//PCMSK0 - portB (D8-D13) (PCINT0  - PCINT6)
//PCMSK1 - portC (A0-A5)  (PCINT8  - PCINT14)
//PCMSK2 - portD (D0-D7)  (PCINT16 - PCINT23)
//PCICR -  Pin Change Interrupt Control Register
//  PCIE0 - Pin Change Interrupt Enable 0 - Port B (D8-D13)
//  PCIE1 - Pin Change Interrupt Enable 1 - Port C (A0-A5)
//  PCIE2 - Pin Change Interrupt Enable 2 - Port D (D0-D7)

#include<Servo.h>
#include <PID_v1.h> //PID Library by Brett Beauregard ref:https://github.com/br3ttb/Arduino-PID-Library

//Arduino - MPU6050 Library ref : https://github.com/jarzebski/Arduino-MPU6050+
#include <Wire.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define DEBUG

//MotorConnections
#define m0 4
#define m1 5
#define m2 6
#define m3 7

// Date : 20Mar - Changing the Motor value from Angle(i.e servo.write) to microSeconds(servo.writeMicroSeconds)
#define motorArmValue 1152 // 60
#define motorMinValue 1200 //65
#define motorMaxValue 1800 //130
#define motorArmDelay 2000 //wait after motors armed

#define throttleMinValue 1200
#define throttleMaxValue 1800

//PID Constants
#define pitchPVal 1
#define pitchDVal 0
#define pitchIVal 0

#define rollPVal 0
#define rollDVal 0
#define rollIVal 0

#define yawPVal 0
#define yawDVal 0
#define yawIVal 0


//Flight Parameters
#define pitchMin -90
#define pitchMax 90
#define rollMin -90
#define rollMax 90
#define yawMin -180
#define yawMax 180


//#define pidPitchInfluence 20
//#define pidRollInfluence 20
//#define pidYawInfluence 20


//MPU 6050 Offsets
#define mpuGyroXOffset 98
#define mpuGyroYOffset 20
#define mpuGyroZOffset 34

#define mpuAccelXOffset -1034
#define mpuAccelYOffset -571
#define mpuAccelZOffset 884


volatile boolean recvPCInt = false;

Servo motor0;
Servo motor1;
Servo motor2;
Servo motor3;

//-------------------------------------------------------------------
//MPU variables
//Check example in Library for more details
//-------------------------------------------------------------------

MPU6050 mpu;                           // mpu interface object


uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t devStatus;                     // device status
uint16_t packetSize;                   // estimated packet size
uint16_t fifoCount;                    // fifo buffer size
uint8_t fifoBuffer[64];                // fifo buffer

Quaternion q;                          // quaternion for mpu output
VectorFloat gravity;                   // gravity vector for ypr
float ypr[3] = {0.0f, 0.0f, 0.0f};     // yaw pitch roll values
//float yprLast[3] = {0.0f, 0.0f, 0.0f};

volatile bool mpuInterrupt = false;    //interrupt flag

//-------------------------------------------------------------------
// PID Variables
//-------------------------------------------------------------------
double pidPitchIn, pidPitchOut, pidPitchSetPoint = 0; //though in UNO there is no diff between float and double - PID library throwing error if type is float
double pidRollIn, pidRollOut, pidRollSetPoint = 0;
double pidYawIn, pidYawOut, pidYawSetPoint = 0;


//-------------------------------------------------------------------
//PID Instances
//-------------------------------------------------------------------
PID pitchController(&pidPitchIn, &pidPitchOut, &pidPitchSetPoint, pitchPVal, pitchIVal, pitchDVal, REVERSE); //what is that reverse and forward ?
PID rollController(&pidRollIn, &pidRollOut, &pidRollSetPoint, rollPVal, rollIVal, rollDVal, REVERSE);
PID yawController(&pidYawIn, &pidYawOut, &pidYawSetPoint, yawPVal, yawIVal, yawDVal, DIRECT);


//used to store the pwm duration
volatile unsigned long pwmDuration[4];
volatile unsigned long pwmStart[4];
unsigned long pwmEnd[4];

//pinDeclaration for Rx
const byte rxCh[] = {8, 9, 10, 11};
const byte noOfChannels = sizeof(rxCh);

//portStatus
volatile int prevPortState[] = {0, 0, 0, 0};
volatile int presentPortState[4];
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
  //portValue = PINB & 0x0f;//we are only intrested in first four bits
}

void updateMotors() {

  int width = pwmDuration[2];

  if (width < 1200 ) {
#ifdef DEBUG
    Serial.print("received signal width");
    Serial.println(width);
    Serial.println("not updating motors");
#endif
    motor0.write(motorArmValue);
    motor1.write(motorArmValue);
    motor2.write(motorArmValue);
    motor3.write(motorArmValue);


  }
  else
  {
    int throttle = map(width, throttleMinValue, throttleMaxValue, motorMinValue, motorMaxValue);
    #ifdef DEBUG
    Serial.print("received val is ");
    Serial.println(throttle);
    #endif
     int m0Value = throttle - pidPitchOut; //- pidYawOut;
     int m1Value = throttle - pidRollOut; //+ pidYawOut;
     int m2Value = throttle + pidPitchOut; //- pidYawOut;
     int m3Value = throttle + pidRollOut; //+ pidYawOut;
//#ifdef DEBUG
//    Serial.print("fl,fr,bl,br,po,ro\t");
    Serial.print("<");
    Serial.print(m0Value);
    Serial.print(",");
    Serial.print(m1Value);
    Serial.print(",");
    Serial.print(m2Value);
    Serial.print(",");
    Serial.print(m3Value);
    Serial.print(",");
    Serial.print(pidYawIn);
    Serial.print(",");
    Serial.print(pidPitchIn);
    Serial.print(",");
    Serial.print(pidRollIn);
    Serial.print(",");
    Serial.print(pidPitchOut);
    Serial.print(",");
    Serial.print(pidRollOut);
    Serial.println(">");
//#endif
    motor0.write(m0Value);
    motor1.write(m1Value);
    motor2.write(m2Value);
    motor3.write(m3Value);
#ifdef DEBUG
    Serial.print("motors are updated with value => ");
    Serial.println(throttle);
#endif

  }//end of ifElse
}//end of update motors


void initializeMotors() {
  motor0.attach(m0);
  motor1.attach(m1);
  motor2.attach(m2);
  motor3.attach(m3);
#ifdef DEBUG
  Serial.println("motors initialized");
#endif
}//end of initializeMotorsFunction


void armAllMotors() {
  motor0.write(motorArmValue);
  motor1.write(motorArmValue);
  motor2.write(motorArmValue);
  motor3.write(motorArmValue);
  delay(motorArmDelay);
#ifdef DEBUG
  Serial.println("motors are Armed ");
#endif
}//end of armAllMotors function

//ISR for MPU 6050
void dmpDataReady() {
  mpuInterrupt = true;
}//end of dmpDataReady

void initMPU() {
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  //MPU Offsets
  mpu.setXGyroOffset(mpuGyroXOffset);
  mpu.setYGyroOffset(mpuGyroYOffset);
  mpu.setZGyroOffset(mpuGyroZOffset);
  mpu.setXAccelOffset(mpuAccelXOffset);
  mpu.setYAccelOffset(mpuAccelYOffset);
  mpu.setZAccelOffset(mpuAccelZOffset);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
  }//end of If

#ifdef DEBUG
  Serial.println("mpu initialization completed");
#endif
}//end of initMPU function


void updateAnglesFromMPU() {
  //Check example from MPU6050 Library
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {

    mpu.resetFIFO();

  } else if (mpuIntStatus & 0x02) {

    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  }

}//end of getAnglesFromMPU

void initPID() {
  rollController.SetOutputLimits(rollMin, rollMax);
  pitchController.SetOutputLimits(pitchMin, pitchMax);
  yawController.SetOutputLimits(yawMin, yawMax);
  rollController.SetMode(AUTOMATIC);
  pitchController.SetMode(AUTOMATIC);
  yawController.SetMode(AUTOMATIC);
//  rollController.SetSampleTime(10);
//  pitchController.SetSampleTime(10);
//  yawController.SetSampleTime(10);

#ifdef DEBUG
  Serial.println("pid initialisation completed");
#endif
}//end of initPID

void computePID() {
  pidYawIn = ypr[0]*180/M_PI; //Converts Radians to degrees ref - https://forum.arduino.cc/index.php?topic=446713.msg3078076#msg3078076
  pidPitchIn = ypr[2]*180/M_PI; //Changed 1 to 2 Due to some problem in Orientation or sensor ? NEED TO FIX IT
  pidRollIn = ypr[1]*180/M_PI;
  
  #ifdef DEBUG
    Serial.print("y p r ");
    Serial.print(pidYawIn);
    Serial.print("\t");
    Serial.print(pidPitchIn);
    Serial.print("\t");
    Serial.println(pidRollIn);
  #endif
  rollController.Compute();
  pitchController.Compute();
  yawController.Compute();
}//end of computePID

void setup() {
  cli(); //Clear all interrupts
  PCICR |= 1 << PCIE0; //Enable port B Registers i.e D8-D13
  PCMSK0 |= 1 << PCINT2;// Pin10
  sei(); //enable all interrupts
  pinMode(10, INPUT);
  digitalWrite(10, HIGH); //enable pull up in pin
//#ifdef DEBUG
  Serial.begin(115200);
//#endif
  initializeMotors();
  armAllMotors();
  initMPU();
  initPID();
}//end of setup

void loop() {
  while (!mpuInterrupt && fifoCount < packetSize) {
    //Do nothing while mpu is not working
    //they are saying it is a short delay .. i need a way to avoid this
//    Serial.print("waiting for mpu");
  }//end of while loop

  updateAnglesFromMPU();
  computePID();
  updateMotors();
}//end of loop

