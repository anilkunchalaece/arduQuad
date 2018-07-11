/*
 * Author : Kunchala Anil
 * Email : anilkunchalaece@gmail.com
 * Date : 14 June 2018
 * 
 * This sketch is used to calibrate the ESC's
 * 
 * 
 */

//variables used in pinChange ISR
volatile boolean recvPCInt = false;

//variables to store the motor values calculated using pid and throttle
int m0Value, m1Value, m2Value, m3Value;

//used to store the pwm duration
volatile int pwmDuration[4];
volatile unsigned long pwmStart[4];

//pinDeclaration for Rx
const byte rxCh[] = {8, 9, 10, 11};
const byte noOfChannels = sizeof(rxCh);

//portStatus
volatile byte prevPortState[] = {0, 0, 0, 0};
volatile byte presentPortState[4];

//MotorConnections
#define m0 4
#define m1 5
#define m2 6
#define m3 7

unsigned long loopTimer=0; //to calculate dt for gyro integration

unsigned long now = 0, difference = 0;

//calibration variables
char chRecv ;
boolean startCalibration = false;

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

}//end of ISR Fcn

void initReceiver() {
  cli(); //Clear all interrupts
  PCICR |= 1 << PCIE0; //Enable port B Registers i.e D8-D13
  PCMSK0 |= 1 << PCINT3 | 1 << PCINT2 | 1 << PCINT1 | 1 << PCINT0 ; // Pin11,10,9,8
  sei(); //enable a
  for (int i = 0 ; i < noOfChannels ; i++) {
    pinMode(rxCh[i], INPUT);
    digitalWrite(rxCh[i], HIGH); //enable pullup
  }//end of for loop

}//end of initReceiver Fcn

void initializeMotors() {
    pinMode(m0,OUTPUT);
    pinMode(m1,OUTPUT);
    pinMode(m2,OUTPUT);
    pinMode(m3,OUTPUT);
}//end of initializeMotors Fcn


void setup() {
  // put your setup code here, to run once:
  initReceiver();
  initializeMotors();

  Serial.begin(9600);
  Serial.println("send a to start calibration");
  Serial.println("send b to stop calibration");
  
}

void loop() {
loopTimer = micros() ; // used to calculate the loop timer [do we need it?]
if (Serial.available()){
  char recvChar = Serial.read();
  
  if (recvChar == 'a'){
    startCalibration = true;
  }

  if (recvChar == 'b'){
    startCalibration = false;
  }
}

if(startCalibration == true){
  // put your main code here, to run repeatedly:
int throttle = pwmDuration[2];
m0Value = throttle;
m1Value = throttle;
m2Value = throttle;
m3Value = throttle;

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
}//end of If
}//end of loop
