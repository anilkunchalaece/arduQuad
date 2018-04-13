/*
  Author : Kunchala Anil
  email : anilkunchalaece@gmail.com
  Date : 14 Mar 2018

  Checking the altitude of KSRM Quad


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

#define rxPin 10
#define DEBUG

//MotorConnections

#define frontLeftMotorPin 4
#define frontRightMotorPin 5
#define backLeftMotorPin 6
#define backRightMotorPin 7

#define motorArmValue 60
#define motorMinValue 65
#define motorMaxValue 130

#define throttleMinValue 1200
#define throttleMaxValue 1800
#define pitchMinValue 1200
#define pitchMaxValue 1800
#define rollMinValue 1200
#define rollMaxValue 1800
#define yawMinValue 1200
#define yawMaxValue 1800



volatile boolean recvPCInt = false;
volatile byte portValue;
unsigned long pwmDuration;
unsigned long pwmStart;
unsigned long pwmEnd;

Servo frontLeftMotor;
Servo frontRightMotor;
Servo backLeftMotor;
Servo backRightMotor;

//Interrupt Service Routine will fire when for PinChange in PortB
ISR(PCINT0_vect) {
  recvPCInt = true;
  portValue = PINB; //PINB is used to read all port input values
}

void updateMotors(int val) {
  frontLeftMotor.write(val);
  frontRightMotor.write(val);
  backLeftMotor.write(val);
  backRightMotor.write(val);
#ifdef DEBUG
  Serial.print("motors are updated with value => ");
  Serial.println(val);
#endif
}


void initializeMotors() {
  frontLeftMotor.attach(frontLeftMotorPin);
  frontRightMotor.attach(frontRightMotorPin);
  backLeftMotor.attach(backLeftMotorPin);
  backRightMotor.attach(backRightMotorPin);
#ifdef DEBUG
  Serial.println("motors initialized");
#endif
}//end of initializeMotorsFunction


void armAllMotors() {
  frontLeftMotor.write(motorArmValue);
  frontRightMotor.write(motorArmValue);
  backLeftMotor.write(motorArmValue);
  backRightMotor.write(motorArmValue);
#ifdef DEBUG
  Serial.println("motors are Armed ");
#endif
}//end of armAllMotors function



void setup() {
  cli(); //Clear all interrupts
  PCICR |= 1 << PCIE0; //Enable port B Registers i.e D8-D13
  PCMSK0 |= 1 << PCINT2;// Pin9
  sei(); //enable all interrupts
  pinMode(rxPin, INPUT);
  digitalWrite(rxPin, HIGH); //enable pull up in pin
#ifdef DEBUG
  Serial.begin(9600);
#endif
  initializeMotors();
  armAllMotors();
}//end of setup

void loop() {
  if (recvPCInt == true) {
    boolean event = digitalRead(rxPin);
    if (event == HIGH)
    {
      pwmStart = micros();
    }
    else if (event == LOW) {
      pwmEnd = micros();
      int width = pwmEnd - pwmStart;

      int mappedValue = map(width, throttleMinValue, throttleMaxValue, motorMinValue, motorMaxValue);

       updateMotors(mappedValue);

#ifdef DEBUG
      Serial.print("Throttle Value is  ");
      Serial.println(pwmEnd - pwmStart);
#endif
    }
    recvPCInt = false;
  }//end of If

}//end of loop


