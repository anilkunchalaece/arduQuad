/*
   Author : Kunchala Anil
   Email : anilkunchlaece@gmail.com
   Date : 6 Mar 2018
   Pin Change Interrupts - Single Rx Channel i.e Single PWM Wave
  PCMSKx - Pin Change Mask Register
  PCMSK0 - portB (D8-D13) (PCINT0  - PCINT6)
  PCMSK1 - portC (A0-A5)  (PCINT8  - PCINT14)
  PCMSK2 - portD (D0-D7)  (PCINT16 - PCINT23)
  PCICR -  Pin Change Interrupt Control Register
    PCIE0 - Pin Change Interrupt Enable 0 - Port B (D8-D13)
    PCIE1 - Pin Change Interrupt Enable 1 - Port C (A0-A5)
    PCIE2 - Pin Change Interrupt Enable 2 - Port D (D0-D7)
*/

/*#############################
  Rx Connections
  Ch1 = D8 PCINT0
  Ch2 = D9 PCINT1
  Ch3 = D10 PCINT2
  Ch4 = D11 PCINT3
*/
#define DEBUG // comment it out to disable the debug mode

volatile boolean recvPCInt = false; //to know interrupt status
volatile int portValue;

//used to store the pwm duration
volatile unsigned long pwmDuration[4];
volatile unsigned long pwmStart[4];
unsigned long pwmEnd[4];

//pinDeclaration for Rx
const byte rxCh[] = {8, 9, 10, 11,12};
const byte noOfChannels = sizeof(rxCh);

//portStatus
volatile int prevPortState[] = {0, 0, 0, 0};
volatile int presentPortState[4];

//Interrupt Service Routine will fire when for PinChange in PortB
ISR(PCINT0_vect) {
  recvPCInt = true;
  for (int ch = 0; ch < noOfChannels ; ch++) {
    presentPortState[ch] = digitalRead(rxCh[ch]);
  }//end of for loop

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

void setup() {
  cli(); //Clear all interrupts
  PCICR |= 1 << PCIE0; //Enable port B Registers i.e D8-D13
  PCMSK0 |= (1 << PCINT0) | (1 << PCINT1) | (1 << PCINT2) | (1 << PCINT3);// Pin8,9,10,11
  sei(); //enable all interrupts

#ifdef DEBUG
  Serial.begin(9600);
  Serial.println("DEBUG mode Enabled");
#endif

  for (int ch = 0 ; ch < noOfChannels; ch++) {
    pinMode(rxCh[ch], INPUT_PULLUP); //make pin input with pullup enabled
  }//end of for loop

}//end of setup

void checkForRxPulseWidths() {
  for (int i = 0; i < noOfChannels ; i++ ) {
#ifdef DEBUG
    Serial.print("ch => ");
    Serial.print(i);
    Serial.print(" pulseWidth ");
    Serial.println(pwmDuration[i]);
#endif
  }
}

void loop() {
  if (recvPCInt == true) {
    checkForRxPulseWidths();
    recvPCInt = false;
  }//end of if
}//end of loop


