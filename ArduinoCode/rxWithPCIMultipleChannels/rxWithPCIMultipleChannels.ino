/*
 * Author : Kunchala Anil
 * Email : anilkunchlaece@gmail.com
 * Pin Change Interrupts - Single Rx Channel i.e Single PWM Wave
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
* Rx Connections
* Ch1 = D8 PCINT0
* Ch2 = D9 PCINT1
* Ch3 = D10 PCINT2
* Ch4 = D11 PCINT3
*/
#define DEBUG // comment it out to disable the debug mode

volatile boolean recvPCInt = false; //to know interrupt status

//used to store the pwm duration
unsigned long pwmDuration[4];
unsigned long pwmStart[4];
unsigned long pwmEnd[4];

//pinDeclaration for Rx
const byte rxCh[] = {8,9,10,11};
const byte noOfChannels = sizeof(rxCh);

//portStatus
const byte prevPortState[] = {1,1,1,1};
const byte presentPortState[4];

//Interrupt Service Routine will fire when for PinChange in PortB
ISR(PCINT0_vect){
  recvPCInt = true;
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

for (int ch=0 ; ch < noOfChannels; ch++){
  pinMode(rxCh[ch],INPUT_PULLUP);//make pin input with pullup enabled
}//end of for loop

}//end os setup

void loop() {
  if(recvPCInt == true){
  for (int ch=0; ch < noOfChannels; ch++){
    boolean event = digitalRead(rxCh[ch]);
    if(event == HIGH){
      pwmStart[ch] = micros();
    }else if(event == LOW){
      pwmEnd[ch] = micros();
      pwmDuration[ch] = pwmEnd[ch] - pwmStart[ch];
      #ifdef DEBUG
      Serial.print("pwm duration for channel ");
      Serial.print(ch);
      Serial.print(" ");
      Serial.println(pwmDuration[ch]);
      #endif 
    }//end of ifElseif
    }//end of for loop
    recvPCInt = false;
  }
}
