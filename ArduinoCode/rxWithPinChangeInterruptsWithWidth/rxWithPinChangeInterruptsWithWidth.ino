 /*
 * Author : Kunchala Anil
 * email : anilkunchalaece@gmail.com
 * Date : 01 Mar 2018
 * 
 * Pin Change Interrupts - Single Rx Channel i.e Single PWM Wave
 * 
 */

//PCMSKx - Pin Change Mask Register
//PCMSK0 - portB (D8-D13) (PCINT0  - PCINT6)
//PCMSK1 - portC (A0-A5)  (PCINT8  - PCINT14)
//PCMSK2 - portD (D0-D7)  (PCINT16 - PCINT23)
//PCICR -  Pin Change Interrupt Control Register
//  PCIE0 - Pin Change Interrupt Enable 0 - Port B (D8-D13)
//  PCIE1 - Pin Change Interrupt Enable 1 - Port C (A0-A5)
//  PCIE2 - Pin Change Interrupt Enable 2 - Port D (D0-D7)

#define rxPin 10

volatile boolean recvPCInt = false;
volatile byte portValue;
unsigned long pwmDuration;
unsigned long pwmStart;
unsigned long pwmEnd;


//Interrupt Service Routine will fire when for PinChange in PortB
ISR(PCINT0_vect){
  recvPCInt = true;
  portValue = PINB; //PINB is used to read all port input values
}

void setup() {
cli(); //Clear all interrupts
PCICR |= 1 << PCIE0; //Enable port B Registers i.e D8-D13
PCMSK0 |= 1 << PCINT2;// Pin10
sei(); //enable all interrupts
pinMode(rxPin,INPUT);
digitalWrite(rxPin,HIGH);//enable pull up in pin 9
Serial.begin(9600);
}

void loop() {
  if(recvPCInt == true){
     boolean event = digitalRead(rxPin);
     if(event == HIGH)
        {
          pwmStart = micros();
        }
       else if(event == LOW){
          pwmEnd = micros();
        Serial.println(pwmEnd-pwmStart);
       }
     recvPCInt = false;
  }
}
