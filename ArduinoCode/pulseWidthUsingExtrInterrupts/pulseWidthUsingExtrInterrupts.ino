/*
 * Author : Kunchala Anil
 * Email : anilkunchalaece@gmail.com
 * Date : 02 June 2018
 * 
 * using external interrupts to measure the pulse width
 */
volatile int pwmValue[2];
volatile int prevTime[2];
volatile boolean readVRA;
volatile boolean readVRB;

#define VRA_CHANNEL 2
#define VRB_CHANNEL 3
 
void setup() {
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(VRA_CHANNEL), risingVRA, RISING);
  attachInterrupt(digitalPinToInterrupt(VRB_CHANNEL),risingVRB,RISING);
}
 
void loop() { 
  Serial.print(pwmValue[0]);
  Serial.print("  ");
  Serial.println(pwmValue[1]);
  }//end of loop
 
void risingVRA() {
  attachInterrupt(digitalPinToInterrupt(VRA_CHANNEL), fallingVRA, FALLING);
  prevTime[0] = micros();
}
 
void fallingVRA() {
  attachInterrupt(digitalPinToInterrupt(VRA_CHANNEL), risingVRA, RISING);
  
  pwmValue[0] = micros()-prevTime[0];
}

void risingVRB(){
  attachInterrupt(digitalPinToInterrupt(VRB_CHANNEL),fallingVRB,FALLING);
  prevTime[1] = micros();
}

void fallingVRB(){
  attachInterrupt(digitalPinToInterrupt(VRB_CHANNEL),risingVRB,RISING);
  pwmValue[1] = micros() - prevTime[1];
}

