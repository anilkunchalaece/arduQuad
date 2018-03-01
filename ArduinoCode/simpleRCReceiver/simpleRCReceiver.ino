int ch3Pin = 7;
int ch1Pin = 8;
unsigned long duration3;
unsigned long duration1;

void setup()
{
  pinMode(ch3Pin, INPUT);
  pinMode(ch1Pin,INPUT);
  Serial.begin(9600);
}

void loop()
{
  duration3 = pulseIn(ch3Pin, HIGH);
  duration1 = pulseIn(ch1Pin,HIGH);
  Serial.print("Ch 3 => ");
  Serial.println(duration3);
  Serial.print("Ch 1 => ");
  Serial.println(duration1);
  delay(1000);
}
