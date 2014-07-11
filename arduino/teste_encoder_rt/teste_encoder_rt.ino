volatile unsigned int counter = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(2, INPUT);
  pinMode(13, OUTPUT);
  attachInterrupt(0, rpmCount, RISING);
}

void rpmCount()
{
  counter++;
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(2)) {
    digitalWrite(13, HIGH);
  }
  else {
   digitalWrite(13, LOW);
  }
}
