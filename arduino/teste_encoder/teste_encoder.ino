volatile unsigned int counter = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(2, INPUT);
  pinMode(13, OUTPUT);
  attachInterrupt(0, rpmCount, RISING);
  Serial.begin(9600);

}

void rpmCount()
{
  counter++;
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(2)) {
    Serial.println("HIGH");
    digitalWrite(13, HIGH);
  }
  else {
    Serial.println("LOW");
    digitalWrite(13, LOW);
  }
  
  Serial.println(counter);
  
  delay(500);

}
