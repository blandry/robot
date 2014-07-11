#include <MotoShield.h>

MotoShield motor;
int pwm = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  motor.begin(15);
}

void loop() {
  // put your main code here, to run repeatedly:
 if(Serial.available() > 0){
   int tmp = Serial.parseInt();
   if(tmp) pwm = tmp;
   Serial.println(pwm, DEC);
 }
 
 if(pwm < 0){
   motor.go(M1, CW, abs(pwm));
 }
 else
 {
   motor.go(M1, CCW, pwm);
 }
}
