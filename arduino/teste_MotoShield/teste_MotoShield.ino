#include <MotoShield.h>

MotoShield motor;
int pwm = 0;

void setup() {
  // put your setup code here, to run once:
  motor.begin(15);
}

void loop() {
  // put your main code here, to run repeatedly:
motor.go(M1, CCW, 100);
motor.go(M2, CCW, 100);
 
}
