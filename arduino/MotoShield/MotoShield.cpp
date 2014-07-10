#include "MotoShield.h"

//Configuration pins

#define A1 7
#define B1 8
#define A2 4
#define B2 9

#define CS1 2
#define CS2 3

#define PWM1 5
#define PWM2 6

MotoShield::MotoShield()
{
}

void MotoShield::begin(int threshold)
{
  MotoShield::setThreshold(threshold);

  pinMode(A1, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(B2, OUTPUT);
  
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);

  digitalWrite(A1, LOW);
  digitalWrite(B1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(B2, LOW);
}

void MotoShield::setThreshold(int threshold)
{
  threshold_ = threshold;
}

uint8_t MotoShield::getThreshold() const
{
  return threshold_;
}

bool MotoShield::isSafe(int motor)
{
  if(motor == M1) return threshold_ <= digitalRead(CS1);
  else if(motor == M2) return threshold_ <= digitalRead(CS2);
  else return false;
}

void MotoShield::stop(int motor, int state)
{

  if(state == BRAKEGND)
  {
    switch(motor)
    {
      case M1:
        digitalWrite(A1, LOW);
        digitalWrite(B1, LOW);
        break;

      case M2:
        digitalWrite(A2, LOW);
        digitalWrite(B2, LOW);
        break;

      case BOTH:
        digitalWrite(A1, LOW);
        digitalWrite(B1, LOW);
        digitalWrite(A2, LOW);
        digitalWrite(B2, LOW);
    }
  } 
  else if(state == BRAKEVCC)
  {
    switch(motor)
    {
      case M1:
        digitalWrite(A1, HIGH);
        digitalWrite(B1, HIGH);
        break;

      case M2:
        digitalWrite(A2, HIGH);
        digitalWrite(B2, HIGH);
        break;

      case BOTH:
        digitalWrite(A1, HIGH);
        digitalWrite(B1, HIGH);
        digitalWrite(A2, HIGH);
        digitalWrite(B2, HIGH);
        break; 
    }
  }
}

void MotoShield::go(int motor, int direction, uint8_t pwm)
{
  switch(motor)
  {
    case M1:
      if(direction == CW){
        digitalWrite(A1, HIGH);
        digitalWrite(B1, LOW);
        analogWrite(PWM1, pwm);
      }else if(direction == CCW){
        digitalWrite(A1, LOW);
        digitalWrite(B1, HIGH);
        analogWrite(PWM1, pwm);
      }
      break;

    case M2:
      if(direction == CW){
        digitalWrite(A2, HIGH);
        digitalWrite(B2, LOW);
        analogWrite(PWM2, pwm);
      }else if(direction == CCW){
        digitalWrite(A2, LOW);
        digitalWrite(B2, HIGH);
        analogWrite(PWM2, pwm);
      }
      break;

    case BOTH:
      if(direction == CW){
        digitalWrite(A1, HIGH);
        digitalWrite(A2, HIGH);
        digitalWrite(B1, LOW);
        digitalWrite(B2, LOW);

        analogWrite(PWM1, pwm);
        analogWrite(PWM2, pwm);
      }else if(direction == CCW){
        digitalWrite(A1, LOW);
        digitalWrite(A2, LOW);
        digitalWrite(B1, HIGH);
        digitalWrite(B2, HIGH);

        analogWrite(PWM1, pwm);
        analogWrite(PWM2, pwm);
      }
  }
} 
