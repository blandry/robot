#ifndef MOTO_SHIELD_H
#define MOTO_SHIELD_H

#include "Arduino.h"

enum
{
  CW,
  CCW,
  BRAKEVCC,
  BRAKEGND,
  M1,
  M2,
  BOTH
};

class MotoShield
{
  public:
    MotoShield();
    
    void begin(int threshold = 15);
    void setThreshold(int threshold);
    uint8_t getThreshold() const;
    bool isSafe(int motor);
    void stop(int motor, int state = BRAKEGND);
    void go(int motor, int direction, uint8_t pwm);

  private:
    int threshold_;
};

#endif
