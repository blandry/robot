/*!
 *\file arduino1_driver.ino
 *
 *\section LICENSE
 *The MIT License
 *
 *Copyright (c) 2014 Francisco Edno de Moura Rodrigues Filho
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
 *and associated documentation files (the "Software"), to deal in the Software without 
 *restriction, including without limitation the rights to use, copy, modify, merge, publish, 
 *distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom 
 *the Software is furnished to do so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all copies or 
 *substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 *INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 *PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 *FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR 
 *OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 *DEALINGS IN THE SOFTWARE.
 *
 *\section Description
 *This file implements the low level driver for the ros package arduino1_driver in the
 *robot project core components.
 */
 
 #include <MotoShield.h> 
 #include <CmdMessenger.h>
 
 //===========================CMD MESSENGER DEFINITIONS==================================//
 
 enum
 {
   kultra_nt,
   kultra_st,
   kultra_et,
   kultra_wt
 };
 
 CmdMessenger ros_driver(Serial);
 
 //===========================ULTRASSONIC SENSORS DEFINITIONS=============================//

 //These are the pins for the ultrassonic sensors
 //Note the pattern trig_xx or echo_xx, representing the trig
 //and the echo pins respectively, where the "xx" represents the position
 //of the sensor in the robot. Possible values are: nt(north), st(south),
 //et(east), wt(west), nw(north-west), ne(north-east), sw(south-west) and
 //se(south-east).
 
 #define trig_nt 22
 #define echo_nt 23
 
 #define trig_st 24
 #define echo_st 25
 
 #define trig_et 26
 #define echo_et 27
 
 #define trig_wt 28
 #define echo_wt 29
 
 /*!
  * Calculates the distance from a given sensor.
  * \param trig_pin Number of the arduino pin where the trigger pin of the sensor is connected
  * \param echo_pin Number of the arduino pin where the echo pin of the sensor is connected
  *
  * \return An unsigned long representing the calculated distance.
  */
 unsigned long distance(int trig_pin, int echo_pin);
 
 //=====================MOTOR CONTROLLER DEFINITION======================/
 

/**********************************************
 *****************SETUP************************
 **********************************************/
 
void setup() {
  for(int i=2; i<=53; i++){
    pinMode(i, INPUT);
  }  
  
  pinMode(trig_nt, OUTPUT);
  
  Serial.begin(9600);
  
  ros_driver.printLfCr(false);
  
}

/**********************************************
 ******************LOOP************************
 **********************************************/
 
void loop() {
  ros_driver.sendCmd<int>( kultra_nt, distance(trig_nt, echo_nt) );
  delay(50);
}

/***********************************************
 ***********FUNCTIONS IMPLEMENTATION************
 ***********************************************/
 
unsigned long distance(int trig_pin, int echo_pin)
{
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  
  digitalWrite(trig_pin, LOW);
  
  return pulseIn(echo_pin, HIGH)/58.2;
}
