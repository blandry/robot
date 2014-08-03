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
 
 //=====================MOTOR CONTROLLER DEFINITION======================/
 
 MotoShield left_motor, right_motor;
 
 void right_motor_cb();
 void left_motor_cb();
 
 //===========================CMD MESSENGER DEFINITIONS==================================//
 
 enum
 {
   kultra_nt,
   kultra_st,
   kultra_et,
   kultra_wt,
   kright_motor,
   kleft_motor,
   kright_encoder,
   kleft_encoder
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
 
 //=========================ENCODER DEFINITIONS===========================//
 #define left_encoder_pin 2 //interrupt 0
 #define right_encoder_pin 3 //interrupt 1
 #define left_encoder_int 0
 #define right_encoder_int 1
 
volatile unsigned long int left_encoder_count = 0;
volatile unsigned long int right_encoder_count = 0;
unsigned long int right_first_count, left_first_count, right_last_count, left_last_count = 0;
int right_delta_count = 0;
int left_delta_count = 0;

unsigned long current_time, last_time, final_time = 0;
int delta_time;
 
void right_encoder_isr();
void left_encoder_isr();

/**********************************************
 *****************SETUP************************
 **********************************************/
 
void setup() {
  //Setting pins as input to save energy
  for(int i=2; i<=53; i++){
    pinMode(i, INPUT);
  }  
  
  //motor initialization
  //Put something like: right_motor.begin(x), whrere x is the threshold for current
 
  //Setting the ultrassonic trig pins as output 
  pinMode(trig_nt, OUTPUT);
  pinMode(trig_st, OUTPUT);
  pinMode(trig_et, OUTPUT);
  pinMode(trig_wt, OUTPUT);
  
  //Setting up the encoders
  pinMode(right_encoder_pin, INPUT);
  pinMode(left_encoder_pin, INPUT);
  
  //Serial baudrate defitinition
  Serial.begin(115200);
  
  //CmdMessenger setup
  ros_driver.attach(kright_motor, right_motor_cb);
  ros_driver.printLfCr(false);
  
}

/**********************************************
 ******************LOOP************************
 **********************************************/
 
void loop() {
  
    current_time = millis();
    right_first_count = right_encoder_count;
    left_first_count = left_encoder_count;
    
  while(true)
  {
    right_encoder_count++;
    left_encoder_count++;
   
    if( (current_time - last_time) >= 30 ){
      last_time = current_time;
      
      delta_time = millis() - current_time;
      right_delta_count = right_encoder_count - right_first_count;
      left_delta_count = left_encoder_count - left_first_count; 
      
      ros_driver.sendCmdStart(kright_encoder);
      ros_driver.sendCmdArg<int>(right_delta_count);
      ros_driver.sendCmdArg<int>(delta_time);
      ros_driver.sendCmdEnd();
      
      ros_driver.sendCmdStart(kleft_encoder);
      ros_driver.sendCmdArg<int>(left_delta_count);
      ros_driver.sendCmdArg<int>(delta_time);
      ros_driver.sendCmdEnd();
    }
    
  }  
  
  ros_driver.sendCmd<int>( kultra_nt, distance(trig_nt, echo_nt) );
  ros_driver.sendCmd<int>( kultra_st, distance(trig_st, echo_st) );
  ros_driver.sendCmd<int>( kultra_et, distance(trig_et, echo_et) );
  ros_driver.sendCmd<int>( kultra_wt, distance(trig_wt, echo_wt) );  
  
  ros_driver.feedinSerialData(); 
  
  delay(10);
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
  
  return pulseIn(echo_pin, HIGH, 80000) / 58;
}

void right_motor_cb()
{
}

void left_motor_cb()
{
}
