#ifndef PID_h
#define PID_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "./Smooth.h"

Smooth  Smooth_speed = Smooth(10);

class PID
{
  public:
    float i_error;
    float kp;
    float charge;
    float last_speed;
    float cur_speed;
    float sp;
    unsigned long tm;

  PID()
  {
    kp = 10;
    i_error = 0;
    charge = 0;
    last_speed = 0;
    cur_speed = 0;
    tm = millis();
  }

  int update(float angle, float angle_rate)
  {
    //return this->fake();
    float p = 0;
    float d = 0;
   
    float setpoint = angle;
    float as = abs(setpoint);

    p = (setpoint>0? 1 : -1.5) * (kp * pow(as, 1.7));

    float speed = p + d;
    cur_speed = speed;

    return -constrain(speed, -32000, 32000);
  }

  void flush()
  {
      PID();
  }
  
  int fake()
  {
      long now = millis() - tm;
      now -= 5000;
      if(now < 0 ) return 0;
      now -= 300;
      if(now < 0 ) return -100;
      tm = millis();
      return 0;
  }
};


#endif
