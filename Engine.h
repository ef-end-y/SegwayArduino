#ifndef ENGINE_H
#define ENGINE_H

#include <Arduino.h>
#include "./Alltunes.h"
// Драйверы двигателей
#ifdef SABERTOOTH
#include "./Sabertooth.h"
#endif

class Engine
{
  private:
    unsigned long when_check_battery;
    unsigned long battery_ok_last_time;
    boolean battery_ok_flag;
#ifdef SABERTOOTH
    static Sabertooth driver;
#endif
#ifdef ROBOTEQ
    static int driver;
    String serial_response;
#endif
  public:
    Engine();
    void init();
    void motor(int motor, int speed);
    void motor1(int speed);
    void motor2(int speed);
    void stop();
    void test();
    String get_var(String request, String response_prefix);
    float motor_amps();
    float battery_volts();
    boolean is_battery_ok();
};
#endif
