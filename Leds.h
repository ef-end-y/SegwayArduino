#ifndef LEDS_H
#define LEDS_H

#include <Arduino.h>
#include "./Alltunes.h"

int LED_PINS[] =  {7, 6, 5, 3, 4};
#define PINS_COUNT  5
#define LED_POWER 4
#define LED_BATTERY 3
#define LED_ANGLE 5
#define LED_GENERAL_ERROR 6

#define LED_HIGH LOW
#define LED_LOW HIGH

class Leds
{
  private:
    boolean led_battery_error;
    boolean led_angle_error;
    boolean led_general_error;
    unsigned long start_time;

  public:
    Leds()
    {
        led_battery_error = false;
        led_angle_error = false;
        led_general_error = false;
        start_time = millis();
        for( int i=0; i < PINS_COUNT; i++)
        {
            pinMode(LED_PINS[i], OUTPUT);
        }
        hideAll();
    }

    void show()
    {
        int led_level = (millis() - start_time) % 300 > 150? LED_LOW : LED_HIGH;
        digitalWrite(LED_POWER, LED_HIGH);
        digitalWrite(LED_BATTERY, led_battery_error? led_level : LED_LOW);
        digitalWrite(LED_ANGLE, led_angle_error? led_level : LED_LOW);
        digitalWrite(LED_GENERAL_ERROR, led_general_error? led_level : LED_LOW);
    }

    void check()
    {
        showAll();
        delay(500);
        hideAll();
    }
    void show(int pin)
    {
        digitalWrite(LED_PINS[pin], LED_HIGH);
    }
    void hide(int pin)
    {
        digitalWrite(LED_PINS[pin], LED_LOW);
    }
    void showAll()
    {
        for(int i=0; i < PINS_COUNT; i++) show(i);
    }
    void hideAll()
    {
        for(int i=0; i < PINS_COUNT; i++) hide(i);
    }
    void wait(int ms)
    {
        hideAll();
        unsigned long time_end = millis() + ms;
        int i=0;
        while( millis() < time_end )
        {
          hideAll();
          show(i);
          delay(90);
          if( ++i == 3) i = 0;
        }
        hideAll();
    }
    void battery_error()
    {
      led_battery_error = true;
    }
    void battery_ok()
    {
      led_battery_error = false;
    }
    void angle_error()
    {
      led_angle_error = true;
    }
    void angle_ok()
    {
      led_angle_error = false;
    }
    void general_error()
    {
      led_general_error = true;
    }
};
#endif
