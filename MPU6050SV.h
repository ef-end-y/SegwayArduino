#ifndef MPU6050SV_H
#define MPU6050SV_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "MPU6050.h"

#include "./Alltunes.h"

class MPU6050SV {
  private:
    static const float RADIANS_TO_DEGREES = 57.2958; //180/3.14159
    MPU6050 instance;
    uint8_t address;
    // показания акселерометра в нулевых положениях
    int16_t zero_x_acc;
    int16_t zero_y_acc;
    int16_t zero_z_acc;
    // показание гироскопа, когда нет вращения
    float zero_gyro;
    // В эти переменные будут записаны сырые данные акселерометра
    int16_t raw_ax, raw_ay, raw_az;
    // В эти переменные будут записаны сырые данные гироскопа
    int16_t raw_gx, raw_gy, raw_gz;
    
    // Значения акселерометра при старте системы
    float base_x_acc;
    float base_y_acc;
    float base_z_acc;

    // Значение земного притяжения
    float base_g;

    float gyro_factor;

    unsigned long time;

  public:
    float ax, ay, az;
    float gx, gy, gz;

    // угол, вычесленный на предыдущей итерации
    float last_angle;
    float gyro_rate;
    float gyro_angle; 
    float acc_angle;

    MPU6050SV(uint8_t _address, float _zero_gyro, float _base_g);
    MPU6050SV(uint8_t _address, int16_t _zero_x_acc, int16_t _zero_y_acc, int16_t _zero_z_acc);
    boolean init();
    void read_acc_and_gyro();
    float angle();
    void find_zero();
};

#endif


