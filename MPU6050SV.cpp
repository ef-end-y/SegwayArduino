#include "MPU6050SV.h"


MPU6050SV::MPU6050SV(uint8_t _address, float _zero_gyro, float _base_g)
{
    address = _address;
    base_g = _base_g;
    zero_gyro = _zero_gyro;
    zero_x_acc = 0;
    zero_y_acc = 0;
    zero_z_acc = 0;
    instance = MPU6050(address);
}

MPU6050SV::MPU6050SV(uint8_t _address, int16_t _zero_x_acc, int16_t _zero_y_acc, int16_t _zero_z_acc)
{
    address = _address;
    zero_x_acc = _zero_x_acc;
    zero_y_acc = _zero_y_acc;
    zero_z_acc = _zero_z_acc;
    zero_gyro = 0;
    instance = MPU6050(address);
}

boolean MPU6050SV::init()
{
    instance.initialize();

    if( !instance.testConnection() ) return false;

    /*
        0 = +/- 250 degrees/sec
        1 = +/- 500 degrees/sec
        2 = +/- 1000 degrees/sec
        3 = +/- 2000 degrees/sec
    */
    uint8_t FS_SEL = 1;
    instance.setFullScaleGyroRange(FS_SEL);

    // Если вдруг не установилось - будем использовать то значение, которое установлено
    uint8_t READ_FS_SEL = instance.getFullScaleGyroRange();
    // Serial.print("FS_SEL = ");
    // Serial.println(READ_FS_SEL);
    
    gyro_factor = 131.0/pow(2, READ_FS_SEL);

    // Масштаб акселерометра, не имеет значения в нашем случае
    uint8_t READ_AFS_SEL = instance.getFullScaleAccelRange();
    // Serial.print("AFS_SEL = ");
    // Serial.println(READ_AFS_SEL);

    // Калибровка
    int num_readings = 10;

    base_x_acc = 0;
    base_y_acc = 0;
    base_z_acc = 0;

    for(int i = 0; i < num_readings; i++)
    {
      read_acc_and_gyro();    
      base_x_acc += ax;
      base_y_acc += ay;
      base_z_acc += az;
#ifndef NEW_ANGLE_METHOD
      zero_gyro += gx;
#endif
      delay(50);
    }

    base_x_acc /= num_readings;
    base_y_acc /= num_readings;
    base_z_acc /= num_readings;

#ifndef NEW_ANGLE_METHOD
    zero_gyro /= num_readings;
    base_g = sqrt(base_x_acc*base_x_acc + base_y_acc*base_y_acc + base_z_acc*base_z_acc);
#endif

    last_angle = 0;
    gyro_angle = 0; 
    acc_angle = 0;
    time = micros();
}


void MPU6050SV::read_acc_and_gyro()
{
    instance.getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
    raw_ax -= zero_x_acc;
    raw_ay -= zero_y_acc;
    raw_az -= zero_z_acc;
    ax = raw_ax / 1024.0;
    ay = raw_ay / 1024.0;
    az = raw_az / 1024.0;
    gx = raw_gx;
    gy = raw_gy;
    gz = raw_gz;
}

/*
   Вычисление текущего угла:

   angle = (1-k) * gyro_angle + k * acc_angle
  
*/

float MPU6050SV::angle()
{
    unsigned long time_now = micros();
    unsigned long period = time_now - time;
    time = time_now;
  
    read_acc_and_gyro();
  
    float dt = period / 1000000.0;
  
    gyro_rate = (gx - zero_gyro) / gyro_factor;
    float delta_angle = gyro_rate * dt;
    last_angle += delta_angle;
    gyro_angle += delta_angle;
 
    float k;

#ifdef NEW_ANGLE_METHOD
    float x = ax - base_x_acc;
    float y = ay - base_y_acc;
    float z = az - base_z_acc;
    float i = sqrt(x*x + y*y + z*z) / 2 / base_g;
    acc_angle = 2 * RADIANS_TO_DEGREES * asin(i>1? 1 :i);
    if( ay > base_y_acc ) acc_angle = -acc_angle;
    k = 0.01;
#else
    // абсолютная величина текущего вектора ускорения
    float acc_len = sqrt(ax*ax + ay*ay + az*az);
   
    // косуинус угла между текущим и стартовым векторами
    float cos_angle = (ax*base_x_acc + ay*base_y_acc + az*base_z_acc) / acc_len / base_g;
   
    if(cos_angle > 1) cos_angle = 1;
  
    // угол по акселерометру в градусах
    acc_angle = acos(cos_angle) * RADIANS_TO_DEGREES;
    if( ay > base_y_acc ) acc_angle = -acc_angle;
   
    k = K_MAX - (K_MAX - K_MIN) * abs(acc_len-base_g) / base_g / (ACC_MAX-1);
    if( k < K_MIN ) k = K_MIN;
#endif

    // Вычислим угол как среднее между старым значением и новым
    // (вернее не среднее, а в заданном соотношении k)
    float angle = (1-k)*last_angle + k*acc_angle;
    last_angle = angle;
    return(angle);
}

void MPU6050SV::find_zero()
{
    float min_x = 10000;
    float max_x = 0;
    float min_y = 10000;
    float max_y = 0;
    float min_z = 10000;
    float max_z = 0;
    for( ;; )
    {
        read_acc_and_gyro();
        float g = sqrt(ax*ax + ay*ay +az*az);
        Serial.print("g=");
        Serial.print(g);
        Serial.print(" base_g=");
        Serial.print(base_g);
        Serial.println("");
        if( true or (abs(g - base_g) / base_g) < 0.005 )
        {
          if( raw_ax < min_x ) min_x = raw_ax;
          if( raw_ax > max_x ) max_x = raw_ax;
          if( raw_ay < min_y ) min_y = raw_ay;
          if( raw_ay > max_y ) max_y = raw_ay;
          if( raw_az < min_z ) min_z = raw_az;
          if( raw_az > max_z ) max_z = raw_az;
          Serial.print("x=");
          Serial.print((float)((max_x+min_x)/2));
          Serial.print(" y=");
          Serial.print((float)((max_y+min_y)/2));
          Serial.print(" z=");
          Serial.print((float)((max_z+min_z)/2));
          Serial.println("");
          delay(300);
        }
    }
}

