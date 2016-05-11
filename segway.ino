#include "./Alltunes.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"      // Акселерометр и гироскоп
#include "MPU6050SV.h"    // Акселерометр и гироскоп, высокоуровневая прослойка
#include "./PID.h"
#include "./Engine.h"     // Двигатели
#include "./Leds.h"       // Светодиодная индикация
#include "./Smooth.h"     // Сглаживание значений

String Command;

#ifdef NEW_ANGLE_METHOD
  MPU6050SV mpu1 = MPU6050SV(MPU6050_ADDRESS_AD0_LOW, -226.5, 16.0);
  MPU6050SV mpu2 = MPU6050SV(MPU6050_ADDRESS_AD0_HIGH, -86, 16.0);
#else
  MPU6050SV mpu1 = MPU6050SV(MPU6050_ADDRESS_AD0_LOW, -14.00, -626.00, -1310.00);
  MPU6050SV mpu2 = MPU6050SV(MPU6050_ADDRESS_AD0_HIGH, 128.00, -576.00, -2566.00);
#endif

Leds    Screen = Leds();
PID     Pid    = PID();
Engine  Motor  = Engine();

Smooth  Smooth_avg_speed = Smooth(10); // средняя скорость за Smooth периодов. 
                                       // Используется для остановки двигателей - с этой скорости будет постепенное падение
float   Avg_speed = 0;

Smooth  Smooth_rate = Smooth(1);
Smooth  l_speed = Smooth(1);
Smooth  r_speed = Smooth(1);

int s_wheel0 = 0; // значение на входе когда руль в центральном положении
int stop_angle_count = 0; // счетчик сколько циклов подряд превышен критический угол

unsigned long time;

void setup()
{ 
  LOG.begin(SERIAL_LOG_SPEED);
  LOG.println("Segway starts...");
  
  Screen.check(); // Зажгутся все индикаторы для визуального тестирования их работоспособности
  Screen.wait(1300); // Пока устаканятся все процессы будем отображать на индикаторах режим ожидания

  Motor.init();

#ifdef DEBUG
  if( DEBUG == 10 ) Motor.test();
#endif

  if( !mpu1.init() ) Sensor_error();
  if( !mpu2.init() ) Sensor_error();
  
  // Центральное положение руля
  pinMode(STEERING_WHEEL, INPUT);
  int max_try = 20; // не много, чтобы не было переполнения s_wheel0
  for( int i=0; i<max_try; i++ )
  {
      s_wheel0 += analogRead(STEERING_WHEEL);
      delay(1);
  }
  s_wheel0 /= max_try;

  time = micros();
}

// ----------------------  Main loop ----------------------

void loop()
{
  unsigned long time = micros();

  Motor.is_battery_ok()? Screen.battery_ok() : Screen.battery_error();

  Screen.show();

  float angle = mpu1.angle();
  float angle2 = mpu2.angle();

#ifdef DEBUG
  debug_params(mpu1, mpu2, angle, angle2);
  if(DEBUG < 10) return;
#endif
  
  float speed = Pid.update(angle, mpu1.gyro_rate);

  Avg_speed = Smooth_avg_speed.get(speed);
  
  Check_stop_angle(angle);

  int s_wheel = analogRead(STEERING_WHEEL) - s_wheel0;
  float turn = abs(s_wheel) < 3? 0 : (s_wheel>0? 1 : -1) * pow(abs(s_wheel), 1.5) * 0.4;
  turn = constrain(turn, -400, 400);
  //turn = 0;
 
  float r_spd = r_speed.get(speed - turn);
  float l_spd = l_speed.get(speed + turn);

#ifdef DEBUG
  if( DEBUG == 11 )
  {
      LOG.print("[{\"s1\":");
      LOG.print(-speed);
      LOG.print(",\"s2\":");
      LOG.print(angle*10);
      LOG.println("}]");
  }
#endif

  Motor.motor1(r_spd);
  Motor.motor2(l_spd);
  
  ProcessCommand();

#ifdef DEBUG
  if( DEBUG == 5 )
  {
      long tm_delta = micros() - time;
      LOG.print("[{time:");
      LOG.print(tm_delta);
      LOG.println("}]");
      return;
  }
  if( DEBUG == 13 )
  {
      LOG.print("[{\"angle\":");
      LOG.print(-angle);
      LOG.print(",\"amp\":");
      LOG.print(-Motor.motor_amps());
      LOG.print(",\"battery\":");
      LOG.print(Motor.battery_volts());
      LOG.println("}]");
  }
#endif

  while( (micros() - time) < MAIN_PERIOD );
}

void ProcessCommand()
{
  while( LOG.available() )
  {
      char c = (char)LOG.read();
      run_command(c);
  }

/*
  while( LOG.available() )
  {
      char c = (char)LOG.read();
      if(c == '\n')
      {
        parseCommand(Command);
        Command = "";
      }else
      {
        Command += c;
      }
  }
*/
}

void Sensor_error()
{
  Screen.general_error();
}

void Stop_message(String msg)
{
  while( true )
  {
      LOG.println(msg);  
      for( int i=0; i < 100; i++ )
      {
          Screen.show();
          delay(10);
      }
  }
}

void Stop_engine(String msg)
{
  Screen.angle_error();
  int max_i = 20; // за сколько шагов уменьшить мощность на двигатели до нуля
  int delay_time = int(STOP_ENGINE_SEC * 1000 / max_i);
  float delta_speed = Avg_speed / max_i;  
  for( int i=0; i < max_i; i++ )
  {
      Avg_speed -= delta_speed;
      Motor.motor1(Avg_speed);
      Motor.motor2(Avg_speed);
      Screen.show();
      delay(delay_time);
  }
  Motor.stop();
  Stop_message(msg);
}

void Check_stop_angle(float angle)
{
  if( abs(angle) < STOP_ANGLE )
  {
    stop_angle_count = 0;
    return;
  }
  if( ++stop_angle_count > STOP_ANGLE_COUNT ) Stop_engine("Max angle!");
}


#ifdef DEBUG
void debug_params(MPU6050SV mpu1, MPU6050SV mpu2, float angle, float angle2)
{
  if( DEBUG == 1 || DEBUG == 12 )
  {
    LOG.print("[{acc_angle:");
    LOG.print(mpu1.acc_angle);
    LOG.print(",gyro_angle:");
    LOG.print(mpu1.gyro_angle);
    LOG.print(",angle:");
    LOG.print(angle);
    LOG.print(",angle2:");
    LOG.print(angle2);
    LOG.print("},{ax:");
    LOG.print(mpu1.ax);
    LOG.print(",ay:");
    LOG.print(mpu1.ay);
    LOG.print(",az:");
    LOG.print(mpu1.az);
    LOG.print("},{gyro_rate:");
    LOG.print(mpu1.gyro_rate);
    LOG.println("}]");
  }
  if( DEBUG == 2 )
  {
    LOG.print("[{a_angle1:");
    LOG.print(mpu1.acc_angle);
    LOG.print(",g_angle1:");
    LOG.print(mpu1.gyro_angle);
    LOG.print(",a_angle2:");
    LOG.print(mpu2.acc_angle);
    LOG.print(",g_angle2:");
    LOG.print(mpu2.gyro_angle);
    LOG.print("},{angle:");
    LOG.print(angle);
    LOG.print(",angle2:");
    LOG.print(angle2);
    LOG.println("}]");
  }
  if( DEBUG == 8 )
  {
    int wheel = analogRead(STEERING_WHEEL) - s_wheel0;
    LOG.print("[{wheel:");
    LOG.print(wheel);
    LOG.println("}]");
  }
}
#endif

/*
void parse_command(String command)
{
  int spaceIndex = command.indexOf(" ");
  String part1 = command.substring(0, spaceIndex);
  String part2 = command.substring(spaceIndex + 1);
  if( part1.equalsIgnoreCase("kp") )
  {
      Pid.kp = part2.toInt();
  }
   else if( part1.equalsIgnoreCase("1") )
  {
      LOG.println("Reset MPU 1");  
      mpu1.init();
  }
   else if( part1.equalsIgnoreCase("2") )
  {
      LOG.println("Reset MPU 2");
      mpu2.init();
  }
  else
  {
      LOG.println("UNKNOWN COMMAND");
      return;
  }
  LOG.println("OK");
}
*/

void run_command(char command)
{
  if( command == '1' )
  {
    Pid.kp *= 1.1;
    LOG.print("kp: ");
    LOG.println(Pid.kp);
    return;
  }
  if( command == '2' )
  {
    Pid.kp *= 0.9;
    LOG.print("kp: ");
    LOG.println(Pid.kp);
    return;
  }

  LOG.println("UNKNOWN COMMAND");
  return;
}

