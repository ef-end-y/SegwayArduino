#include "./Engine.h"
#include "./Alltunes.h"

#define Conroller_port Serial2

#ifdef SABERTOOTH

Sabertooth Engine::driver = Sabertooth(SABERTOOTH_PORT);

Engine::Engine()
{
    SabertoothTXPinSerial.begin(9600);
    driver.autobaud(); // посылает специальный байт 0xAA по которому драйвер поймет скорость
    driver.setRamping(0);
    driver.setDeadband(0);
    this->stop();
}

void Engine::motor(int motor, int speed)
{
    driver.commandMotorPower(motor, constrain(speed, -127, 127));
}

float Engine::get_volts()
{
  return 0;
}

#endif

// ----------------------------------------------

#ifdef ROBOTEQ

int Engine::driver = 0;

Engine::Engine()
{  
   Serial1.begin(115200L);
   Conroller_port.begin(115200L);
   when_check_battery = 0;
   battery_ok_flag = true;
   battery_ok_last_time = millis();
}

void Engine::init()
{
   //Conroller_port.println("~ECHOF 1");
   this->stop();
   delay(50);
   this->stop();
}

void Engine::motor(int motor, int speed)
{
   Conroller_port.print("!G ");
   Conroller_port.print(motor);
   Conroller_port.print(" ");
   Conroller_port.println(constrain( speed, -MAX_SPEED, MAX_SPEED));
}

String Engine::get_var(String request, String response_prefix)
{
   Conroller_port.flush();
   unsigned long time = micros() + 100;
   while( micros() < time )
   {
      if( Conroller_port.available() )
      {
         Conroller_port.read();
         time = micros() + 100;
      }
   }
   Conroller_port.println(request);
   time = micros() + 2000;
   String res = "";
   while( micros() < time )
   {
      if( !Conroller_port.available() ) continue;
      char c = (char)Conroller_port.read();
      if( c != '\r' )
      {
          res += c;
          continue;
      }
      if( res.substring(0, 2).equals(response_prefix) ) return res.substring(2);
      res = "";
   }
   return "";
}

float Engine::motor_amps()
{
   String amps = this->get_var("?A 2", "A=");
   return amps == ""? 0.0 : amps.toInt() / 10.0;
}

float Engine::battery_volts()
{
   String volts = this->get_var("?V 2", "V=");
   return volts == ""? 0.0 : volts.toInt() / 10.0;
}

boolean Engine::is_battery_ok()
{
  unsigned long time = millis();
  if( time < when_check_battery ) return battery_ok_flag;
  float volts = this->battery_volts();
  if( volts < 0.01 )
  {
      when_check_battery = time + 100;
      return battery_ok_flag;
  }
  // Период проверки батареи
  when_check_battery = time + 300;
  if( volts >= LOW_VOLTS_BATERRY )
  {
      battery_ok_last_time = time;
      battery_ok_flag = true;
      return true;
  }
  // esli nizkiy uroven bolwe 10 sec - batareya razryazhena
  if( (time - battery_ok_last_time) > 10000 ) battery_ok_flag = false;
  return battery_ok_flag;
}

#endif

// ----------------------------------------------

void Engine::motor1(int speed)
{
    Engine::motor(1, speed);
}

void Engine::motor2(int speed)
{
    Engine::motor(2, speed); 
}

void Engine::stop()
{
    this->motor1(0);
    this->motor2(0);
}

void Engine::test()
{
    while( true )
    {
      int speed = 100;  
      this->motor1(speed);
      this->motor2(speed);
      delay(1000);
      this->stop();
      delay(1000);
      this->motor1(-speed);
      this->motor2(-speed);
      delay(1000);
      this->stop();
      delay(1000);
    }
}

