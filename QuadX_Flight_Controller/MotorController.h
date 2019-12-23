#ifndef MotorController_h
#define MotorController_h

#include <Arduino.h>
#include "Vector3.h"



class MotorController
{
  public:

  unsigned long loop_timer;
  unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
  
  MotorController();
  
  void calculate_esc_PWM(int throttle, Vector3 PID);
  void write_motor_PWM();
  
  int get_esc_PWM_FL();
  int get_esc_PWM_FR();
  int get_esc_PWM_BL();
  int get_esc_PWM_BR();

  
  private:
  
  int esc_PWM_FL; //FrontLeft etc...
  int esc_PWM_FR;
  int esc_PWM_BL;
  int esc_PWM_BR;
  
  
};

#endif
