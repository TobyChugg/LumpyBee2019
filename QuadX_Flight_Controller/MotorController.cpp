#include <Arduino.h>
#include "MotorController.h"

MotorController::MotorController()
{
  
}

void MotorController::calculate_esc_PWM(int throttle, Vector3 PID)
{
  esc_PWM_FL = throttle - PID.get_x() + PID.get_y() - PID.get_z(); 
  esc_PWM_FR = throttle + PID.get_x() + PID.get_y() + PID.get_z();
  esc_PWM_BL = throttle - PID.get_x() - PID.get_y() + PID.get_z();
  esc_PWM_BR = throttle + PID.get_x() - PID.get_y() - PID.get_z();
  
  if(esc_PWM_FL < 1050){esc_PWM_FL = 1000;}
  if(esc_PWM_FR < 1050){esc_PWM_FR = 1000;}
  if(esc_PWM_BL < 1050){esc_PWM_BL = 1000;}
  if(esc_PWM_BR < 1050){esc_PWM_BR = 1000;}

  if(esc_PWM_FL > 2000){esc_PWM_FL = 2000;}
  if(esc_PWM_FR > 2000){esc_PWM_FR = 2000;}
  if(esc_PWM_BL > 2000){esc_PWM_BL = 2000;}
  if(esc_PWM_BR > 2000){esc_PWM_BR = 2000;}

  if(throttle < 1050)
  {
    esc_PWM_FL = 1000;
    esc_PWM_FR = 1000;
    esc_PWM_BL = 1000;
    esc_PWM_BR = 1000;
  }
  

}

void MotorController::write_motor_PWM()
{
    while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
    loop_timer = micros();                                                    //Set the timer for the next loop.

    PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high. (turned off 6 temporarily as motor is dc'd //first bit is BL (PIN7), second bit is BR (PIN6), third bit is FR (PIN5), fourth bit is FL (PIN4)
    timer_channel_1 = esc_PWM_FL + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
    timer_channel_2 = esc_PWM_FR + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
    timer_channel_3 = esc_PWM_BL + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
    timer_channel_4 = esc_PWM_BR + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.

    while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 7 to low if the time is expired.
  }

}



int MotorController::get_esc_PWM_FL()
{
  return esc_PWM_FL;
}

int MotorController::get_esc_PWM_FR()
{
  return esc_PWM_FR;
}

int MotorController::get_esc_PWM_BL()
{
  return esc_PWM_BL;
}

int MotorController::get_esc_PWM_BR()
{
  return esc_PWM_BR;
}
