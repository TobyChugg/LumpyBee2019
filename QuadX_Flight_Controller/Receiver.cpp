#include "Arduino.h"
#include "Receiver.h"

//global variables
  byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
  unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
  int receiver_input[5];
  
 

Receiver::Receiver()
{
  
}

void Receiver::initialise_interrupts()
{
  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.
}



//interrupts -----------------------------------------

ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                                     //Is input 8 high?
    if(last_channel_1 == 0){                                                //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input[4] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                                    //Is input 9 high?
    if(last_channel_2 == 0){                                                //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                                    //Is input 10 high?
    if(last_channel_3 == 0){                                                //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                                    //Is input 11 high?
    if(last_channel_4 == 0){                                                //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
    
  }  
}

int Receiver::get_receiver_input(int i)
{
  return receiver_input[i];
}

int Receiver::get_receiver_throttle()
{
  int temp;
  temp = map(receiver_input[1], 1000, 2000, 1000, 1800); //making sure the PID control has +400 PWM's worth of control
  
  return temp;
}

Vector3 Receiver::get_stick_PWM()
{
  Vector3 temp_Vector3;

  //deadband
  if(receiver_input[4] < 1492 || receiver_input[4] > 1508) {temp_Vector3.set_x(receiver_input[4]);} else {temp_Vector3.set_x(1500);} //roll
  if(receiver_input[3] < 1492 || receiver_input[3] > 1508) {temp_Vector3.set_y(receiver_input[3]);} else {temp_Vector3.set_y(1500);} //pitch
  if(receiver_input[2] < 1492 || receiver_input[2] > 1508) {temp_Vector3.set_z(receiver_input[2]);} else {temp_Vector3.set_z(1500);} //yaw

  return temp_Vector3;
}
