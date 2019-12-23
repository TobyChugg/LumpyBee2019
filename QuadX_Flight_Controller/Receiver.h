/* Toby Chugg - 24/08/19
 * Receiver Library based on YMFC-AL from Joop Brokking
 */

#ifndef Receiver_h
#define Receiver_h

#include "Arduino.h"
#include "Vector3.h"


class Receiver
{
  public:
  Receiver();
  void initialise_interrupts(); //set pins 8 - 11 as interrupts
  int get_receiver_input(int i); //returns RC signal from an interrupt
  int get_receiver_throttle();
  Vector3 get_stick_PWM();
};

//ISR(PCINT0_vect);

#endif
