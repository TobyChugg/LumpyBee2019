#ifndef PID_h
#define PID_h

#include "Arduino.h"
#include "Vector3.h"


class PID
{
  public:
  
  PID();
  void set_coefficients(Vector3 _roll_coe, Vector3 _pitch_coe, Vector3 _yaw_coe);
  Vector3 geterror();
  Vector3 calculate_error(Vector3 setPoint, Vector3 orientation);
  Vector3 calculate_PID_signal(Vector3 receiver_signal_degrees, Vector3 orientation);
  
};

#endif
