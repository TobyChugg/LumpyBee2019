#include <Arduino.h>
#include "Vector3.h"
#include "PID.h"

//PID coefficients for roll, pitch and yaw
Vector3 roll_PID_coefficients(0, 0, 0); //defaults
Vector3 pitch_PID_coefficients(0, 0, 0);
Vector3 yaw_PID_coefficients(0, 0, 0);
Vector3 prev_orientation(0, 0, 0); //for yaw error calculation

Vector3 MAX_ANGLE(400, 400, 400);


Vector3 error; //x, y, z corrospond to roll, pitch and yaw
Vector3 prev_error; //used for D component 
Vector3 mem_error; //used for incrementing the I component for roll, pitch and yaw

PID::PID()
{
}

void PID::set_coefficients(Vector3 _roll_coe, Vector3 _pitch_coe, Vector3 _yaw_coe)
{
   roll_PID_coefficients = _roll_coe;
   pitch_PID_coefficients = _pitch_coe;
   yaw_PID_coefficients = _yaw_coe;
}

Vector3 PID::calculate_error(Vector3 setPoint, Vector3 orientation)
{
  Vector3 temp_Vector3;
  
  temp_Vector3 = temp_Vector3.subtraction(setPoint, orientation);

  return temp_Vector3;
}

Vector3 PID::calculate_PID_signal(Vector3 receiver_signal_PWM, Vector3 orientation)
{ 
  //mapping the receiver to a min and max desired degrees
  int x1 = map(receiver_signal_PWM.get_x(), 1000, 2000, -30, 30);
  int y1 = map(receiver_signal_PWM.get_y(), 1000, 2000, -30, 30);
  int z1 = map(receiver_signal_PWM.get_z(), 1000, 2000, -30, 30); 
  
  Vector3 set_angle(x1, y1, z1*-1) ; //inverted
  Vector3 temp_PID_signal;  

  error = error.subtraction(orientation, set_angle);

  if(set_angle.get_z() == 0) //if the yaw stick is neutral
  {
    error.set_z((prev_orientation.get_z() - orientation.get_z())*50); //calculate the error based on the change in yaw and multiply to a usable number
  }
  else
  {
    error.set_z(set_angle.get_z()); //control the yaw soley with the stick input
  }
  
  prev_orientation = orientation; //used for calculating change in yaw 
  
  
  mem_error.set_x(mem_error.get_x() + (roll_PID_coefficients.get_y() * error.get_x()));
  mem_error.set_y(mem_error.get_y() + (pitch_PID_coefficients.get_y() * error.get_y()));
  mem_error.set_z(mem_error.get_z() + (yaw_PID_coefficients.get_y() * error.get_z()));

  

  //roll
  if(mem_error.get_x() > MAX_ANGLE.get_x()){mem_error.set_x(MAX_ANGLE.get_x());} 
  else if(mem_error.get_x() < MAX_ANGLE.get_x()*-1){mem_error.set_x(MAX_ANGLE.get_x()*-1);}
  temp_PID_signal.set_x(  (roll_PID_coefficients.get_x() * error.get_x())  +  (mem_error.get_x())  +  (roll_PID_coefficients.get_z()*(error.get_x() - prev_error.get_x()))  );
  if(temp_PID_signal.get_x() >= MAX_ANGLE.get_x()) { temp_PID_signal.set_x(MAX_ANGLE.get_x());}
  if(temp_PID_signal.get_x() <= MAX_ANGLE.get_x() *-1) { temp_PID_signal.set_x(MAX_ANGLE.get_x() *-1);}

  //pitch
  if(mem_error.get_y() > MAX_ANGLE.get_y()){mem_error.set_y(MAX_ANGLE.get_y());} 
  else if(mem_error.get_y() < MAX_ANGLE.get_y()*-1){mem_error.set_y(MAX_ANGLE.get_y()*-1);}
  temp_PID_signal.set_y(  (pitch_PID_coefficients.get_x() * error.get_y())  +  (mem_error.get_y())  +  (pitch_PID_coefficients.get_z()*(error.get_y() - prev_error.get_y())));
  if(temp_PID_signal.get_y() >= MAX_ANGLE.get_y()) { temp_PID_signal.set_y(MAX_ANGLE.get_y());}
  if(temp_PID_signal.get_y() <= MAX_ANGLE.get_y()*-1) { temp_PID_signal.set_y(MAX_ANGLE.get_y()*-1);}

  //yaw
  
  if(mem_error.get_z() > MAX_ANGLE.get_z()){mem_error.set_z(MAX_ANGLE.get_z());} 
  else if(mem_error.get_z() < MAX_ANGLE.get_z()*-1){mem_error.set_z(MAX_ANGLE.get_z()*-1);}
  temp_PID_signal.set_z(  ((yaw_PID_coefficients.get_x() * error.get_z())  +  (mem_error.get_z())  +  (yaw_PID_coefficients.get_z()*(error.get_z() - prev_error.get_z()))) );
  if(temp_PID_signal.get_z() >= MAX_ANGLE.get_z()) { temp_PID_signal.set_z(MAX_ANGLE.get_z());}
  if(temp_PID_signal.get_z() <= MAX_ANGLE.get_z()*-1) { temp_PID_signal.set_z(MAX_ANGLE.get_z()*-1);}
  
  
  
  
  prev_error.set_x(error.get_x());
  prev_error.set_y(error.get_y());
  prev_error.set_z(error.get_z());

  return temp_PID_signal;
}
