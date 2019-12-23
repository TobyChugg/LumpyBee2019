#include <Arduino.h>
#include "Vector3.h"

//instance variables



Vector3::Vector3()
{
  x = 0;
  y = 0;
  z = 0;
}


Vector3::Vector3(float _x, float _y, float _z)
{
  x = _x;
  y = _y;
  z = _z;
}
  
void Vector3::set_vector(float x1, float y1, float z1)
{
  x = x1;
  y = y1;
  z = z1;
}

void Vector3::set_x(float x1)
{
  x = x1;
}

void Vector3::set_y(float y1)
{
  y = y1;
}

void Vector3::set_z(float z1)
{
  z = z1;
}

float Vector3::get_x()
{
  return x;
}

float Vector3::get_y()
{
  return y;
}

float Vector3::get_z()
{
  return z;
}

Vector3 Vector3::addition(Vector3 v1, Vector3 v2)
{
  Vector3 tempVector;
  
  tempVector.set_x(v1.get_x() + v2.get_x());
  tempVector.set_y(v1.get_y() + v2.get_y());
  tempVector.set_z(v1.get_z() + v2.get_z());

  return tempVector;
}

Vector3 Vector3::subtraction(Vector3 v1, Vector3 v2)
{
  Vector3 tempVector;
  
  tempVector.set_x(v1.get_x() - v2.get_x());
  tempVector.set_y(v1.get_y() - v2.get_y());
  tempVector.set_z(v1.get_z() - v2.get_z());

  return tempVector;
}

Vector3 Vector3::scalar_division(Vector3 v1, int i)
{
  Vector3 tempVector;

  tempVector.set_x(v1.get_x()/i);
  tempVector.set_y(v1.get_y()/i);
  tempVector.set_z(v1.get_z()/i);

  return tempVector;
}

Vector3 Vector3::scalar_multiplication(Vector3 v1, float i)
{
  Vector3 tempVector;

  tempVector.set_x(v1.get_x()*i);
  tempVector.set_y(v1.get_y()*i);
  tempVector.set_z(v1.get_z()*i);

  return tempVector;
}

float Vector3::magnitude(Vector3 v1)
{
  float f; //temporary variable

  f = sqrt((v1.get_x() * v1.get_x()) + (v1.get_y() * v1.get_y()) + (v1.get_z() * v1.get_z())); //pythagoras
 
 

  return f;
}

Vector3 dot_product(Vector3 v1, Vector3 v2)
{
  Vector3 tempVector;

  tempVector.set_x(v1.get_x() * v2.get_x());
  tempVector.set_y(v1.get_y() * v2.get_y());
  tempVector.set_z(v1.get_z() * v2.get_z());

  return tempVector;
  
}

bool Vector3::equals_zero(Vector3 v1)
{
  if(v1.get_x() == 0 || v1.get_y() == 0 || v1.get_z() == 0)
  {
    return true;
  }
}

bool Vector3::equals_lessthan(Vector3 v1, int i)
{
  if(v1.get_x() <= i || v1.get_y() <= i || v1.get_z() <= i)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool Vector3::equals_greaterthan(Vector3 v1, int i)
{
  if(v1.get_x() >= i || v1.get_y() >= i || v1.get_z() >= i)
  {
    return true;
  }
  else
  {
    return false;
  }
  
}

void Vector3::print_Vector3()
{
  //if(x > -0.99 && x < 0.99){x = 0;}
  //if(y > -0.99 && y < 0.99){y = 0;}
  //if(z > -0.99 && z < 0.99){z = 0;}
  Serial.print("x: "); 
  Serial.print(x); //Serial.print(x, 0) to remove decimals
  Serial.print(" y: "); 
  Serial.print(y); 
  Serial.print(" z: "); 
  Serial.println(z); 
}
