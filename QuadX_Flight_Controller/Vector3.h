/* Toby Chugg 28/08/19
 *  3D Vector library
 * 
 */

#ifndef Vector3_h
#define Vector3_h

#include "Arduino.h"


class Vector3
{
  public:
  
  
  Vector3();
  Vector3(float _x, float _y, float _z);
  
  void set_vector(float x1, float y1, float z1);
  void set_x(float x1);
  void set_y(float y1);
  void set_z(float z1);
  float get_x();
  float get_y();
  float get_z();
  Vector3 addition(Vector3 v1, Vector3 v2);
  Vector3 subtraction(Vector3 v1, Vector3 v2);
  Vector3 scalar_division(Vector3 v1, int i);
  Vector3 scalar_multiplication(Vector3 v1, float i);
  Vector3 dot_product(Vector3 v1, Vector3 v2);
  float magnitude(Vector3 v1);
  bool equals_zero(Vector3 v1);
  void print_Vector3();
  bool equals_lessthan(Vector3 v1, int i);
  bool equals_greaterthan(Vector3 v1, int i);

  private:
  float x;
  float y;
  float z;

};



#endif
