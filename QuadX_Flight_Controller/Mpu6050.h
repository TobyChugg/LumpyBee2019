/* Toby Chugg 27/08/19
 *  Heavily Inspired by http://www.brokking.net/imu.html
 *  Datasheet: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 *  Register Map: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 */

#ifndef Mpu6050_h
#define Mpu6050_h

#include <Arduino.h>
#include "Vector3.h"






class Mpu6050
{
  
  public: //===========================================================
  
  
  int roll, pitch, yaw; //roll pitch and yaw measured in degrees
  
  //constants
  const int mpu6050_addr_1 = 0x68;
  const int mpu6050_addr_2 = 0x69;

  //functions  
  Mpu6050();
  void initialise(bool b); //if b is true then two mpu6050's are expected. mpu6050_addr = 0x68, 0x69 is 2nd device, but must have AD0 pin to high
  Vector3 get_rpy(); //returns the roll, pitch and yaw (X, Y, Z) of the mpu6050 in a Vector3
  Vector3 get_raw_acc();
  Vector3 get_raw_gyro();
  void read_data();

 
  private: //===========================================================

  //variables
  Vector3 acc_offset, gyro_offset; //offset vectors for calibration on startup
  int temperature;

  Vector3 corrected_acc; //declare temporary Vector3 variables
  Vector3 corrected_gyro;
  Vector3 acc_orientation; //calculated accelerometer angle (degrees)
  Vector3 gyro_orientation; //calculated gyro angle (degrees)
  Vector3 orientation; //final (roll, pitch, yaw) vector, calculated from the acc_ori. + the gyro_ori. (degrees)
  Vector3 raw_acc_data, raw_gyro_data; //raw data vectors
  
  
  float acc_magnitude; //magnitude of accelerometer vector
  
  

  //functions
  void calibrate();
  
};


#endif
