/* Toby Chugg 27/08/19 test
 *  Heavily Inspired by http://www.brokking.net/imu.html
 *  Datasheet: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 *  Register Map: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 */

#include <Arduino.h>
#include "Mpu6050.h"
#include "Vector3.h"
#include <Wire.h>

Vector3 drift_correction(0, 0, 0);
bool flag = false;

Mpu6050::Mpu6050()
{
  
}


void Mpu6050::initialise(bool b)
{

  if(!b)
  {
    //Activate the MPU-6050
    Wire.beginTransmission(mpu6050_addr_1);                              //Start communicating with the MPU-6050
    Wire.write(0x6B);                                                    //Send the requested starting register
    Wire.write(0x00);                                                    //Set the requested starting register
    Wire.endTransmission();                                              //End the transmission
    //Configure the accelerometer (+/-8g)
    Wire.beginTransmission(mpu6050_addr_1);                              //Start communicating with the MPU-6050
    Wire.write(0x1C);                                                    //Send the requested starting register
    Wire.write(0x10);                                                    //Set the requested starting register
    Wire.endTransmission();                                              //End the transmission
    //Configure the gyro (FS_sel = 2)
    Wire.beginTransmission(mpu6050_addr_1);                              //Start communicating with the MPU-6050
    Wire.write(0x1B);                                                    //Send the requested starting register
    Wire.write(0x08);                                                    //Set the requested starting register
    Wire.endTransmission();                                              //End the transmission

  }
  calibrate();
}

void Mpu6050::read_data()
{
  Wire.beginTransmission(mpu6050_addr_1);                             //start I2C communication
  Wire.write(0x3B);                                                   //register 0x3B which corrosponds to ACCEL_XOUT_H on mpu6050 register map
  Wire.endTransmission();
  Wire.requestFrom(mpu6050_addr_1, 14);                               //request from addresses (14 in total)

  while(Wire.available() < 14);
  raw_acc_data.set_y(Wire.read() << 8 | Wire.read());                  //Reading registers 0x3B and 0x3C, combining them using bitwise OR to get final data //swapped x and y to account for actual drone
  raw_acc_data.set_x(Wire.read() << 8 | Wire.read());
  raw_acc_data.set_z(Wire.read() << 8 | Wire.read());
  temperature = Wire.read() << 8 | Wire.read();
  raw_gyro_data.set_y(Wire.read() << 8 | Wire.read());
  raw_gyro_data.set_x(Wire.read() << 8 | Wire.read());
  raw_gyro_data.set_z(Wire.read() << 8 | Wire.read());

}

void Mpu6050::calibrate()
{
  Vector3 gyro_temp; //temporary variables to sum data per loop
  Vector3 acc_temp;
  
  for(int i = 0; i < 2000; i++)
  {
    read_data();

    gyro_temp = gyro_temp.addition(gyro_temp, raw_gyro_data); //use vector addition function to increment *_temp by the new raw_*_data
    acc_temp = acc_temp.addition(acc_temp, raw_acc_data);
  }
  
  acc_offset = acc_temp.scalar_division(acc_temp, 2000); //set the final calculated offsets as the average of all the values
  gyro_offset = gyro_temp.scalar_division(gyro_temp, 2000);
}

Vector3 Mpu6050::get_rpy() //get RollPitchYaw Vector
{
  read_data();  //update raw global mpu6050 data

  corrected_acc = corrected_acc.subtraction(raw_acc_data, acc_offset); //subtract calculated offset from the raw data
  corrected_gyro = corrected_gyro.subtraction(raw_gyro_data, gyro_offset);
  
  acc_magnitude = raw_acc_data.magnitude(raw_acc_data);
  
  //=========================== Calculate gyroscope vector angles

  //NOTE: x and y gyro values are inverted                  vvHEREvv
  gyro_orientation.set_y((gyro_orientation.get_y() + (corrected_gyro.get_x()) * 0.0000611)); //0.0000611 is constant to account for how much time has passed (the refresh rate of the loop) and the raw data to degrees constant (65.5 - see register map)
  gyro_orientation.set_x((gyro_orientation.get_x() + (corrected_gyro.get_y()) * 0.0000611));
  gyro_orientation.set_z((gyro_orientation.get_z() + (corrected_gyro.get_z()) * 0.0000611));

  gyro_orientation.set_y(gyro_orientation.get_y() - (gyro_orientation.get_x() * sin(gyro_orientation.get_z() * 0.000001066)));                    //0.000001066 is the same as the previous constant but including the radians to degrees functionality
  gyro_orientation.set_x(gyro_orientation.get_x() + (gyro_orientation.get_y() * sin(gyro_orientation.get_z() * 0.000001066)));                  //to account for the acceleration not being present when rotating roll to pitch

  //=========================== Calculate accelerometer vector angles

  if(abs(corrected_acc.get_x()) < acc_magnitude) {acc_orientation.set_x(asin((float)(corrected_acc.get_x()/acc_magnitude)) *57.296);}
  if(abs(corrected_acc.get_y()) < acc_magnitude) {acc_orientation.set_y(asin((float)(corrected_acc.get_y()/acc_magnitude)) *-57.296);}
  if(abs(corrected_acc.get_z()) < acc_magnitude) {acc_orientation.set_z(asin((float)(corrected_acc.get_z()/acc_magnitude)) *57.296);}


  gyro_orientation = gyro_orientation.addition(gyro_orientation.scalar_multiplication(gyro_orientation, 0.996), gyro_orientation.scalar_multiplication(acc_orientation, 0.004)); //complementary filter

  orientation = gyro_orientation;


  return orientation;
  
}

Vector3 Mpu6050::get_raw_acc()
{
  return raw_acc_data;
}

Vector3 Mpu6050::get_raw_gyro()
{
  return raw_gyro_data;
}
