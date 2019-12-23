/* Toby Chugg 16/11/19
   Heavily Inspired by http://www.brokking.net/ymfc-al_main.html
   Datasheet: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
   Register Map: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

   This code is 
*/

#include "Mpu6050.h"
#include "PID.h"
#include "Receiver.h"
#include "Vector3.h"
#include "MotorController.h"
#include <Wire.h>

//////////////========
bool debugging = false; //turn this on for serial output
bool motors_on = true;
//////////////=========


//PID constants
Vector3 roll_coefficients(4, 0.03, 60); //(P, I, D)
Vector3 pitch_coefficients(4, 0.03, 60);
Vector3 yaw_coefficients(6, 0, 0);

//objects
Mpu6050 mpu6050;
Receiver receiver;
PID pid_controller;
MotorController motorcontroller;

//program timers
unsigned long startMicros_1, currentMicros_1, startMicros_2, currentMicros_2; 

//======================

Vector3 orientation; //drone orientation (Vector3 with a roll, pitch and yaw attribute)
Vector3 stickdegrees(0,0,0);
Vector3 pidcontrol;
int throttle;


void setup() {

  if(debugging){Serial.begin(57600);} 
  Wire.begin(); //initialise I2C line
  TWBR = 12;  //Set the I2C clock speed to 400kHz.
  
  mpu6050.initialise(false); //parameter sets if there are one (false) or two (true) mpu6050's (not implemented yet)
  receiver.initialise_interrupts(); //initialise receiver
  pid_controller.set_coefficients(roll_coefficients, pitch_coefficients, yaw_coefficients); //set PID coefficients

  DDRD |= B11110000; //Configure digital port 4, 5, 6 and 7 as output.

  startMicros_1 = micros();  //start timers
  startMicros_2 = micros();
}

void loop() {
  
  currentMicros_1 = micros();
  if(debugging){currentMicros_2 = micros();} //debugging
  
  if (currentMicros_1 - startMicros_1 >= 4000)
  {
    orientation = mpu6050.get_rpy(); //get roll pitch yaw
    stickdegrees = receiver.get_stick_PWM(); //get receiver PWM signal
    throttle = receiver.get_receiver_throttle(); //get throttle
    if(debugging && !motors_on){throttle = 1500; stickdegrees.set_vector(1500, 1500, 1500);}
    pidcontrol = pid_controller.calculate_PID_signal(stickdegrees, orientation); //calculate PID values
    motorcontroller.calculate_esc_PWM(throttle, pidcontrol); //convert to ESC PWM values, accessed by get_esc_PWM_FL() etc..

    //write to motors
    if(motors_on)
    {
      motorcontroller.write_motor_PWM();
    }
    
    startMicros_1 = currentMicros_1; //reset timer
  }


  //DEBUGGING LOOP ========================================================================================
  if(debugging)
  {
    if (currentMicros_2 - startMicros_2 >= 50000)
   {    
    //Serial.println(throttle);
       //stickdegrees.print_Vector3();
       //Serial.print("X: "); Serial.print(orientation.get_x()); Serial.print(" "); Serial.print("Y: "); Serial.println(orientation.get_y());
      
      //orientation.print_Vector3();
      //pidcontrol.print_Vector3();
      Serial.print("FL: "); Serial.print(" "); Serial.print(motorcontroller.get_esc_PWM_FL()); Serial.print(" ");  Serial.print("FR: "); Serial.print(" "); Serial.print(motorcontroller.get_esc_PWM_FR()); Serial.print(" ");  Serial.print("BL: "); Serial.print(" "); Serial.print(motorcontroller.get_esc_PWM_BL()); Serial.print(" ");  Serial.print("BR: "); Serial.print(" "); Serial.println(motorcontroller.get_esc_PWM_BR());

      startMicros_2 = currentMicros_2; //reset timer
   }
  }
}
