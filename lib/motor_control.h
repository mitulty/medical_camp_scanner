/*
* motor_control.h
* Created: 23/03/21 01:15:58 PM
* Author: Bitboot Team
*/ 

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

// Function to configure motor pins
void motors_pin_config(void);

// Function to configure left and right channel pins of the L293D Motor Driver IC for PWM
void pwm_pin_config(void);

// Function to make Firebird-V move forward
void forward (void); //both wheels forward

// Function to make Firebird-V move backward
void back (void); //both wheels backward

// Function to make Firebird-V rotate left
void left (void); //Left wheel backward, Right wheel forward

// Function to make Firebird-V rotate right
void right (void); //Left wheel forward, Right wheel backward

// Function to make Firebird-V rotate soft left
void soft_left (void); //Left wheel stationary, Right wheel forward

// Function to make Firebird-V rotate soft right
void soft_right (void); //Left wheel forward, Right wheel is stationary

// Function to make Firebird-V rotate backward left
void backward_left (void); //Left wheel backward, right wheel stationary

// Function to make Firebird-V rotate backward right
void backward_right (void); //Left wheel stationary, Right wheel backward

// Function to make Firebird-V stop
void stop (void); // Both wheel stationary

// Timer Init 
void timer_pwm_init();

//Function to control the speed of both the motors of Firebird-V
void velocity (unsigned char left_motor, unsigned char right_motor);

// Initializes the setup by configuring all the required devices
int motor_init_setup(void);

#endif
