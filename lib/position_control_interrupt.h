/* 
* position_control_interrupt.h
* Created: 23/03/21 01:15:58 PM
* Author: Bitboot Team
*/ 
#ifndef POSITION_CONTROL_INTERRUPT_H_
#define POSITION_CONTROL_INTERRUPT_H_

#define angle_resolution 4.090	  //resolution used for angle rotation
#define distance_resolution 5.338 //resolution used for distance traversal

unsigned long int ShaftCountLeft ;  //to keep track of left position encoder
unsigned long int ShaftCountRight ; //to keep track of right position encoder
	
// Function to configure left and right encoder pins
void position_encoder_pin_config (void);

// Function to configure external interrupt for encoder pins
void position_encoder_interrupt_config (void);

// Function to rotate Firebird-V by specified degrees
void angle_rotate(unsigned int Degrees);

// Function to move Firebird-V by specified distance
void linear_distance_mm(unsigned int DistanceInMM);

// Function to move forward Firebird-V by specified distance
void forward_mm(unsigned int DistanceInMM);

// Function to move backward Firebird-V by specified distance
void back_mm(unsigned int DistanceInMM);

// Function to rotate Firebird-V left by specified degrees
void left_degrees(unsigned int Degrees);

// Function to rotate Firebird-V right by specified degrees
void right_degrees(unsigned int Degrees);

// Function to rotate Firebird-V left by specified degrees
void soft_left_degrees(unsigned int Degrees);

// Function to rotate Firebird-V left by specified degrees
void soft_left_degrees(unsigned int Degrees);

// Function to rotate Firebird-V left by specified degrees
void soft_left_2_degrees(unsigned int Degrees);

// Function to rotate Firebird-V right by specified degrees
void soft_right_2_degrees(unsigned int Degrees);
#endif