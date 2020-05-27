
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include "hardware.h"

//motor feedback variable
static double calculated_motor_feedback = 0;

//minimum allowable motor feedback
//This variable to be adjusted for the controller to overcome the friction
//TODO:  Calibrate this variable by increasing the PWM steadily until the cart moves?
static int minimum_motor_feedback = 0;

//for converting volt output of control calculation to PWM
//1023 / 12 [V] = 85.25
//Therefore, multiply this constant by the calculated voltage from the controller to scale the PWM correctly.
static const float pwm_conversion_factor = 85.25;

int motor_feedback_control(double* cmf){
	
	calculated_motor_feedback = *cmf * pwm_conversion_factor;   //cmf comes in in units of [V], multiply by
	//the pwm_conversion_factor to put it in units of PWM
	//set plant limits to PWM signal
	if(calculated_motor_feedback < minimum_motor_feedback && calculated_motor_feedback > 0){
		
		calculated_motor_feedback = minimum_motor_feedback;
		PORTC = 0b00000000 | _BV(PC0); //turn on PH0, turn off PH1
		
		}else if(calculated_motor_feedback > -minimum_motor_feedback && calculated_motor_feedback < 0){
		
		calculated_motor_feedback = -minimum_motor_feedback;
		PORTC = 0b00000000 | _BV(PC1);
		
		}else if(calculated_motor_feedback > 1023){
		
		calculated_motor_feedback = 1023;
		PORTC = 0b00000000 | _BV(PC0);
		
		}else if (calculated_motor_feedback < -1023){
		
		calculated_motor_feedback = -1023;
		PORTC = 0b00000000 | _BV(PC1);
		
		}else if (calculated_motor_feedback == 0){
		
		PORTC = 0b00000000;							//turn off PH0 and PH1
		
	}
	
	return abs(calculated_motor_feedback);		//set the compare register
}

void rotary_encoders_setup(){
	//Set up PORTD to accept the rotary encoder data
	PCICR |= _BV(PCIE2);  //turn on interrupt
	PCMSK2 |= _BV(PCINT21) | _BV(PCINT20) | _BV(PCINT19) | _BV(PCINT18); // enable pins on interrupt
	DDRD = 0b11000000;                   //all pins on Port B are inputs except for PD7/6
	PORTD = 0b00111100;                  //pullups for portD0, portD1, portD2, portD3
	PIND = 0b11000000;					 //release the brakes
}

void motor_setup(){
	//Set up PWM
	TCCR1A |= _BV(COM1A1) | _BV(WGM11) | _BV(WGM10); //set the COM1A1 pin to 1 for non-inverting mode, and the WGM0x pins to 1
	TCCR1B |= _BV(CS10);  /*don't set the force compare bits, _BV(CS12) if you want prescaling
							*WGM02 bit, select the clock with [(-101)1024 prescaler] [(-001) no prescaler)]
							*(use the prescaler for lower frequency PWM)*/
	
	OCR1A = 0x03ff;			//10-bit compare register, initially set at maximum
	DDRB = 0b0010010;		//pin 9 (PB1) is PWM, output, PB4 is status LED (red) control
	
	//Motor direction control on PORTC
	DDRC = 0b00000011;
}

void timer_setup(){
	PRR &= _BV(PRTIM2); 		//makes sure the PRTIM2 bit is off, timer1 used for PWM and timer0 used for millis()
	TCCR0B |= _BV(CS02) | _BV(CS00);	//1024bit prescaler
	//clock prescaler select (1024 prescaler used => 1 increment = 0.000064 [s] = 0.064 [ms]
}