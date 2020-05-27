/*
 * main.c for Inverted Pendulum project
 *
 * Created: 9/11/2015
 * Author : Daniel Sullivan
 */ 

#define F_CPU 16000000L // Specify oscillator frequency
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "control_functions.h"
#include "hardware.h"

#define derivative_sample_size 5//have to do this b/c C is dumb
#define message_sample_size 10000
#define sample_size 50 //sample size for data acquisition

//baud rate
unsigned int baud = 9600;

//loop counter
static long loop_count = 0;

//variables for the rotary encoder(s) count and angle
volatile long rot_pendulum = 0;
volatile long rot_cart = 0;
double angle = 0;
double angular_velocity;
double position = 0;
double velocity = 0;
double previous_angle;
double previous_position;

//moving average arrays
double angular_velocity_ma[derivative_sample_size];
double velocity_ma[derivative_sample_size];

//moving average helper variables
double oldest_angular_velocity;
double oldest_velocity;

//time variables
double current_time_millis;
double previous_time_millis;
double current_time;
double previous_time;
double dt;
double dt_millis;

//timer handling variables
volatile unsigned long timer0_overflow_count = 0;
long double millis_per_timer0_tick = 0.064;
double millis_per_overflow = 16.32;
static uint8_t timer0_byte = 0;

//other conversion factors
const float angle_conversion_factor_degrees = 0.15;		//rotary encoders have 600 pulses per rotation
const float angle_conversion_factor_rads = 0.00261799;	//rotary encoders have 600 pulses per rotation
const float position_conversion_factor = 0.000025; //converts pulses to radians, then [m] using Rpulley

//setpoint variables that are set during debounce()
double x_desired = 0.0;
double xdot_desired = 0.0;
double theta_desired = 3.14159265;
double thetadot_desired = 0;

//error variables
double x_error = 0;
double xdot_error = 0;
double theta_error = 0;
double thetadot_error = 0;

//state machine controlling variable
int state = 0;

//stores output from motor driver to eventually store into PWM compare register
static uint16_t compare_register_set;

//message to be sent
char message[40];
char outstr[9];

//for sending floats
char* tempsign;
float tempval;
float tempfrac;
int tempint1;
int tempint2;

int sample_index = 0;
int count_until_sample = 50; //take sample every 50 runs of the control loop
double t_array[sample_size];
long rot_pend_array[sample_size];
double thetadot_array[sample_size];
long rot_cart_array[sample_size];
double velocity_array[sample_size];
int input_array[sample_size];
	
void USART_Init(unsigned int ubrr){
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void USART_Transmit(unsigned char data){
	/* Wait for empty transmit buffer */
	while (!(UCSR0A & (1<<UDRE0)));
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

unsigned char USART_Receive(void)
{
	/* Wait for data to be received */
	while (!(UCSR0A & (1<<RXC0)));
	/* Get and return received data from buffer */
	return UDR0;
}

ISR(PCINT2_vect){
	
	static const int8_t rot_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
	static uint8_t AB[2] = {0x03, 0x03};  //for storing old and new values of pins
	uint8_t t = PIND;                      //read Port B
	t = (t >> 2) & 0b00001111;								//used to keep dig pin 0/1 open for TX/RX in futue
	
	//check for rotary state change
	AB[0] <<= 2;       //save previous state (by bit shifting it left)
	AB[0] |= t & 0x03; //add the new state to the 2 LSB of AB (by making this the 2 LSB)
	rot_pendulum += rot_states[AB[0] & 0x0f]; //increment or decrement the rotation
	
	AB[1] <<= 2;		//save previous state
	AB[1] |= (t >> 2) & 0x03; //shift t to the right two (move 2nd r.e. inputs to the LSB) 
	rot_cart += rot_states[AB[1] & 0x0f]; //increment or decrement
}

ISR(TIMER0_OVF_vect){
	
	timer0_overflow_count++;

}

long double millis(){
	//read the low byte before the high byte
	timer0_byte = TCNT0;
	return (long double)timer0_overflow_count * millis_per_overflow + timer0_byte * millis_per_timer0_tick;
}

void transmit_data(char* data){
	//for(int i = 0; data[i] != '\0'; i++){
	while(*data!='\0'){
		USART_Transmit(*data++);
	}
}

static inline void debounce(void){
	//Taken from some stackoverflow thread, forgot to get link
	static uint8_t count = 0;
	static uint8_t button_state = 0;
	uint8_t current_state = PINB & _BV(PB0);
	
	if(current_state != button_state){ //If the current state is different than the old state
		
		count++;					  //count number of times there is a difference
		
		if(count >= 4){				//if state is different > 4 times in a row
			button_state = current_state;	//set the state
			
			if(button_state == 1){			//if the button is now on
				state++;					//increment the "state" variable (will be used for state machine transition of overall system)
				if(state == 1){ //initial conditions for state1
					
					theta_desired = rot_pendulum * angle_conversion_factor_rads;
					x_desired = rot_cart * position_conversion_factor;
				}
				else if(state == 2){
					//initial conditions for state 2
				}
				
			}
			count = 0;
		}
	}
}

static inline void moving_average_derivative_calculation(){
	//moving average for derivative calculations
	oldest_angular_velocity = angular_velocity_ma[loop_count % derivative_sample_size];
	oldest_velocity = velocity_ma[loop_count % derivative_sample_size];
	angular_velocity_ma[loop_count % derivative_sample_size] = (angle - previous_angle) / dt;
	velocity_ma[loop_count % derivative_sample_size] = (position - previous_position) / dt;
	angular_velocity += ((angular_velocity_ma[loop_count % derivative_sample_size] - oldest_angular_velocity) / derivative_sample_size);
	velocity += ((velocity_ma[loop_count % derivative_sample_size] - oldest_velocity) / derivative_sample_size);
}

static inline void get_time_information(){
	//get time information
	current_time_millis = millis();
	current_time = current_time_millis * 0.001;
	dt = current_time - previous_time;
}

static inline void calculate_error(){
	//calculate errors
	x_error = position - x_desired;
	xdot_error = velocity - xdot_desired;
	theta_error = angle - theta_desired;
	thetadot_error = angular_velocity - thetadot_desired;
}

static inline void save_previous_states(){
	previous_position = position;
	previous_angle = angle;
	previous_time = current_time;
}

static inline void take_sample(){
	//only take sample every count_until_sample times
	if(loop_count % count_until_sample == 0){
		t_array[sample_index] = current_time;
		rot_cart_array[sample_index] = angle;
		thetadot_array[sample_index] = angular_velocity;
		rot_pend_array[sample_index] = position;
		velocity_array[sample_index] = velocity;
		input_array[sample_index] = compare_register_set;
		sample_index++;
	}
}

int main(void)
{
	//-------------SETUP---------------
	
	//Commented out this line as STATE 2 is currently commented out and data is not being sent back to the PC	
	//USART_Init(103);
	
	rotary_encoders_setup();
	motor_setup();
	timer_setup();
	
	//ENABLE GLOBAL INTERRUPTS
	sei();
	
	//-------------END SETUP-------------
    while(1)
    {
		//debounce the state transition button
		debounce();
		
		//reset the loop count
		loop_count = 0;
		
		while(state == 1 && sample_index < sample_size){
			
			//continue to debounce the state transition button
			debounce();
			
			//measurement estimates
			angle = (double)rot_pendulum * angle_conversion_factor_rads;
			position = (double)rot_cart * position_conversion_factor;
			
			get_time_information();
			
			moving_average_derivative_calculation();
			
			save_previous_states();
			
			calculate_error();
					
			//CONTROLLERS
				
			//P Controller
			//compare_register_set = motor_feedback_control(P(&angle));
				
			//PI Controller
			//compare_register_set = motor_feedback_control(PI(&angle, &dt));
				
			//LQR Controller
			compare_register_set = motor_feedback_control(LQR(&x_error, &xdot_error, &theta_error, &thetadot_error));
			OCR1A = compare_register_set;
			
			//commented out this line as currently data is not being sent back to the PC
			//take_sample();
			
			loop_count++;
		}
		//reset the loop count
		loop_count = 0;
		
		//reset the sample index
		sample_index = 0;
		while(state == 2){
			
			//STATE 2 is commented out as USART data sendback is currently poor
			
			//STATE 2:  SEND BACK DATA
			
			//debounce the state transition button
			//debounce();
			
			/*for(int i = 0; i < sample_size; i++){

				dtostrf(t_array[i], 9, 4, outstr);
				sprintf(message, "t:%s", outstr);
				transmit_data(message);
				
				dtostrf(thetadot_array[i], 9, 4, outstr);
				sprintf(message, "th:%ldtd:%s", rot_pend_array[i], outstr);
				transmit_data(message);
				
				dtostrf(velocity_array[i], 9, 4, outstr);
				sprintf(message, "x:%ldv:%su:%d", rot_cart_array[i], outstr, input_array[i]);
				transmit_data(message);
			}*/
			
			//once serial comms is done, move to next state
			state = 3;
		}
		loop_count++;
		
    }
}