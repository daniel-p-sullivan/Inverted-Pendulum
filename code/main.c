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

#define derivative_sample_size 5//have to do this b/c C is dumb
#define message_sample_size 10000

//baud rate
unsigned int baud = 9600;

//loop counter
static long long loop_count = 0;

//variables to check against the loop count
static long debounce_check = 1000; //number of loops between button checks

//state machine controlling variable
static int previous_button = 0;
static int state = 0;

//variables for the rotary encoder(s) count and angle
volatile long rot_pendulum;
volatile long rot_cart;
double angle;
//static double angular_velocity[derivative_sample_size];
double angular_velocity;
double position;
//static double velocity[derivative_sample_size];
double velocity;
double previous_angle;
double previous_position;

//moving average arrays
double angular_velocity_ma[derivative_sample_size];
double velocity_ma[derivative_sample_size];

//moving average helper variables
double oldest_angular_velocity;
double oldest_velocity;


//motor feedback variable
static double calculated_motor_feedback;

//conversion factors
const float angle_conversion_factor_degrees = 0.15;
const float angle_conversion_factor_rads = 0.00261799;
const float pwm_conversion_factor = 85.25;
const float position_conversion_factor = 0.000025; //converts ticks to radians, then [m] using Rpulley

//PID controller parameters
double setpoint;
double Kp = 200.0;
double Ki = 50.0;
double Kd = 5.0;
double PID_value = 0.0;

//PID error parameters
double error_p;			//error for P control
double error_sum;		//error sum for I control
double derror_dt;		//d(error)/dt for D control
double previous_error;	//previous error for D control calculation

//LQR parameters
double x_desired = 0.0;
double xdot_desired = 0.0;
double theta_desired = 3.14159265;
double thetadot_desired = 0;
double K[4] = {-200, -140.4723, 483.9879, 23.8336}; //volts
	
	//Arduino ATMEGA328P tuning
	//-1000, -481.366, 1135.7, 57.7873
	//-173.2051, -85.3628, 206.4035, 14.4565
	//-200, -140.4723, 483.9879, 23.8336
	//-63.2456, -44.51, 153.6991, 7.7519
	
	
	//-200.0, -140.6674, 485.3084, 24.2954 <-gains for the mega
	
	//OLD ARDUINO MEGA (sucks?)
	//-3162.3, -2475.9, 9508.0, 415.4657
	//-316.2278, -212.0473, 695.8289, 37.3721
	//-316.2278, -212.0523, 695.8465, 37.3776 <good stabilizer, 2 large damping poles
	//-1414.2, -757.928, 1923.3, 173.7042 ds40 mmf0
	//-1414.2, -604.6825,  1199,   125.4942 ds20, mmf0 <-BEST ONE YET, not enough thetadot? theta?
	//-1414.2, -558.762,  1013.7,   118.0016 ds20, mmf0<- best one yet, doesn't value theta enough? maybe thetadot?
	//-447.2136, -293.3809,  724.8837,   74.8125 ds20, mmf0
	//-316.2278, -207.4673,  512.7319,   52.9258 d_s20, mmf 0
	//-100.0000,  -65.6434,  162.5170,   16.7958 - d_s20, mmf 50
double LQR_value;


//for dealing with bad PWM
static int minimum_motor_feedback = 0;

//time variables
double current_time_millis;
double previous_time_millis;
double current_time;
double previous_time;
double dt;
double dt_millis;

//loop control flags
bool start_control_flag = 0;

//timer handling variables
volatile unsigned long timer0_overflow_count = 0;
double millis_per_timer0_tick = 0.064;
double millis_per_overflow = 16.32;
static uint8_t timer0_byte = 0;

//message to be sent
char message[40];

//for sending floats
char* tempsign;
float tempval;
float tempfrac;
int tempint1;
int tempint2;

int sample_index = 0;
int sample_size = 1000;
double t_array[sample_size];
long rot_pend_array[sample_size];
double thetadot_array[sample_size];
long rot_cart_array[sample_size];
double velocity_array[sample_size];
int input_array[sample_size];
	
//*****COMMUNICATION CODE FROM DOCUMENTATION************

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

double millis(){
	//read the low byte before the high byte
	timer0_byte = TCNT0;
	return (double)timer0_overflow_count * millis_per_overflow + timer0_byte * millis_per_timer0_tick;
}

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

double* PID(double* angle){
	
	
	//get the time
	current_time_millis = millis();
	current_time = current_time_millis * 0.001;
	dt = current_time - previous_time;
	previous_time = current_time;
	
	//set the error components for P, I, D
	error_p = *angle - setpoint;
	error_sum += error_p * dt;
	if(dt == 0){
		derror_dt = 0;
	}else{
		derror_dt = (error_p - previous_error) / dt;
	}
	previous_error = error_p;
	
	//section for capping integral windup, uncomment to turn on
	if(error_sum > 100){
		error_sum = 100;
	}else if(error_sum < -100){
		error_sum = -100;
	}
	
	//PID
	
	PID_value = Kp * error_p + Ki * error_sum + Kd * derror_dt;
	return &PID_value;
	
}

double* PI(double* angle, double* dt){
	
	
	//set the error components for P, I, D
	error_p = *angle - theta_desired;
	error_sum += error_p * *dt;
	
	//section for capping integral windup, uncomment to turn on
	if(error_sum > 100){
		error_sum = 100;
	}
	else if(error_sum < -100){
		error_sum = -100;
	}
	previous_error = error_p;
	//PID
	
	PID_value = -1*(Kp * error_p + Ki * error_sum);
	return &PID_value;
	
}

double* P(double* angle){
	error_p = *angle - theta_desired;
	PID_value = -Kp * error_p;
	return &PID_value;
}

double* LQR(double* x, double* xdot, double* theta, double* thetadot){
	LQR_value = -(K[0]*(*x - x_desired) + K[1]*(*xdot - xdot_desired) + K[2]*(*theta - theta_desired) + K[3]*(*thetadot - thetadot_desired));
	return &LQR_value;
	
}

void transmit_data(char* data){
	//for(int i = 0; data[i] != '\0'; i++){
	while(*data!='\0'){
		USART_Transmit(*data++);
	}
}

//not used anymore
double motor_tester(long long lc){
	return (double)(lc % 0x03ff);
}

static inline void debounce(void){
	static uint8_t count = 0;
	static uint8_t button_state = 0;
	uint8_t current_state = PINB & _BV(PB0);
	
	if(current_state != button_state){
		count++;
		if(count >= 4){
			button_state = current_state;
			
			if(button_state == 1){
				state++;
				if(state == 1){ //initial conditions for state1
					setpoint = rot_pendulum * angle_conversion_factor_degrees;
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

//old moving average function
double* ma(double* ma_array){
	double sum = 0;
	for(int i = 0; i < derivative_sample_size; i++){
		sum += ma_array[i];
	}
	sum /= derivative_sample_size;
	return &sum;
}

int main(void)
{
	//-------------SETUP---------------
	
	//****************************************
	//*			USART COMMUNICATION			 *
	//****************************************
	
	//USART_Init(103);
	
	//****************************************
	//*			ROTARY ENCODERS				 *
	//****************************************
	
	//Set up PORTD to accept the rotary encoder data
    PCICR |= _BV(PCIE2);  //turn on interrupt 
	PCMSK2 |= _BV(PCINT21) | _BV(PCINT20) | _BV(PCINT19) | _BV(PCINT18); // enable pins on interrupt
	DDRD = 0b11000000;                   //all pins on Port B are inputs except for PD7/6
	PORTD = 0b00111100;                  //pullups for portD0, portD1, portD2, portD3
	PIND = 0b11000000;					 //release the brakes
	
	
	
	//****************************************
	//*			MOTOR PWM & TIMING			 *
	//****************************************
	
	//Set up PWM
	TCCR1A |= _BV(COM1A1) | _BV(WGM11) | _BV(WGM10); //set the COM1A1 pin to 1 for non-inverting mode, and the WGM0x pins to 1
	TCCR1B |= _BV(CS10);  /*don't set the force compare bits, _BV(CS12) if you want prescaling
							*WGM02 bit, select the clock with [(-101)1024 prescaler] [(-001) no prescaler)]
							*(use the prescaler for lower frequency PWM)*/
	
	OCR1A = 0x03ff;			//10-bit compare register, initially set at maximum
	DDRB = 0b0010010;		//pin 9 (PB1) is PWM, output, PB4 is status LED (red) control
	
	//Motor direction control on PORTC
	DDRC = 0b00000011;
	
	//****************************************
	//*				TIMING					 *
	//****************************************
	PRR &= _BV(PRTIM2); 		//makes sure the PRTIM0 bit is off, PRTIM1 used for PWM, PRTIM2 used for timing?
	//TCCR1A = 0x00;							//make sure COM1A1, COM1A0, COM1B1, COM1B0, COM1C0, COM1C1, WGM11, WGM10 are turned off
	//TCCR1B &= ~(_BV(WGM13) | _BV(WGM12));	//make sure WGM13 and WGM12 are turned off
	//TCCR1B |= _BV(CS12) | _BV(CS10); 		//clock prescaler select (1024 prescaler used => 1 increment = 0.000064 [s] = 0.064 [ms]
											//wgm13, wgm 12 = 0 for normal counting operation
											
	TCCR0B |= _BV(CS02) | _BV(CS00);	//1024bit prescaler
	
	/*
	
	TIMING Section used when control was really slow to work
	
	=========================================================================================================================================
	PRR &= _BV(PRTIM2); 		//makes sure the PRTIM0 bit is off, PRTIM1 used for PWM, PRTIM2 used for timing?
	//TCCR1A = 0x00;							//make sure COM1A1, COM1A0, COM1B1, COM1B0, COM1C0, COM1C1, WGM11, WGM10 are turned off
	//TCCR1B &= ~(_BV(WGM13) | _BV(WGM12));	//make sure WGM13 and WGM12 are turned off
	//TCCR1B |= _BV(CS12) | _BV(CS10); 		//clock prescaler select (1024 prescaler used => 1 increment = 0.000064 [s] = 0.064 [ms]
	//wgm13, wgm 12 = 0 for normal counting operation
	
	TCCR0B |= _BV(CS02) | _BV(CS00);	//1024bit prescaler
	========================================================================================================================================
	*/
	
	
	//VARIABLES
	
	//initial values for rotation and angles
	rot_pendulum = 0;
	angle = 0;
	rot_cart = 0;
	position = 0;
		
	//initial value for motor feedback
	calculated_motor_feedback = 0;
	
	//initial value for timer
	current_time_millis = millis();
	current_time = current_time_millis * 0.001;
	previous_time = current_time;
	
	static uint16_t compare_register_set;
	
	//ENABLE GLOBAL INTERRUPTS
	sei();
	//-------------END SETUP-------------
	state = 0;
    while(1)
    {
		debounce();
		
		PINB &= 0b11101111;
		loop_count = 0;
		while(state == 1 && sample_index < sample_size){
			debounce();
			
			//measurement estimates
			angle = (double)rot_pendulum * angle_conversion_factor_rads;
			position = (double)rot_cart * position_conversion_factor;
			
			//get time information	
			current_time_millis = millis();
			current_time = current_time_millis * 0.001;
			dt = current_time - previous_time;
			
			//moving average for derivative calculations
			oldest_angular_velocity = angular_velocity_ma[loop_count % derivative_sample_size];
			oldest_velocity = velocity_ma[loop_count % derivative_sample_size];
			angular_velocity_ma[loop_count % derivative_sample_size] = (angle - previous_angle) / dt;
			velocity_ma[loop_count % derivative_sample_size] = (position - previous_position) / dt;
			angular_velocity += ((angular_velocity_ma[loop_count % derivative_sample_size] - oldest_angular_velocity) / derivative_sample_size);
			velocity += ((velocity_ma[loop_count % derivative_sample_size] - oldest_velocity) / derivative_sample_size);
			
			//save old states
			previous_time = current_time;
			previous_position = position;
			previous_angle = angle;	
				
			//P Controller
			//compare_register_set = motor_feedback_control(P(&angle));
				
			//PI Controller
			//compare_register_set = motor_feedback_control(PI(&angle, &dt));
				
			//LQR Controller
			compare_register_set = motor_feedback_control(LQR(&position, &velocity, &angle, &angular_velocity));
			OCR1A = compare_register_set;
				
			//t_array[sample_index] = current_time;
			//rot_cart_array[sample_index] = angle;
			//thetadot_array[sample_index] = angular_velocity;
			//rot_pend_array[sample_index] = position;
			//velocity_array[sample_index] = velocity;
			//input_array[sample_index] = compare_register_set;
			sample_index++;
			//}
			loop_count++;
		}
		loop_count = 0;
		sample_index = 0;
		while(state == 2){
			//STATE 2:  SEND BACK DATA
			
			debounce();
			
			//for(int i = 0; i < sample_size; i++){
				//tempval = t_array[i];
				//tempint1 = tempval;				//get integer
				//tempfrac = tempval - tempint1;	//get the fraction
				//tempint2 = trunc(tempfrac*10000); //turn into integer
				//sprintf(message, "t:%d.%d", tempint1, tempint2);
				//transmit_data(message);
				//
				//tempsign = (thetadot_array[i] < 0) ? "-" : "";
				//tempval = (thetadot_array[i] < 0) ? -thetadot_array[i] : thetadot_array[i];
				//tempint1 = tempval;				//get integer
				//tempfrac = tempval - tempint1;	//get the fraction
				//tempint2 = trunc(tempfrac*10000); //turn into integer
				//
				//sprintf(message, "th:%dtd:%s%d.%d", rot_pend_array[i], tempsign, tempint1, tempint2);
				//transmit_data(message);
				//
				//tempsign = (velocity_array[i] < 0) ? "-" : "";
				//tempval = (velocity_array[i] < 0) ? -velocity_array[i] : velocity_array[i];
				//tempint1 = tempval;				//get integer
				//tempfrac = tempval - tempint1;	//get the fraction
				//tempint2 = trunc(tempfrac*10000); //turn into integer
				//
				//sprintf(message, "x:%dv:%s%d.%du:%d", rot_cart_array[i], tempsign, tempint1, tempint2, input_array[i]);
				//transmit_data(message);
			//}
			
			state = 3;
		}
		loop_count++;
		
    }
}