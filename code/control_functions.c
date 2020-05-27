
#include "control_functions.h"

//PID controller parameters
static double Kp = 200.0;
static double Ki = 50.0;
static double Kd = 5.0;

//PID error parameters
static double error_p = 0;			//error for P control
static double error_sum = 0;		//error sum for I control
static double derror_dt = 0;		//d(error)/dt for D control
static double previous_error = 0;	//previous error for D control calculation
static double PID_value = 0;		//store PID calculation

//LQR controller parameters
static double LQR_value = 0;
static double K[4] = {-200, -140.4723, 483.9879, 23.8336}; //gain array for feedback
	
	//PAST VALUES FOR K:
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
	

double* PID(double* theta_error, double* dt){
	
	//set the error components for P, I, D
	error_sum += *theta_error * *dt;
	if(*dt == 0){
		derror_dt = 0;
		}
	else{
		derror_dt = (*theta_error - previous_error) / *dt;
	}
	previous_error = error_p;
	
	//section for capping integral windup, uncomment to turn on
	if(error_sum > 100){
		error_sum = 100;
	}
	else if(error_sum < -100){
		error_sum = -100;
	}
	
	//PID
	
	PID_value = Kp * *theta_error + Ki * error_sum + Kd * derror_dt;
	return &PID_value;
	
}

double* PI(double* theta_error, double* dt){
	
	
	//set the error components for P, I, D
	error_sum += *theta_error * *dt;
	
	//section for capping integral windup, uncomment to turn on
	if(error_sum > 100){
		error_sum = 100;
	}
	else if(error_sum < -100){
		error_sum = -100;
	}
	
	//PID
	PID_value = -1*(Kp * *theta_error + Ki * error_sum);
	return &PID_value;
	
}

double* P(double* theta_error){
	PID_value = -Kp * *theta_error;
	return &PID_value;
}

double* LQR(double* x_error, double* xdot_error, double* theta_error, double* thetadot_error){
	
	//Calculate the LQR with gain array K and passed errors
	LQR_value = -(K[0]*(*x_error) + K[1]*(*xdot_error) + K[2]*(*theta_error) + K[3]*(*thetadot_error));
	return &LQR_value;

}