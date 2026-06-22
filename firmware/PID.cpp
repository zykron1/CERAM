#include "PID.h"

float PID::updateLoop(float state, float setpoint, float dt) {
	// Calculate error
	float error = setpoint - state;

	// Update integral
	this->integral += error*dt;

	// Calculate derivative(delta y / delta t)
	float derivative = (error - prevError) / dt;
	this->prevError = error;
	
	return (this->Kp * error)
		 + (this->Ki * integral) 
		 + (this->Kd * derivative);
}
