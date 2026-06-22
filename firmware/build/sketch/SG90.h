#line 1 "/home/ahsan/code/ArduinoTVC/SG90.h"
#pragma once
#include <ESP32Servo.h>

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2400;
const int SWEEP_DELAY  = 15;

class SG90 {
	private:
		int port;
		Servo servo;
	public:
		void initServo(int p);
		void setPosition(int angle);
		void smoothPosition(int angle);
};
