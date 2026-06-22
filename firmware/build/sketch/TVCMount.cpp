#line 1 "/home/ahsan/code/ArduinoTVC/TVCMount.cpp"
#include "TVCMount.h"

void TVCMount::moveMount(float x, float y) {
	this->x_servo.setPosition(this->x_offset + (this->x_ratio*x));
	this->y_servo.setPosition(this->y_offset + (this->y_ratio*y));
}
