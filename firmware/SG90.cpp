#include "SG90.h"

void SG90::initServo(int p) {
	servo.setPeriodHertz(50);
	bool ok = servo.attach(p, SERVO_MIN_US, SERVO_MAX_US);
	Serial.printf("Servo attach on pin %d: %s\n", p, ok ? "OK" : "FAILED");
	delay(500);
}

void SG90::setPosition(int angle) {
	angle = constrain(angle, 0, 180);
	servo.write(angle);
}

void SG90::smoothPosition(int angle) {
	angle = constrain(angle, 0, 180);
	int current = servo.read();
	int step = (angle > current) ? 1 : -1;
	for (int pos = current; pos != angle; pos += step) {
		servo.write(pos);
		delay(SWEEP_DELAY);
	}
	servo.write(angle);
}
