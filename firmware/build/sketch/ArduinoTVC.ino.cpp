#include <Arduino.h>
#line 1 "/home/ahsan/code/ArduinoTVC/ArduinoTVC.ino"
#include <Wire.h>
#include "MPU6050.h"
#include "StateEstimator.h"
#include "SG90.h"
#include "States.h"
#include "BMP180.h"
#include "PID.h"
#include "TVCMount.h"
#include "esp32-hal-gpio.h"

#define MOTOR_BURN 395.5f
#define DEPLOY_TIMER 5 // If baro fails
#define APOGEE_DROPOUT 2 // Meters to fall before deployment

MPU6050 imu;
BMP180 baro;

SG90 xservo;
SG90 yservo;
TVCMount mount(xservo, yservo);

PID xPID(0.4, 0, 0.2);
PID yPID(0.4, 0, 0.2);

MPU6050Data i;
BMP180Data b;
float apogee;

StateEstimator state;
States stage = PAD;

uint32_t startTime = 0;
uint32_t lastTime = 0;

#line 35 "/home/ahsan/code/ArduinoTVC/ArduinoTVC.ino"
void setup();
#line 55 "/home/ahsan/code/ArduinoTVC/ArduinoTVC.ino"
void loop();
#line 35 "/home/ahsan/code/ArduinoTVC/ArduinoTVC.ino"
void setup() {
	Serial.begin(115200);
	Wire.begin(6, 7);

	imu.init(0x68);
	baro.init(0x77);

	delay(100);
	imu.calibrate(500, 10); // 500 samples @ 10ms
	Serial.printf("{%f, %f, %f} \n", imu.drift.gx, imu.drift.gy, imu.drift.gz);

	xservo.initServo(9); // INNER
	yservo.initServo(2); // OUTER
	pinMode(4, OUTPUT);

	startTime = micros();
	lastTime = startTime;
}

const uint32_t PERIOD_US = 10000; 
void loop() {
	switch (stage) {
		case PAD:
		{
			// Wait for launch, poll accelerometers, do not integrate state
			i = imu.getData();
			b = baro.getData();
			Serial.printf("AWAITING LAUNCH %f m/s2 BARO ALT: %f\n", i.ax, b.altitude);
			if (i.ax > 1.0f) { // 1 m/s threshold
				Serial.printf("Staging to TVC \n");
				stage = next(stage);
				startTime = micros();
				lastTime = micros();
			}
			break;
		}
		case POWERED_FLIGHT:
		{
			uint32_t now = micros();
			float t = (now - startTime) / 1000000.0f;

			// Stage after motor burnout
			if (t >= MOTOR_BURN) {
				state.resetVelocity();
				stage = next(stage);
				startTime = micros(); 
				lastTime = micros();
				b = baro.getData();
				apogee = b.altitude;
				Serial.println("Staging to coast phase");
				break;
			}
			if (now - lastTime >= PERIOD_US) {
				float actual_ms = (now - lastTime) / 1000.0f;
				lastTime += PERIOD_US;

				i = imu.getData();

				state.updateState({i.gx, i.gy, i.gz}, b.altitude, actual_ms); // dont poll alt during flight

				Orientation rocket = state.getOrientation() * RAD_TO_DEG;
				float xAngle = rocket.y;
				float yAngle = -rocket.z; // due to mpu mounting

				int xCommand = (int)xPID.updateLoop(xAngle, 0, 0.01); // state=xAngle, setpoint 0, dt = 10ms
				int yCommand = (int)yPID.updateLoop(yAngle, 0, 0.01); // state=yAngle, setpoint 0, dt = 10ms
				xCommand = constrain(xCommand, -5, 5);
				yCommand = constrain(yCommand, -5, 5);


				mount.moveMount(xCommand, yCommand);

				Serial.printf("TVC: t=%f, dt=%f ms, roll=%f, x=%f, y=%f command = %d, %d \n",
						(micros() - startTime) / 1000000.0f,
						actual_ms,
						rocket.x, xAngle, yAngle, xCommand, yCommand);

			}
			break;
		}
		case COAST:
		{
			uint32_t now = micros();
			float t = (now - startTime) / 1000000.0f;

			if (now - lastTime >= PERIOD_US) {
				float actual_ms = (now - lastTime) / 1000.0f;
				lastTime += PERIOD_US;

				i = imu.getData();
				b = baro.getData();

				state.updateState({i.gx, i.gy, i.gz}, b.altitude, actual_ms);
				Serial.printf("COASTING: %f m \n", b.altitude);

				if (b.altitude > apogee) {
					Serial.printf("NEW APOGEE: %f m\n", b.altitude);
					apogee = b.altitude;
				}

				if (b.altitude < apogee - APOGEE_DROPOUT || t > DEPLOY_TIMER) {
					Serial.printf("Staging to parachute deployment @ %f m t = %f s\n", b.altitude, t);
					stage = next(stage);
					startTime = micros();
					lastTime = micros();
				}
			}
			break;
		}
		case DEPLOY:
		{
			// Set explosive channels to high
			delay(20*1000);
			Serial.printf("FIRING! \n");
			// Parachute should hopefully deploy..
			digitalWrite(4, HIGH); // PYRO1 HIGH
			break;
		}
	}
}

