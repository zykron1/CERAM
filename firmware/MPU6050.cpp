#include "MPU6050.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

void MPU6050::init(int address) {

	if (!this->mpu.begin(address)) {
		Serial.println("MPU6050 not found!");
		while (1) delay(10);
	}

	// Configuration
	this->mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
	this->mpu.setGyroRange(MPU6050_RANGE_500_DEG);
	this->mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);

	Serial.println("Ready");
}


void MPU6050::calibrate(int samples, int dt_ms) {
    this->drift = {0};
    delay(200); // let filter settle
	
    
    const uint32_t period_us = dt_ms * 1000;
    uint32_t lastTime = micros();
    for (int i = 0; i < samples; i++) {
        while (micros() - lastTime < period_us);
        lastTime += period_us;
        // Discard and re-read to ensure fresh data
        sensors_event_t a, g, t;
        this->mpu.getEvent(&a, &g, &t);
        delay(1);
        this->mpu.getEvent(&a, &g, &t);
        MPU6050Data d = {a.acceleration.x, a.acceleration.y, a.acceleration.z,
                         g.gyro.x, g.gyro.y, g.gyro.z, t.temperature};
        this->drift += d;
    }
    this->drift /= samples;
	
}

MPU6050Data MPU6050::getData() {
	sensors_event_t accel, gyro, temp;
	this->mpu.getEvent(&accel, &gyro, &temp);
	return MPU6050Data {
		accel.acceleration.x - this->drift.ax,
		accel.acceleration.y - this->drift.ay,
		accel.acceleration.z - this->drift.az,
		gyro.gyro.x - this->drift.gx,
		gyro.gyro.y - this->drift.gy,
		gyro.gyro.z - this->drift.gz,
		temp.temperature - this->drift.temp
	};
}
