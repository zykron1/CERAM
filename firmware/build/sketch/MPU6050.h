#line 1 "/home/ahsan/code/ArduinoTVC/MPU6050.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

struct MPU6050Data {
	float ax, ay, az, gx, gy, gz, temp = 0;

	MPU6050Data& operator+=(const MPU6050Data& other) {
		ax += other.ax; ay += other.ay; az += other.az;
		gx += other.gx; gy += other.gy; gz += other.gz;
		temp += other.temp;
		return *this;
	}

	MPU6050Data& operator/=(float scalar) {
		ax /= scalar; ay /= scalar; az /= scalar;
		gx /= scalar; gy /= scalar; gz /= scalar;
		temp /= scalar;
		return *this;
	}
};

class MPU6050 {
	private:
		Adafruit_MPU6050 mpu;
	public:
		MPU6050Data drift;
		void init(int address);
		void calibrate(int samples, int dt_ms);
		MPU6050Data getData();
};
