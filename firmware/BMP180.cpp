#include "BMP180.h"

void BMP180::init(int address) {
	if (!this->bmp.begin(address)) {
		Serial.println("BMP180 not found!");
		while (1) delay(10);
	}
}

BMP180Data BMP180::getData() {
	return {
		bmp.readTemperature(),
		bmp.readPressure() / 100.0f,
		bmp.readAltitude() 
	};
}
