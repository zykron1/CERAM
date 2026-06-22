#include <Adafruit_BMP085.h>
struct BMP180Data {
	float temperature, pressure, altitude;
};

class BMP180 {
	private:
		Adafruit_BMP085 bmp;
	public:
		void init(int address);
		BMP180Data getData();
};
