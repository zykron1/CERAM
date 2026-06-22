#line 1 "/home/ahsan/code/ArduinoTVC/StateEstimator.h"
struct Orientation {
    float x, y, z;
    Orientation operator*(float s) const { return {x*s, y*s, z*s}; }
    Orientation& operator+=(const Orientation& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
};

class StateEstimator {
public:
    void updateState(Orientation gyro, float altitude, float elapsed_ms);
    Orientation getOrientation(); // i use radians 
	float getAltitude();
	float getVelocity();
	void resetVelocity();

private:
	// dcm init rows, credit wikipedia
    float R[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };

	bool _initialized = false;
	float _lastAltitude = 0.0f;
	float _velocity = 0.0f;

    void normalizeDCM();
};
