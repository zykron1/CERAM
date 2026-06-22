#include "SG90.h"
class TVCMount {
	private:
		const float x_ratio = 1 / -0.394; // inner
		const float y_ratio = 1 / 0.335; // outer

		const float x_offset = 100;
		const float y_offset = 90;

		SG90& x_servo;
		SG90& y_servo;

	public:
		TVCMount(SG90& x, SG90& y)
            : x_servo(x), y_servo(y) {}
		void moveMount(float x, float y);
};
