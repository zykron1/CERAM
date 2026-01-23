from machine import Pin, PWM
from time import sleep

class Servo:
    def __init__(self, pin):
        self.pin = pin
        self.pwm = PWM(self.pin)
        self.pwm.freq(50)
        
    def set_servo_angle(self, angle):
        min_duty = 1638
        max_duty = 8191
        duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
        self.pwm.duty_u16(duty)

from machine import Pin
from time import sleep_us, sleep_ms

class BitBangServo:
    def __init__(self, pin):
        self.pin = Pin(pin, Pin.OUT)
        self.min_us = 1000   # 0° pulse width in microseconds
        self.max_us = 2000   # 180° pulse width in microseconds
        self.period_ms = 20  # Servo refresh period (20ms)
    
    def set_servo_angle(self, angle, duration_ms=500):
        """
        Move servo to angle gradually for duration_ms milliseconds.
        """
        # Clamp angle
        angle = max(0, min(180, angle))
        pulse_width = self.min_us + (angle / 180) * (self.max_us - self.min_us)
        
        # Calculate number of cycles
        cycles = duration_ms // self.period_ms
        for _ in range(cycles):
            self.pin.value(1)
            sleep_us(int(pulse_width))
            self.pin.value(0)
            sleep_ms(int(self.period_ms - pulse_width // 1000))

if __name__ == "__main__":
    servoY = BitBangServo(26)   # Y axis

    while True:
        servoY.set_servo_angle(50)
        sleep_ms(500)
        servoY.set_servo_angle(120)
        sleep_ms(500)
