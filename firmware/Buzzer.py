from machine import Pin, PWM
import time

class Buzzer:
    def __init__(self, pin):
        self.buzzer = PWM(Pin(pin))
    def play_tone(self, freq, duration):
        self.buzzer.freq(freq)
        self.buzzer.duty_u16(30000)  # 50% duty cycle
        time.sleep(duration)
        self.buzzer.duty_u16(0)
        time.sleep(0.05)
        
if __name__ == "__main__":
    buzzer = Buzzer(14)
    for i in range(3):
        buzzer.play_tone(950, .1)
        time.sleep(0.01)