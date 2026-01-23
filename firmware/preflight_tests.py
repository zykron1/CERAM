from machine import Pin, I2C
import time
from drivers.SG90 import Servo
from drivers.MPU6050 import MPU6050
from drivers.Buzzer import Buzzer

#Setup Code
servoX = Servo(Pin(10)) # X Axis
servoY = Servo(Pin(5)) #Y axis
buzzer = Buzzer(14)
i2c = I2C(0, scl=Pin(1), sda=Pin(0))
mpu = MPU6050(i2c)

#Initialization
buzzer.play_tone(990, 0.2)
buzzer.play_tone(990, 0.2)
servoX.set_servo_angle(0)
servoY.set_servo_angle(0)
mpu._run_tune(10)
buzzer.play_tone(990, 0.2)
servoX.set_servo_angle(90)
servoY.set_servo_angle(90)


#Detect "launch"
print("Waiting for launch...")
launched = False
while launched == False:
    dt = 0.01
    velocity_x, velocity_y, velocity_z = mpu.get_velocity(dt)
    if velocity_z > 0.25:
        launched = True
    
#Main loop
c = 0
while True:
    dt = 0.01
    angle_x, angle_y, angle_z = mpu.get_orientation(dt)
    velocity_x, velocity_y, velocity_z = mpu.get_velocity(dt)
    print(
        f"Orientation (°){c}: "
        f"{angle_x:.2f}, {angle_y:.2f}, {angle_z:.2f} | "
        f"Velocity (m/s): "
        f"{velocity_x:.2f}, {velocity_y:.2f}, {velocity_z:.2f}"
    )
    servoX.set_servo_angle(abs(angle_x))
    servoY.set_servo_angle(abs(angle_y))
    time.sleep(dt)
    c += 1