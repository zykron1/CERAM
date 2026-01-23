from machine import I2C, Pin
import time
import math

class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')
        self.accel_drift = [0,0,0]
        self.gyro_drift = [0,0,0]
        self.angle_x = 0.0
        self.angle_y = 0.0
        self.angle_z = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0

    def _read_raw(self, reg):
        data = self.i2c.readfrom_mem(self.addr, reg, 2)
        value = int.from_bytes(data, 'big')
        if value > 32767:
            value -= 65536
        return value

    def _run_tune(self, seconds):
        print(f"Calibrating for {seconds} seconds...")

        accel_offsets = [0.0, 0.0, 0.0]
        gyro_offsets = [0.0, 0.0, 0.0]
        sample_count = 0

        start_time = time.time()
        while time.time() - start_time < seconds:
            ax, ay, az, gx, gy, gz = self._get_sensor_data()

            # Accumulate
            accel_offsets[0] += ax
            accel_offsets[1] += ay
            accel_offsets[2] += az
            gyro_offsets[0] += gx
            gyro_offsets[1] += gy
            gyro_offsets[2] += gz

            sample_count += 1
            time.sleep(0.01)

        # Average
        accel_offsets = [v / sample_count for v in accel_offsets]
        gyro_offsets = [v / sample_count for v in gyro_offsets]

        accel_offsets[2] -= 1.0

        print("Accel drift:", accel_offsets)
        print("Gyro drift:", gyro_offsets)

        self.accel_drift = accel_offsets
        self.gyro_drift = gyro_offsets
        
    def _get_sensor_data(self):
        ax = self._read_raw(0x3B) / 16384.0
        ay = self._read_raw(0x3D) / 16384.0
        az = self._read_raw(0x3F) / 16384.0
        gx = self._read_raw(0x43) / 131.0
        gy = self._read_raw(0x45) / 131.0
        gz = self._read_raw(0x47) / 131.0
        return ay, az, ax, gy, gz, gx #due to MPU6050 mounting on the rocket
    
        
    def get_sensor_data(self):
        ax, ay, az, gx, gy, gz = self._get_sensor_data()
        
        ax -= self.accel_drift[0]
        ay -= self.accel_drift[1]
        az -= self.accel_drift[2]
        gx -= self.gyro_drift[0]
        gy -= self.gyro_drift[1]
        gz -= self.gyro_drift[2]

        return ax, ay, az, gx, gy, gz
    
    def get_orientation(self, dt):
        ax, ay, az, gx, gy, gz = self.get_sensor_data()

        # Integrate gyroscope data (simple integration: θ += ω * dt)
        self.angle_x += gx * dt                                                
        self.angle_y += gy * dt
        self.angle_z += gz * dt

        return self.angle_x, self.angle_y, self.angle_z
    
    def get_velocity(self, dt):
        ax, ay, az, gx, gy, gz = self.get_sensor_data()
        az -= 1.0

        # Integrate acecelerometer data (simple integration: θ += ω * dt); convert G's to m/s
        self.velocity_x += ax * dt * 9.8                                                
        self.velocity_y += ay * dt * 9.8
        self.velocity_z += az * dt * 9.8
        return self.velocity_x, self.velocity_y, self.velocity_z

# Example usage
if __name__ == "__main__":
    i2c = I2C(0, scl=Pin(1), sda=Pin(0))
    mpu = MPU6050(i2c)
    mpu._run_tune(10)

    while True:
        dt = 0.01
        angle_x, angle_y, angle_z = mpu.get_orientation(dt)
        #IMPORTANT: THE Z COORDINATE ON THE ROCKET IS THE X COORDINATE ON THE MPU6050 AND VICE VERSA DUE TO MOUNTING METHOD
        print("Orientation (°):", angle_x, angle_y, angle_z)
        time.sleep(dt)
