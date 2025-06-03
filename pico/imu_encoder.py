from machine import Pin, I2C
import time
from bno055 import BNO055
from quadrature import Quadrature

# === I2C IMU Setup ===
i2c = I2C(0, scl=Pin(1), sda=Pin(0))  # GP1 = SCL, GP0 = SDA
imu = BNO055(i2c)
time.sleep(.05)  # Let IMU boot

# === PIO Quadrature Encoder Setup ===
encoder = Quadrature(pin_a=2, pin_b=3)

# === Main Loop ===
while True:
    t = time.ticks_us()
    count = encoder.position()
    try:
        euler = imu.euler()
    except:
        euler = (None, None, None)

    print(f"{t},{count},{euler[0]},{euler[1]},{euler[2]}")
    time.sleep_us(500)  # ~2 kHz loop
