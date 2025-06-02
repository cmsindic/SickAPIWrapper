import sys
import time
import uselect  # This is available on MicroPython for Pico
from machine import Pin, I2C
from bno055 import BNO055
from quadrature import Quadrature

# === I2C IMU Setup ===
i2c = I2C(0, scl=Pin(1), sda=Pin(0))
imu = BNO055(i2c)
time.sleep(0.05)

p = Pin(16)

print(p)

# === Encoder Setup ===
encoder = Quadrature(pin_a=2, pin_b=3)

# === Poller for serial input ===
poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

# === State variables ===
base_count = 0
base_euler = (0.0, 0.0, 0.0)

# === Main loop ===
while True:
    if poll.poll(0):  # Non-blocking check
        cmd = sys.stdin.readline().strip()
        if cmd == "RESET":
            base_count = encoder.position()
            try:
                base_euler = imu.euler()
            except:
                base_euler = (0.0, 0.0, 0.0)

    t = time.ticks_us()
    raw_count = encoder.position()

    try:
        euler = imu.euler()
    except:
        euler = (None, None, None)

    count = raw_count - base_count
    rel_euler = tuple(
        e - b if e is not None and b is not None else None
        for e, b in zip(euler, base_euler)
    )

    print(f"{t},{count},{rel_euler[0]},{rel_euler[1]},{rel_euler[2]},{p.value()}")
    time.sleep_us(500)

