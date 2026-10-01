from bno08x import *
from machine import I2C, Pin


class Compass:
    # Static / class-level hardware configuration
    I2C0_SDA = Pin(14)
    I2C0_SCL = Pin(15)
    i2c0 = I2C(1, scl=I2C0_SCL, sda=I2C0_SDA, freq=400_000, timeout=200_000)

    # BNO08X sensor instance
    bno = BNO08X(i2c0, debug=False)

    # Configure sensor once at class-definition time
    bno.set_quaternion_euler_vector(BNO_REPORT_GAME_ROTATION_VECTOR)
    fusion_frequency = 100  # Hz
    bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, fusion_frequency)

    def heading():
        return Compass.bno.euler_heading