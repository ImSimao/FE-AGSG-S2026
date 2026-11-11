from machine import Pin, PWM


class Servo:
    _servo = PWM(Pin(3))
    _servo.freq(50)  # 50Hz

    _MIN_DUTY = 1638   # ~0.5ms at 50Hz
    _MAX_DUTY = 8192   # ~2.5ms at 50Hz

    _CENTER_STEERING = 96.5
    _MAX_STEERING_OFFSET = 29

    current_angle = 0   # latest clamped steering angle (degrees)

    @staticmethod
    def set_angle(angle: int) -> None:
        
        if angle > Servo._MAX_STEERING_OFFSET:
            angle = Servo._MAX_STEERING_OFFSET
        elif angle < -Servo._MAX_STEERING_OFFSET:
            angle = -Servo._MAX_STEERING_OFFSET

        Servo.current_angle = angle

        angle = Servo._CENTER_STEERING - angle

        
        duty = int(
            Servo._MIN_DUTY
            + (angle / 180) * (Servo._MAX_DUTY - Servo._MIN_DUTY)
        )
        Servo._servo.duty_u16(duty)
