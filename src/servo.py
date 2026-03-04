from machine import Pin, PWM


class Servo:
    _servo = PWM(Pin(3))
    _servo.freq(50)  # 50Hz

    _MIN_DUTY = 1638   # ~0.5ms em 50Hz
    _MAX_DUTY = 8192   # ~2.5ms em

    _CENTER_STEERING = 90
    _MAX_STEERING_OFFSET = 40

    @staticmethod
    def set_angle(angle: int) -> None:
        angle = max(
            Servo._CENTER_STEERING - Servo._MAX_STEERING_OFFSET,
            min(
                Servo._CENTER_STEERING + Servo._MAX_STEERING_OFFSET,
                Servo._CENTER_STEERING + angle,
            ),
        )

        duty = int(
            Servo._MIN_DUTY
            + (angle / 180) * (Servo._MAX_DUTY - Servo._MIN_DUTY)
        )
        Servo._servo.duty_u16(duty)
