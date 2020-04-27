import utils
import RPi.GPIO


STOPPED  = 0
FORWARD  = 1
BACKWARD = 2

PWM_MIN = 0
PWM_MAX = 100


class Motor:

    def __init__(self, enable_pin, motor_pin_1, motor_pin_2, pwm_offset):

        self.motor_pin_1 = motor_pin_1
        self.motor_pin_2 = motor_pin_2
        self.pwm_offset  = pwm_offset

        RPi.GPIO.setmode(RPi.GPIO.BCM)
        RPi.GPIO.setwarnings(False)

        RPi.GPIO.setup(enable_pin , RPi.GPIO.OUT)
        RPi.GPIO.setup(motor_pin_1, RPi.GPIO.OUT)
        RPi.GPIO.setup(motor_pin_2, RPi.GPIO.OUT)

        RPi.GPIO.output(motor_pin_1, RPi.GPIO.LOW)
        RPi.GPIO.output(motor_pin_2, RPi.GPIO.LOW)
        RPi.GPIO.output(enable_pin , RPi.GPIO.LOW)

        self.sw_pwm = RPi.GPIO.PWM(enable_pin, PWM_MAX)
        self.sw_pwm.start(PWM_MIN)
        self.speed     = PWM_MIN
        self.direction = STOPPED

    def forward(self, speed):

        speed += self.pwm_offset
        speed  = utils.clamp(speed, self.pwm_offset, PWM_MAX)

        if self.direction == STOPPED:

            self.sw_pwm.start(speed)
            self.speed = speed

        elif speed != self.speed:

            self.sw_pwm.ChangeDutyCycle(speed)
            self.speed = speed

        if self.direction != FORWARD:

            RPi.GPIO.output(self.motor_pin_1, RPi.GPIO.LOW )
            RPi.GPIO.output(self.motor_pin_2, RPi.GPIO.HIGH)

            self.direction = FORWARD

    def backward(self, speed):

        speed += self.pwm_offset
        speed  = utils.clamp(speed, self.pwm_offset, PWM_MAX)

        if self.direction == STOPPED:

            self.sw_pwm.start(speed)
            self.speed = speed

        elif speed != self.speed:

            self.sw_pwm.ChangeDutyCycle(speed)
            self.speed = speed

        if self.direction != BACKWARD:

            RPi.GPIO.output(self.motor_pin_1, RPi.GPIO.HIGH)
            RPi.GPIO.output(self.motor_pin_2, RPi.GPIO.LOW )

            self.direction = BACKWARD

    def stop(self):

        if self.direction != STOPPED:

            self.sw_pwm.stop()
            self.speed = PWM_MIN

            RPi.GPIO.output(self.motor_pin_1, RPi.GPIO.LOW)
            RPi.GPIO.output(self.motor_pin_2, RPi.GPIO.LOW)

            self.direction = STOPPED

    def get_speed(self):

        return self.speed