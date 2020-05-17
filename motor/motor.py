import const
import utils
import RPi.GPIO
import gpiozero
import pigpio


STOPPED  = 0
FORWARD  = 1
BACKWARD = 2

PWM_FREQUENCY = 20000
PWM_MIN       = 0
PWM_MAX       = 100


class Motor:

    def __init__(self, gpio_interface, enable_pin, motor_pin_1, motor_pin_2, pwm_offset):

        self.gpio_interface = gpio_interface
        self.enable_pin     = enable_pin
        self.motor_pin_1    = motor_pin_1
        self.motor_pin_2    = motor_pin_2
        self.pwm_offset     = pwm_offset

        if self.gpio_interface == const.USE_RPI_GPIO:

            RPi.GPIO.setmode(RPi.GPIO.BCM)
            RPi.GPIO.setwarnings(False)
    
            RPi.GPIO.setup(enable_pin , RPi.GPIO.OUT)
            RPi.GPIO.setup(motor_pin_1, RPi.GPIO.OUT)
            RPi.GPIO.setup(motor_pin_2, RPi.GPIO.OUT)
    
            RPi.GPIO.output(motor_pin_1, RPi.GPIO.LOW)
            RPi.GPIO.output(motor_pin_2, RPi.GPIO.LOW)
            RPi.GPIO.output(enable_pin , RPi.GPIO.LOW)
    
            self.pwm = RPi.GPIO.PWM(enable_pin, PWM_FREQUENCY)
            self.pwm.start(PWM_MIN)

        elif self.gpio_interface == const.USE_RPI_ZERO:

            self.motor = gpiozero.Motor(motor_pin_2, motor_pin_1, enable_pin, pwm = True)
        
        elif self.gpio_interface == const.USE_PI_GPIO:

            self.pigpio = pigpio.pi()

            self.pigpio.set_mode(enable_pin , pigpio.OUTPUT)
            self.pigpio.set_mode(motor_pin_1, pigpio.OUTPUT)
            self.pigpio.set_mode(motor_pin_2, pigpio.OUTPUT)

            self.pigpio.write(enable_pin , 0)
            self.pigpio.write(motor_pin_1, 0)
            self.pigpio.write(motor_pin_2, 0)

            self.pigpio.set_PWM_range    (enable_pin, PWM_MAX      )
            self.pigpio.set_PWM_frequency(enable_pin, PWM_FREQUENCY)

        self.speed     = PWM_MIN
        self.direction = STOPPED

    def forward(self, speed):

        speed += self.pwm_offset
        speed  = utils.clamp(speed, self.pwm_offset, PWM_MAX)

        if self.direction == STOPPED:

            if self.gpio_interface == const.USE_RPI_GPIO:

                self.pwm.start(speed)

            elif self.gpio_interface == const.USE_RPI_ZERO:

                self.motor.forward(speed / PWM_MAX)

            elif self.gpio_interface == const.USE_PI_GPIO:

                self.pigpio.set_PWM_dutycycle(self.enable_pin, speed)

        elif speed != self.speed:

            if self.gpio_interface == const.USE_RPI_GPIO:

                self.pwm.ChangeDutyCycle(speed)

            elif self.gpio_interface == const.USE_RPI_ZERO:

                self.motor.forward(speed / PWM_MAX)

            elif self.gpio_interface == const.USE_PI_GPIO:

                self.pigpio.set_PWM_dutycycle(self.enable_pin, speed)

        self.speed = speed

        if self.direction != FORWARD:

            if self.gpio_interface == const.USE_RPI_GPIO:

                RPi.GPIO.output(self.motor_pin_1, RPi.GPIO.LOW )
                RPi.GPIO.output(self.motor_pin_2, RPi.GPIO.HIGH)

            elif self.gpio_interface == const.USE_RPI_ZERO:

                pass

            elif self.gpio_interface == const.USE_PI_GPIO:

                self.pigpio.write(self.motor_pin_1, 0)
                self.pigpio.write(self.motor_pin_2, 1)

            self.direction = FORWARD

    def backward(self, speed):

        speed += self.pwm_offset
        speed  = utils.clamp(speed, self.pwm_offset, PWM_MAX)

        if self.direction == STOPPED:

            if self.gpio_interface == const.USE_RPI_GPIO:

                self.pwm.start(speed)

            elif self.gpio_interface == const.USE_RPI_ZERO:

                self.motor.backward(speed / PWM_MAX)

            elif self.gpio_interface == const.USE_PI_GPIO:

                self.pigpio.set_PWM_dutycycle(self.enable_pin, speed)

        elif speed != self.speed:

            if self.gpio_interface == const.USE_RPI_GPIO:

                self.pwm.ChangeDutyCycle(speed)

            elif self.gpio_interface == const.USE_RPI_ZERO:

                self.motor.backward(speed / PWM_MAX)

            elif self.gpio_interface == const.USE_PI_GPIO:

                self.pigpio.set_PWM_dutycycle(self.enable_pin, speed)

        self.speed = speed

        if self.direction != BACKWARD:

            if self.gpio_interface == const.USE_RPI_GPIO:

                RPi.GPIO.output(self.motor_pin_1, RPi.GPIO.HIGH)
                RPi.GPIO.output(self.motor_pin_2, RPi.GPIO.LOW )

            elif self.gpio_interface == const.USE_RPI_ZERO:

                pass

            elif self.gpio_interface == const.USE_PI_GPIO:

                self.pigpio.write(self.motor_pin_1, 1)
                self.pigpio.write(self.motor_pin_2, 0)

            self.direction = BACKWARD

    def stop(self):

        if self.direction != STOPPED:

            if self.gpio_interface == const.USE_RPI_GPIO:

                self.pwm.stop()

                RPi.GPIO.output(self.motor_pin_1, RPi.GPIO.LOW)
                RPi.GPIO.output(self.motor_pin_2, RPi.GPIO.LOW)

            elif self.gpio_interface == const.USE_RPI_ZERO:

                self.motor.stop()

            elif self.gpio_interface == const.USE_PI_GPIO:

                self.pigpio.set_PWM_dutycycle(self.enable_pin, 0)

                self.pigpio.write(self.motor_pin_1, 0)
                self.pigpio.write(self.motor_pin_2, 0)

            self.speed     = PWM_MIN
            self.direction = STOPPED

    def get_speed(self):

        return self.speed