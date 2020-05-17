import const
import utils
import RPi.GPIO
import gpiozero
import pigpio

"""
Decode a rotary encoder.

             +---------+         +---------+      0
             |         |         |         |
   1         |         |         |         |
             |         |         |         |
   +---------+         +---------+         +----- 1

       +---------+         +---------+            0
       |         |         |         |
   2   |         |         |         |
       |         |         |         |
   ----+         +---------+         +---------+  1
"""

BOUNCE_TIME = 10


class Encoder:

    def __init__(self, gpio_interface, encoder_pin_1, encoder_pin_2):

        self.gpio_interface = gpio_interface
        self.encoder_pin_1  = encoder_pin_1
        self.encoder_pin_2  = encoder_pin_2

        if self.gpio_interface == const.USE_RPI_GPIO:

            RPi.GPIO.setmode(RPi.GPIO.BCM)
            RPi.GPIO.setwarnings(False)

            RPi.GPIO.setup(encoder_pin_1, RPi.GPIO.IN)
            RPi.GPIO.setup(encoder_pin_2, RPi.GPIO.IN)

            RPi.GPIO.add_event_detect(encoder_pin_1, RPi.GPIO.FALLING, callback = self.callback, bouncetime = BOUNCE_TIME)

        elif self.gpio_interface == const.USE_RPI_ZERO:

            self.encoder_pin_1_device = gpiozero.DigitalInputDevice(encoder_pin_1, bounce_time = BOUNCE_TIME / 1000)
            self.encoder_pin_2_device = gpiozero.DigitalInputDevice(encoder_pin_2, bounce_time = BOUNCE_TIME / 1000)

            self.encoder_pin_1_device.when_deactivated = self.callback

        elif self.gpio_interface == const.USE_PI_GPIO:

            self.level_encoder_1 = 0
            self.level_encoder_2 = 0
            self.last_event_gpio = None

            self.pigpio = pigpio.pi()

            self.pigpio.set_mode(encoder_pin_1, pigpio.INPUT)
            self.pigpio.set_mode(encoder_pin_2, pigpio.INPUT)

            self.pigpio.set_pull_up_down(encoder_pin_1, pigpio.PUD_UP)
            self.pigpio.set_pull_up_down(encoder_pin_2, pigpio.PUD_UP)

            self.pigpio.callback(encoder_pin_1, pigpio.EITHER_EDGE, self.callback2)
            self.pigpio.callback(encoder_pin_2, pigpio.EITHER_EDGE, self.callback2)

        self.counter = 0

    def callback(self, channel):

        if self.gpio_interface == const.USE_RPI_GPIO:

            pin_1_state = RPi.GPIO.input(self.encoder_pin_1)
            pin_2_state = RPi.GPIO.input(self.encoder_pin_2)

        elif self.gpio_interface == const.USE_RPI_ZERO:

            pin_1_state = self.encoder_pin_1_device.value
            pin_2_state = self.encoder_pin_2_device.value

        if pin_1_state != pin_2_state:
            self.counter += 1
        else:
            self.counter -= 1

    def callback2(self, gpio, level, tick):

        if gpio == self.encoder_pin_1:
            self.level_encoder_1 = level
        else:
            self.level_encoder_2 = level;

        if gpio != self.last_event_gpio:

            self.last_event_gpio = gpio

            if gpio == self.encoder_pin_1 and level == 1:


                if self.level_encoder_2 == 1:
                    self.counter -= 1

            elif gpio == self.encoder_pin_2 and level == 1:

                if self.level_encoder_1 == 1:
                    self.counter += 1

    def get_counter(self):
        return self.counter

    def reset_counter(self):
        self.counter = 0