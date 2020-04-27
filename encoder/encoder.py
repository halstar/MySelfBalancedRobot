import RPi.GPIO

BOUNCE_TIME = 10


class Encoder:

    def __init__(self, encoder_pin_1, encoder_pin_2):

        self.encoder_pin_1 = encoder_pin_1
        self.encoder_pin_2 = encoder_pin_2

        RPi.GPIO.setmode(RPi.GPIO.BCM)
        RPi.GPIO.setwarnings(False)

        RPi.GPIO.setup(encoder_pin_1, RPi.GPIO.IN)
        RPi.GPIO.setup(encoder_pin_2, RPi.GPIO.IN)

        RPi.GPIO.add_event_detect(encoder_pin_1, RPi.GPIO.FALLING, callback=self.callback, bouncetime=BOUNCE_TIME)

        self.counter = 0

    def callback(self, channel):

        pin_1_state = RPi.GPIO.input(self.encoder_pin_1)
        pin_2_state = RPi.GPIO.input(self.encoder_pin_2)

        if pin_1_state != pin_2_state:
            self.counter += 1
        else:
            self.counter -= 1

    def get_counter(self):
        return self.counter

    def reset_counter(self):
        self.counter = 0