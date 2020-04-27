import gpiozero
import time


class ProximitySensor:

    def __init__(self, trigger_pin, echo_pin):

        self.trigger_pin    = gpiozero.OutputDevice      (trigger_pin)
        self.echo_pin       = gpiozero.DigitalInputDevice(echo_pin   )
        self.round_distance = 0

    def get_distance(self):

        pulse_start = None
        pulse_end   = None


        self.trigger_pin.on()
        time.sleep(0.00001)
        self.trigger_pin.off()

        while self.echo_pin.is_active == False:
            pulse_start = time.time()

        while self.echo_pin.is_active == True:
            pulse_end = time.time()

        if pulse_start and pulse_end:

            pulse_duration = pulse_end - pulse_start
            distance = 34300 * (pulse_duration / 2)
            self.round_distance = round(distance, 1)

        return self.round_distance

    def print_proximity_info(self):

        print("Next obstacle  @ {:6.2f} cm ".format(self.round_distance))