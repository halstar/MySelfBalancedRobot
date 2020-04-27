import os
import json
import motor

LEFT_MOTOR_ENABLE = 16
LEFT_MOTOR_PIN_1  = 20
LEFT_MOTOR_PIN_2  = 21

RIGHT_MOTOR_ENABLE = 13
RIGHT_MOTOR_PIN_1  = 26
RIGHT_MOTOR_PIN_2  = 19

SETUP_FILE = 'setup.json'


def main():

    os.system("clear")

    print("")
    print("/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\ ")
    print("| Motors calibration starting... |")
    print("\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/ ")
    print("")

    with open(SETUP_FILE, 'r') as json_file:
        setup_data = json.load(json_file)

    left_motor  = motor.Motor(LEFT_MOTOR_ENABLE , LEFT_MOTOR_PIN_1 , LEFT_MOTOR_PIN_2 , 0)
    right_motor = motor.Motor(RIGHT_MOTOR_ENABLE, RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2, 0)

    speed_value   = 0
    is_motor_done = False

    while not is_motor_done:

        left_motor.forward(speed_value)
        print("Left motor speed: {}. Hit Enter if motor is not moving, else type OK: ".format(speed_value), end="", flush=True)
        user_input = input()
        if user_input == 'OK':
            is_motor_done = True
            left_motor.stop()
        else:
            speed_value += 1

    setup_data['LEFT_MOTOR_OFFSET'] = speed_value

    speed_value   = 0
    is_motor_done = False

    print("")

    while not is_motor_done:

        right_motor.forward(speed_value)
        print("Right motor speed: {}. Hit Enter if motor is not moving, else type OK: ".format(speed_value), end="", flush=True)
        user_input = input()
        if user_input == 'OK':
            is_motor_done = True
            right_motor.stop()
        else:
            speed_value += 1

    setup_data['RIGHT_MOTOR_OFFSET'] = speed_value

    with open(SETUP_FILE, "w") as json_file:
        json.dump(setup_data, json_file, indent=4)

    print("")
    print("/\/\/\/\/\/\/\/\/\/\/\/\/\/\ ")
    print("| Motors calibration done! |")
    print("\/\/\/\/\/\/\/\/\/\/\/\/\/\/ ")
    print("")


if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print("Keyboard interrupt...")

    except Exception as e:
        print("Error: " + str(e))
