import os
import json
import queue
import const
import utils
import imu
import motor
import encoder
import proximity
import pid
import time
import RPi.GPIO
import bluetooth
import picamera
import streamer
import threading

SETUP_FILE             = 'setup.json'
BALANCE_LOOP_TIME_STEP = 0.005

TARGET_SPEED_STEP =   85
TURNING_SPEED_STEP =  25
TURNING_ANGLE_STEP =  90
OBSTACLE_DISTANCE  =  30
SHUTDOWN_PITCH     =  30
DISTANCE_SAMPLES   = 100

# Default constants used by the speed PID controller
SPEED_PID_KP     = 0.020
SPEED_PID_KI     = 0.100
SPEED_PID_KD     = 0.000
SPEED_PID_TARGET = 0.000
SPEED_PID_MIN    = -TARGET_SPEED_STEP * 2
SPEED_PID_MAX    =  TARGET_SPEED_STEP * 2
SPEED_PID_WINDUP = 0.025

# Default constants used by the balance PID controller
BALANCE_PID_KP     =  20.00
BALANCE_PID_KI     = 100.00
BALANCE_PID_KD     =   0.25
BALANCE_PID_TARGET =  -2.50
BALANCE_PID_MIN    =  -motor.PWM_MAX
BALANCE_PID_MAX    =   motor.PWM_MAX
BALANCE_PID_WINDUP =  0.05

LEFT_MOTOR_ENABLE = 16
LEFT_MOTOR_PIN_1  = 20
LEFT_MOTOR_PIN_2  = 21

RIGHT_MOTOR_ENABLE = 13
RIGHT_MOTOR_PIN_1  = 26
RIGHT_MOTOR_PIN_2  = 19

LEFT_MOTOR_ENCODER_PIN_1 = 22
LEFT_MOTOR_ENCODER_PIN_2 = 23

RIGHT_MOTOR_ENCODER_PIN_1 = 18
RIGHT_MOTOR_ENCODER_PIN_2 = 17

PROXIMITY_SENSOR_TRIGGER_PIN = 12
PROXIMITY_SENSOR_ECHO_PIN    =  6

debug_mode                  = True
are_motors_on               = False
is_camera_on                = False
is_camera_recording         = False
is_obstacles_avoidance_on   = False
complementary_filter_factor = 0.998
distance_readings           = []
distance_samples            = DISTANCE_SAMPLES
equilibrium_angle           = BALANCE_PID_TARGET
equilibrium_limit           = 0.1
filtered_pitch              = 0.0
relative_yaw_angle          = 0.0
target_speed                = SPEED_PID_TARGET
turn_angle_order            = 0.0
turn_speed_step             = 0.0
left_counter                = 0
right_counter               = 0


def balance_control_thread(imu_device, speed_pid_controller, balance_pid_controller):

    global debug_mode
    global complementary_filter_factor
    global distance_readings
    global distance_samples
    global equilibrium_angle
    global equilibrium_limit
    global filtered_pitch
    global relative_yaw_angle
    global target_speed
    global turn_angle_order
    global turn_speed_step
    global left_counter
    global right_counter

    with open(SETUP_FILE, 'r') as json_file:
        setup_data = json.load(json_file)

    # Setup motors with their dedicated GPIOs and offsets
    left_motor  = motor.Motor(const.USE_RPI_GPIO, LEFT_MOTOR_ENABLE , LEFT_MOTOR_PIN_1 , LEFT_MOTOR_PIN_2 , setup_data['LEFT_MOTOR_OFFSET' ])
    right_motor = motor.Motor(const.USE_RPI_GPIO, RIGHT_MOTOR_ENABLE, RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2, setup_data['RIGHT_MOTOR_OFFSET'])

    # Setup left/right motors encoders with their dedicated GPIOs
    left_encoder  = encoder.Encoder(const.USE_RPI_GPIO, LEFT_MOTOR_ENCODER_PIN_1 , LEFT_MOTOR_ENCODER_PIN_2 )
    right_encoder = encoder.Encoder(const.USE_RPI_GPIO, RIGHT_MOTOR_ENCODER_PIN_1, RIGHT_MOTOR_ENCODER_PIN_2)

    # Setup IMU
    imu_device.reset        ()
    imu_device.reset_offsets()

    # Setup acceleration offsets
    imu_device.set_x_acceleration_offset(setup_data['ACCELERATION_X_OFFSET'])
    imu_device.set_y_acceleration_offset(setup_data['ACCELERATION_Y_OFFSET'])
    imu_device.set_z_acceleration_offset(setup_data['ACCELERATION_Z_OFFSET'])

    # Setup gyroscope offsets
    imu_device.set_x_gyroscope_offset(setup_data['GYROSCOPE_X_OFFSET'])
    imu_device.set_y_gyroscope_offset(setup_data['GYROSCOPE_Y_OFFSET'])
    imu_device.set_z_gyroscope_offset(setup_data['GYROSCOPE_Z_OFFSET'])

    while True:

        start_time = time.time()

        # ######################### #
        # Speed control computation #
        # ######################### #

        speed_pid_controller.set_target(target_speed)

        left_counter  = left_encoder.get_counter()
        right_counter = right_encoder.get_counter()

        distance_readings.append((left_counter + right_counter) / 2)

        if len(distance_readings) > distance_samples:
            distance_readings.pop(0)
            current_speed = sum(distance_readings) / (distance_samples * BALANCE_LOOP_TIME_STEP)
        else:
            current_speed = target_speed

        target_angle = equilibrium_angle + speed_pid_controller.update(current_speed, distance_samples * BALANCE_LOOP_TIME_STEP)

        left_encoder.reset_counter ()
        right_encoder.reset_counter()

        # ########################### #
        # Balance control computation #
        # ########################### #

        balance_pid_controller.set_target(target_angle)

        try:
            imu_device.read_acceleration_data()
            imu_device.read_gyroscope_data   ()
        except Exception as e:
            if debug_mode == True:
                print("IMU/I2C error detected")
                print("Error: " + str(e))
                print("Retrying!...")
            continue

        imu_device.compute_angles()
        imu_device.compute_rates ()

        pitch      = imu_device.get_pitch     ()
        pitch_rate = imu_device.get_pitch_rate()
        yaw_rate   = imu_device.get_yaw_rate  ()

        filtered_pitch    = complementary_filter_factor * (filtered_pitch + pitch_rate * BALANCE_LOOP_TIME_STEP) + (1 - complementary_filter_factor) * pitch
        balance_pid_speed = balance_pid_controller.update(filtered_pitch, BALANCE_LOOP_TIME_STEP)

        relative_yaw_angle += yaw_rate * BALANCE_LOOP_TIME_STEP

        # ########################### #
        # Left/right turn computation #
        # ########################### #

        if (turn_speed_step > 0 and relative_yaw_angle >= turn_angle_order) \
        or (turn_speed_step < 0 and relative_yaw_angle <= turn_angle_order):

            turn_angle_order = 0.0
            turn_speed_step  = 0.0

            if debug_mode == True:
                print("Ordered turn is over!")

        # ########################## #
        # Overall speeds computation #
        # ########################## #

        overall_left_speed  = balance_pid_speed + turn_speed_step
        overall_right_speed = balance_pid_speed - turn_speed_step

        # ######################### #
        # Update motors with speeds #
        # ######################### #

        if (are_motors_on == False) \
            or ((overall_left_speed == 0) and (overall_right_speed == 0)) \
            or (target_angle - equilibrium_limit <= filtered_pitch <= target_angle + equilibrium_limit) \
            or (filtered_pitch <= -SHUTDOWN_PITCH) \
            or (filtered_pitch >= SHUTDOWN_PITCH):

            left_motor.stop ()
            right_motor.stop()

        else:

            if overall_left_speed > 0:
                left_motor.forward ( overall_left_speed)
            else:
                left_motor.backward(-overall_left_speed)

            if overall_right_speed > 0:
                right_motor.forward ( overall_right_speed)
            else:
                right_motor.backward(-overall_right_speed)

        end_time = time.time()

        elapsed_time = end_time - start_time

        if elapsed_time < BALANCE_LOOP_TIME_STEP:
            time.sleep(BALANCE_LOOP_TIME_STEP - elapsed_time)
        elif debug_mode == True:
            print("Balance thread is late!")


def obstacles_avoidance_thread(proximity_sensor):

    global is_obstacles_avoidance_on
    global target_speed
    global turn_angle_order
    global turn_speed_step
    global relative_yaw_angle

    number_of_samples = 0
    obstacle_distance_samples = queue.Queue()
    robot_seems_stuck = False

    while True:

        next_obstacle_distance = proximity_sensor.get_distance()

        obstacle_distance_samples.put(next_obstacle_distance)
        number_of_samples += 1

        if number_of_samples >= 3:

            if max(list(obstacle_distance_samples.queue)) - min(list(obstacle_distance_samples.queue)) < 1:
                robot_seems_stuck = True
            else:
                robot_seems_stuck = False

            obstacle_distance_samples.get()

        if is_obstacles_avoidance_on == False:

            time.sleep(1.0)

        else:

            if robot_seems_stuck == True:

                if debug_mode == True:
                    print("Robot seems stuck: stopping")

                turn_angle_order = 0.0
                turn_speed_step  = 0.0
                target_speed     = 0.0

                time.sleep(1.0)

                if debug_mode == True:
                    print("Robot seems stuck: going backward")

                target_speed = -TARGET_SPEED_STEP * 1.10

                time.sleep(1.5)

                if debug_mode == True:
                    print("Robot seems stuck: stopping")

                target_speed = 0.0

                time.sleep(1.0)

                if debug_mode == True:
                    print("Robot seems stuck: turning right")

                turn_angle_order = relative_yaw_angle - TURNING_ANGLE_STEP * 0.75
                turn_speed_step  = -TURNING_SPEED_STEP

                time.sleep(2.0)

                robot_seems_stuck = False
                number_of_samples = 0
                obstacle_distance_samples.empty()

            elif next_obstacle_distance < OBSTACLE_DISTANCE:

                if debug_mode == True:
                    print("Obstacle ahead: stopping")

                turn_angle_order = 0.0
                turn_speed_step  = 0.0
                target_speed     = 0.0

                time.sleep(1.0)

                if debug_mode == True:
                    print("Obstacle ahead: turning right")

                turn_angle_order = relative_yaw_angle - TURNING_ANGLE_STEP * 0.75
                turn_speed_step  = -TURNING_SPEED_STEP

                time.sleep(2.0)

            else:

                if debug_mode == True:
                    print("Path is clear: keeping going forward")

                turn_angle_order = 0.0
                turn_speed_step  = 0.0
                target_speed     = TARGET_SPEED_STEP * 1.10

                time.sleep(0.1)


def bluetooth_control_thread():

    global debug_mode
    global are_motors_on
    global is_camera_on
    global is_obstacles_avoidance_on
    global relative_yaw_angle
    global target_speed
    global turn_angle_order
    global turn_speed_step

    while True:

        if debug_mode == True:
            print("Initiating bluetooth server")

        server_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        server_socket.bind(("", bluetooth.PORT_ANY))
        server_socket.listen(1)

        if debug_mode == True:
            print("Waiting for a bluetooth connection...")

        client_sock, address = server_socket.accept()

        if debug_mode == True:
            print("Got a bluetooth connection. Waiting for commands...")

        while True:

            try:
                rx_data      = client_sock.recv(32)
                decoded_data = rx_data.decode('ascii', 'ignore')
            except Exception as e:
                if debug_mode == True:
                    print("Bluetooth reception error or disconnection")
                    print("Error: " + str(e))
                    print("Restarting!...")
                break

            if decoded_data == "C1":

                if debug_mode == True:
                    print("Received CAMERA ON command")
                is_camera_on = True

            elif decoded_data == "C0":

                if debug_mode == True:
                    print("Received CAMERA OFF command")
                is_camera_on = False

            elif decoded_data == "M1":

                if debug_mode == True:
                    print("Received MOTORS ON command")
                are_motors_on = True

            elif decoded_data == "M0":

                if debug_mode == True:
                    print("Received MOTORS OFF command")
                are_motors_on = False

            elif decoded_data == "O1":

                if debug_mode == True:
                    print("Received AVOID OBSTACLES ON command")
                is_obstacles_avoidance_on = True

            elif decoded_data == "O0":

                if debug_mode == True:
                    print("Received AVOID OBSTACLES OFF command")
                is_obstacles_avoidance_on = False

            elif decoded_data == "L1":

                if debug_mode == True:
                    print("Received LEFT 1 command")
                turn_angle_order    = relative_yaw_angle + TURNING_ANGLE_STEP * 1
                turn_speed_step     = TURNING_SPEED_STEP
                target_speed        = 0.0

            elif decoded_data == "L2":

                if debug_mode == True:
                    print("Received LEFT 2 command")
                turn_angle_order    = relative_yaw_angle + TURNING_ANGLE_STEP * 2
                turn_speed_step     = TURNING_SPEED_STEP
                target_speed        = 0.0

            elif decoded_data == "R1":

                if debug_mode == True:
                    print("Received RIGHT 1 command")
                turn_angle_order    = relative_yaw_angle - TURNING_ANGLE_STEP * 1
                turn_speed_step     = -TURNING_SPEED_STEP
                target_speed        = 0.0

            elif decoded_data == "R2":

                if debug_mode == True:
                    print("Received RIGHT 2 command")
                turn_angle_order    = relative_yaw_angle - TURNING_ANGLE_STEP * 2
                turn_speed_step     = -TURNING_SPEED_STEP
                target_speed        = 0.0

            elif decoded_data == "F1":

                if debug_mode == True:
                    print("Received FORWARD 1 command")
                turn_angle_order = 0.0
                turn_speed_step  = 0.0
                target_speed     = TARGET_SPEED_STEP * 1

            elif decoded_data == "F2":

                if debug_mode == True:
                    print("Received FORWARD 2 command")
                turn_angle_order = 0.0
                turn_speed_step  = 0.0
                target_speed     = TARGET_SPEED_STEP * 1.4

            elif decoded_data == "B1":

                if debug_mode == True:
                    print("Received BACKWARD 1 command")
                turn_angle_order = 0.0
                turn_speed_step  = 0.0
                target_speed     = -TARGET_SPEED_STEP * 1

            elif decoded_data == "B2":

                if debug_mode == True:
                    print("Received BACKWARD 2 command")
                turn_angle_order = 0.0
                turn_speed_step  = 0.0
                target_speed     = - TARGET_SPEED_STEP * 1.4

            elif decoded_data == "S":

                if debug_mode == True:
                    print("Received STOP command")
                turn_angle_order = 0.0
                target_speed     = 0.0

            else:

                yaw   = rx_data[0]
                roll  = rx_data[1]
                pitch = rx_data[2]

                if debug_mode == True:
                    print("Received EVENT: {:6.2f} / {:6.2f} / {:6.2f}".format(roll, pitch , yaw))

                if 165 < pitch < 255:
                    forward_value = float(pitch - 165 - 90) / 90
                elif 0 < pitch < 90:
                    forward_value = float(pitch) / 90
                else:
                    forward_value = 0

                if 165 < roll < 255:
                    turn_value = float(roll - 165 - 90) / 90
                elif 0 < roll < 90:
                    turn_value = float(roll) / 90
                else:
                    turn_value = 0

                if (turn_value > -0.10) and (turn_value < 0.10):

                    turn_angle_order = 0.0
                    turn_speed_step  = 0.0
                    target_speed     = forward_value * TARGET_SPEED_STEP * 4

                else:

                    if turn_value < 0.0:
                        turn_angle_order = relative_yaw_angle - TURNING_ANGLE_STEP
                        turn_speed_step  = turn_value * TURNING_SPEED_STEP * 2
                    else:
                        turn_angle_order = relative_yaw_angle + TURNING_ANGLE_STEP
                        turn_speed_step  = turn_value * TURNING_SPEED_STEP * 2
                    target_speed = 0.0


def camera_control_thread():

    global debug_mode
    global is_camera_on
    global is_camera_recording

    if debug_mode == True:
        print("Initiating camera module")

    camera = picamera.PiCamera(resolution=streamer.IMAGE_WIDTH + 'x' + streamer.IMAGE_HEIGHT, framerate=12)

    while True:

        if is_camera_on and not is_camera_recording:

            if debug_mode == True:
                print("Camera starting to record")

            camera.start_recording(streamer.streaming_output, format='mjpeg')
            is_camera_recording = True

        elif not is_camera_on and is_camera_recording:

            if debug_mode == True:
                print("Camera stopping to record")

            camera.stop_recording()
            is_camera_recording = False

        else:

            time.sleep(0.1)


def streaming_control_thread():

    global debug_mode

    while True:

        if debug_mode == True:
            print("Initiating streaming server")

        try:
            address = ('', 8000)
            steamer = streamer.StreamingServer(address, streamer.StreamingHandler)
            steamer.serve_forever()
        except Exception as e:
            if debug_mode == True:
                print("Streaming server exception")
                print("Error: " + str(e))
                print("Restarting!...")


def print_help():

    print("")
    print("Enter 1 to select speed PID, 2 to select balance PID")
    print("Enter selected PID values like: 'p=12', 'i=0.15', d='100', w='0.05'")
    print("")
    print("Enter target speed value like        : 's=50'")
    print("Enter distance samples value like    : 'm=50'")
    print("Enter equilibrium angle value like   : 'a=-3.00'")
    print("Enter equilibrium limit value like   : 'e=0.30'")
    print("Enter complementary filter value like: 'k=0.95'")
    print("")
    print("Press c to display current PIDs' values")
    print("Press u to display current IMU's values")
    print("Press o to display other parameters")
    print("")
    print("Press q to go for operational mode")
    print("Press h to display this help")
    print("")


def debug_control_thread(imu_device, proximity_sensor, speed_pid_controller, balance_pid_controller):

    global debug_mode
    global complementary_filter_factor
    global distance_readings
    global distance_samples
    global equilibrium_angle
    global equilibrium_limit
    global filtered_pitch
    global relative_yaw_angle
    global target_speed
    global turn_angle_order
    global left_counter
    global right_counter

    selected_pid = 1

    print_help()

    while debug_mode == True:

        print("> ", end = '')

        user_input = input()

        if len(user_input) == 1:

            command = user_input[0]

            if command == '1':
                print("")
                print("Speed PID selected")
                print("")
                selected_pid = 1
            elif command == '2':
                print("")
                print("Balance PID selected")
                print("")
                selected_pid = 2
            elif command == 'c':
                print("")
                print("SPEED PID:")
                speed_pid_controller.print_pid_info()
                print("")
                print("BALANCE PID:")
                balance_pid_controller.print_pid_info()
                print("")
            elif command == 'u':
                print("")
                imu_device.print_imu_info()
                print("")
            elif command == 'o':
                print("")
                print("K filter          = {:6} / Distance samples  = {:6}".format(complementary_filter_factor, distance_samples  ))
                print("Equilibrium angle = {:6} / Equilibrium limit = {:6}".format(equilibrium_angle          , equilibrium_limit ))
                print("Target speed      = {:6.2f} / Turn angle order  = {:6.2f}".format(target_speed         , turn_angle_order  ))
                print("Filtered pitch    = {:6.2f} / Relative yaw      = {:6.2f}".format(filtered_pitch       , relative_yaw_angle))
                print("Left encoder      = {:6} / Right encoder     = {:6}".format(left_counter               , right_counter     ))
                proximity_sensor.print_proximity_info()
                print("")
            elif command == 'q':
                print("")
                print("***** GOING TO OPERATIONAL MODE *****")
                debug_mode = False
            elif command == 'h':
                print_help()

        elif len(user_input) > 2 and user_input[1] == '=':

            command = user_input[0]
            value   = float(user_input[2:])

            if command == 'p':
                if selected_pid == 1:
                    speed_pid_controller.set_kp(value)
                elif selected_pid == 2:
                    balance_pid_controller.set_kp(value)
            elif command == 'i':
                if selected_pid == 1:
                    speed_pid_controller.set_ki(value)
                elif selected_pid == 2:
                    balance_pid_controller.set_ki(value)
            elif command == 'd':
                if selected_pid == 1:
                    speed_pid_controller.set_kd(value)
                elif selected_pid == 2:
                    balance_pid_controller.set_kd(value)
            elif command == 'w':
                if selected_pid == 1:
                    speed_pid_controller.set_anti_wind_up(value)
                elif selected_pid == 2:
                    balance_pid_controller.set_anti_wind_up(value)
            elif command == 's':
                target_speed = value
            elif command == 'm':
                distance_samples  = value
                distance_readings = []
            elif command == 'a':
                equilibrium_angle = value
            elif command == 'e':
                equilibrium_limit = value
            elif command == 'k':
                complementary_filter_factor = value


def main():

    global debug_mode
    global are_motors_on

    if debug_mode == True:
        os.system("clear")
        print("")
        print("               -------           ")
        print("               | (O) |           ")
        print("             - ------- -         ")
        print("             |    //   |         ")
        print("     |-----|-------------|-----| ")
        print("     |      /- /(O) (O)\       | ")
        print("     |     ||  \   --  /       | ")
        print("     |-----|-------------|-----| ")
        print("     |     /=*|||||||||*=\     | ")
        print("     |-----|-------------|-----| ")
        print("     |          /   \          | ")
        print("     /-----\  ||     ||  /-----\ ")
        print("     |- - -|*****   *****|- - -| ")
        print("     |-----|*****   *****|-----| ")
        print("     |- - -|             |- - -| ")
        print("     \-----/             \-----/ ")
        print("")

    if debug_mode == True:
        print("*****    CURRENTLY RUNNING IN DEBUG MODE    *****")
    else:
        print("***** CURRENTLY RUNNING IN OPERATIONAL MODE *****")

    if are_motors_on == True:
        print("*****      MOTORS CURRENTLY TURNED ON       *****")
    else:
        print("*****      MOTORS CURRENTLY TURNED OFF      *****")

    print("")

    imu_device                 = imu.ImuDevice()
    proximity_sensor           = proximity.ProximitySensor(PROXIMITY_SENSOR_TRIGGER_PIN, PROXIMITY_SENSOR_ECHO_PIN)
    speed_pid_controller       = pid.Pid(SPEED_PID_KP  , SPEED_PID_KI  , SPEED_PID_KD  , SPEED_PID_TARGET  , SPEED_PID_MIN  , SPEED_PID_MAX  , SPEED_PID_WINDUP  )
    balance_pid_controller     = pid.Pid(BALANCE_PID_KP, BALANCE_PID_KI, BALANCE_PID_KD, BALANCE_PID_TARGET, BALANCE_PID_MIN, BALANCE_PID_MAX, BALANCE_PID_WINDUP)

    bluetooth_control = threading.Thread(target = bluetooth_control_thread, args = [])
    bluetooth_control.start()

    balance_control = threading.Thread(target = balance_control_thread, args = [imu_device, speed_pid_controller, balance_pid_controller])
    balance_control.start()

    obstacles_avoidance = threading.Thread(target = obstacles_avoidance_thread, args = [proximity_sensor])
    obstacles_avoidance.start()

    camera_control = threading.Thread(target = camera_control_thread, args = [])
    camera_control.start()

    streaming_control = threading.Thread(target = streaming_control_thread, args = [])
    streaming_control.start()

    if debug_mode == True:
        user_input_control = threading.Thread(target = debug_control_thread, args = [imu_device, proximity_sensor, speed_pid_controller, balance_pid_controller])
        user_input_control.start()

    bluetooth_control.join()
    balance_control.join  ()
    obstacles_avoidance.join()
    camera_control.join   ()
    streaming_control.join()

    if debug_mode == True:
        user_input_control.join()


if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print("Keyboard interrupt...")

    except Exception as e:
        print("Error: " + str(e))

    finally:
        RPi.GPIO.cleanup()
        os._exit(0)