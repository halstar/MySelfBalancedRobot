import os
import json
import imu

SETUP_FILE = 'setup.json'

CALIBRATION_LOOP_COUNT = 10000

MOVING_STAR_PATTERN = ['/', '-', '\\', '|']


def main():

    os.system("clear")

    print("")
    print("/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\ ")
    print("|  IMU calibration starting... | ")
    print("\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/ ")
    print("")

    with open(SETUP_FILE, 'r') as json_file:
        setup_data = json.load(json_file)

    imu_device = imu.ImuDevice()

    imu_device.reset        ()
    imu_device.reset_offsets()

    imu_device.print_imu_info()

    x_acceleration_measure = 0
    y_acceleration_measure = 0
    z_acceleration_measure = 0

    x_gyroscope_measure = 0
    y_gyroscope_measure = 0
    z_gyroscope_measure = 0

    print("  ")

    for i in range(0, CALIBRATION_LOOP_COUNT):
        print("\b" + MOVING_STAR_PATTERN[i % 4], end = '', flush = True)

        imu_device.read_acceleration_data()
        imu_device.read_gyroscope_data   ()

        x_acceleration_measure += imu_device.get_x_acceleration()
        y_acceleration_measure += imu_device.get_y_acceleration()
        z_acceleration_measure += imu_device.get_z_acceleration()

        x_gyroscope_measure += imu_device.get_x_gyroscope()
        y_gyroscope_measure += imu_device.get_y_gyroscope()
        z_gyroscope_measure += imu_device.get_z_gyroscope()

    x_acceleration_measure /= CALIBRATION_LOOP_COUNT
    y_acceleration_measure /= CALIBRATION_LOOP_COUNT
    z_acceleration_measure /= CALIBRATION_LOOP_COUNT

    x_gyroscope_measure /= CALIBRATION_LOOP_COUNT
    y_gyroscope_measure /= CALIBRATION_LOOP_COUNT
    z_gyroscope_measure /= CALIBRATION_LOOP_COUNT

    x_acceleration_offset = int(x_acceleration_measure)
    y_acceleration_offset = int(y_acceleration_measure)
    z_acceleration_offset = int(z_acceleration_measure) - 16384

    x_gyroscope_offset = int(x_gyroscope_measure)
    y_gyroscope_offset = int(y_gyroscope_measure)
    z_gyroscope_offset = int(z_gyroscope_measure)

    imu_device.set_x_acceleration_offset(x_acceleration_offset)
    imu_device.set_y_acceleration_offset(y_acceleration_offset)
    imu_device.set_z_acceleration_offset(z_acceleration_offset)

    imu_device.set_x_gyroscope_offset(x_gyroscope_offset)
    imu_device.set_y_gyroscope_offset(y_gyroscope_offset)
    imu_device.set_z_gyroscope_offset(z_gyroscope_offset)

    imu_device.print_imu_info()

    setup_data['ACCELERATION_X_OFFSET'] = x_acceleration_offset
    setup_data['ACCELERATION_Y_OFFSET'] = y_acceleration_offset
    setup_data['ACCELERATION_Z_OFFSET'] = z_acceleration_offset

    setup_data['GYROSCOPE_X_OFFSET'] = x_gyroscope_offset
    setup_data['GYROSCOPE_Y_OFFSET'] = y_gyroscope_offset
    setup_data['GYROSCOPE_Z_OFFSET'] = z_gyroscope_offset

    with open(SETUP_FILE, "w") as json_file:
        json.dump(setup_data, json_file, indent=4)

    print("")
    print("/\/\/\/\/\/\/\/\/\/\/\/\/\ ")
    print("| IMU calibration done!  | ")
    print("\/\/\/\/\/\/\/\/\/\/\/\/\/ ")
    print("")

if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print("Keyboard interrupt...")

    except Exception as e:
        print("Error: " + str(e))
