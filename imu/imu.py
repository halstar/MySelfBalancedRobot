import i2c
import math
import time

# MPU6050 I2C bus/address
IMU_BUS     = 0x01
IMU_ADDRESS = 0x68

# MPU6050 registers
PWR_MGMT_1   = 0x6B
GYRO_CONFIG  = 0x18
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
XA_OFFSET_H  = 0x06
YA_OFFSET_H  = 0x08
ZA_OFFSET_H  = 0x0A
XG_OFFSET_H  = 0x13
YG_OFFSET_H  = 0x15
ZG_OFFSET_H  = 0x17

ACCELERATION_SCALE_FACTOR = 16384.0
GYROSCOPE_SCALE_FACTOR    =   131.0


class ImuDevice:

    def __init__(self):

        self.acceleration_x = 0
        self.acceleration_y = 0
        self.acceleration_z = 0

        self.gyroscope_x = 0
        self.gyroscope_y = 0
        self.gyroscope_z = 0

        self.acceleration_offset_x = 0
        self.acceleration_offset_y = 0
        self.acceleration_offset_z = 0

        self.gyroscope_offset_x = 0
        self.gyroscope_offset_y = 0
        self.gyroscope_offset_z = 0

        self.roll  = 0
        self.pitch = 0

        self.roll_rate  = 0
        self.pitch_rate = 0
        self.yaw_rate   = 0

        self.i2c_device = i2c.I2cDevice(IMU_BUS, IMU_ADDRESS)

        # Write power management register
        self.i2c_device.write_byte(PWR_MGMT_1, 0x01)

        # Write configuration register
        self.i2c_device.write_byte(CONFIG, 0x02)

        # Write gyroscope configuration register
        self.i2c_device.write_byte(GYRO_CONFIG, 0x00)

        # Write sample rate divider register
        #self.i2c_device.write_byte(SMPLRT_DIV, 0x27)
        self.i2c_device.write_byte(SMPLRT_DIV, 0x04)

    def __read_word__(self, register):

        # Acceleration and gyroscope data are 16-bit
        higher_byte = self.i2c_device.read_byte(register    )
        lower_byte  = self.i2c_device.read_byte(register + 1)

        # Concatenate higher and lower bytes
        value = (higher_byte << 8) + lower_byte

        # Get signed value
        if value > 32768:
            value = value - 65536
        return value

    def __write_word__(self, register, value):

        # Acceleration and gyroscope data are 16-bit
        self.i2c_device.write_byte(register    , value >> 8    )
        self.i2c_device.write_byte(register + 1, value & 0x00FF)

    def __get_x_rotation__(self):
        radians = math.atan2(self.acceleration_y, math.sqrt(self.acceleration_x ** 2 +
                                                            self.acceleration_z ** 2))
        return math.degrees(radians)

    def __get_y_rotation__(self):
        radians = math.atan2(self.acceleration_x, math.sqrt(self.acceleration_y ** 2 +
                                                            self.acceleration_z ** 2))
        return -math.degrees(radians)

    def reset(self):
        self.i2c_device.write_byte(PWR_MGMT_1, 0x81)
        time.sleep(1)
        self.i2c_device.write_byte(PWR_MGMT_1, 0x01)
        time.sleep(1)

    def reset_offsets(self):

        self.__write_word__(XA_OFFSET_H, 0)
        self.__write_word__(YA_OFFSET_H, 0)
        self.__write_word__(ZA_OFFSET_H, 0)

        self.__write_word__(XG_OFFSET_H, 0)
        self.__write_word__(YG_OFFSET_H, 0)
        self.__write_word__(ZG_OFFSET_H, 0)

        self.acceleration_offset_x = 0
        self.acceleration_offset_y = 0
        self.acceleration_offset_z = 0

        self.gyroscope_offset_x = 0
        self.gyroscope_offset_y = 0
        self.gyroscope_offset_z = 0

    def read_acceleration_data(self):
        self.acceleration_x = self.__read_word__(ACCEL_XOUT_H) - self.acceleration_offset_x
        self.acceleration_y = self.__read_word__(ACCEL_YOUT_H) - self.acceleration_offset_y
        self.acceleration_z = self.__read_word__(ACCEL_ZOUT_H) - self.acceleration_offset_z

    def read_gyroscope_data(self):
        self.gyroscope_x = self.__read_word__(GYRO_XOUT_H) - self.gyroscope_offset_x
        self.gyroscope_y = self.__read_word__(GYRO_YOUT_H) - self.gyroscope_offset_y
        self.gyroscope_z = self.__read_word__(GYRO_ZOUT_H) - self.gyroscope_offset_z

    def get_x_acceleration(self):
        return self.acceleration_x

    def get_y_acceleration(self):
        return self.acceleration_y

    def get_z_acceleration(self):
        return self.acceleration_z

    def get_x_gyroscope(self):
        return self.gyroscope_x

    def get_y_gyroscope(self):
        return self.gyroscope_y

    def get_z_gyroscope(self):
        return self.gyroscope_z

    def get_x_acceleration_scaled(self):
        return self.acceleration_x / ACCELERATION_SCALE_FACTOR

    def get_y_acceleration_scaled(self):
        return self.acceleration_y / ACCELERATION_SCALE_FACTOR

    def get_z_acceleration_scaled(self):
        return self.acceleration_z / ACCELERATION_SCALE_FACTOR

    def get_x_gyroscope_scaled(self):
        return self.gyroscope_x / GYROSCOPE_SCALE_FACTOR

    def get_y_gyroscope_scaled(self):
        return self.gyroscope_y / GYROSCOPE_SCALE_FACTOR

    def get_z_gyroscope_scaled(self):
        return self.gyroscope_z / GYROSCOPE_SCALE_FACTOR

    def compute_angles(self):
        self.roll  = self.__get_x_rotation__()
        self.pitch = self.__get_y_rotation__()

    def compute_rates(self):
        self.roll_rate  = self.get_x_gyroscope_scaled()
        self.pitch_rate = self.get_y_gyroscope_scaled()
        self.yaw_rate   = self.get_z_gyroscope_scaled()

    def get_x_acceleration_offset(self):
        return self.acceleration_offset_x

    def set_x_acceleration_offset(self, offset):
        self.acceleration_offset_x = offset

    def get_y_acceleration_offset(self):
        return self.acceleration_offset_y

    def set_y_acceleration_offset(self, offset):
        self.acceleration_offset_y = offset

    def get_z_acceleration_offset(self):
        return self.acceleration_offset_z

    def set_z_acceleration_offset(self, offset):
        self.acceleration_offset_z = offset

    def get_x_gyroscope_offset(self):
        return self.gyroscope_offset_x

    def set_x_gyroscope_offset(self, offset):
        self.gyroscope_offset_x = offset

    def get_y_gyroscope_offset(self):
        return self.gyroscope_offset_y

    def set_y_gyroscope_offset(self, offset):
        self.gyroscope_offset_y = offset

    def get_z_gyroscope_offset(self):
        return self.gyroscope_offset_z

    def set_z_gyroscope_offset(self, offset):
        self.gyroscope_offset_z = offset

    def get_roll(self):
        return self.roll

    def get_pitch(self):
        return self.pitch

    def get_roll_rate(self):
        return self.roll_rate

    def get_pitch_rate(self):
        return self.pitch_rate

    def get_yaw_rate(self):
        return self.yaw_rate

    def print_imu_info(self):

        print("X / Y / Z acceleration offsets: {} / {} / {}".format(self.acceleration_offset_x, self.acceleration_offset_y, self.acceleration_offset_z))
        print("X / Y / Z gyroscope    offsets: {} / {} / {}".format(self.gyroscope_offset_x   , self.gyroscope_offset_y   , self.gyroscope_offset_z   ))

        self.read_acceleration_data()
        self.read_gyroscope_data   ()
        self.compute_angles        ()

        print("")
        print("X / Y / Z acceleration raw data:  {} / {} / {}".format(self.acceleration_x, self.acceleration_y, self.acceleration_z))
        print("X / Y / Z gyroscope    raw data:  {} / {} / {}".format(self.gyroscope_x   , self.gyroscope_y   , self.gyroscope_z   ))
        print("")
        print("X / Y / Z acceleration scaled data: {:6.2f} / {:6.2f} / {:6.2f}".format(self.get_x_acceleration_scaled(), self.get_y_acceleration_scaled(), self.get_z_acceleration_scaled()))
        print("X / Y / Z gyroscope    scaled data: {:6.2f} / {:6.2f} / {:6.2f}".format(self.get_x_gyroscope_scaled()   , self.get_y_gyroscope_scaled()   , self.get_z_gyroscope_scaled()   ))
        print("")
        print("Roll : {:6.2f} - Pitch: {:6.2f}".format(self.roll, self.pitch))
