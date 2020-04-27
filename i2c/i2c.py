import smbus


class I2cDevice:

    def __init__(self, bus, address):
        self.bus     = smbus.SMBus(bus)
        self.address = address

    def read_byte(self, register):
        return self.bus.read_byte_data(self.address, register)

    def write_byte(self, register, value):
        self.bus.write_byte_data(self.address, register, value)
