import sys, smbus
from subprocess import call, check_output

MOTOR_L = 0x0a
MOTOR_R = 0x0b


def read(bus, address, register):
    return bus.read_byte_data(address, register)

def readl(bus, address, register):
    return bus.read_word_data(address, register)


def write(bus, address, register, value):
    bus.write_byte_data(address, register, value)

def writel(bus, address, register, value):
    bus.write_word_data(address, register, value)
