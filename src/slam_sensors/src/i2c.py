import sys, smbus
from subprocess import call, check_output

MOTOR_L = 0x0a
MOTOR_R = 0x0b

def read(address, register):
    """ read a byte """
    result = check_output(["sudo", "i2cget", "-y", "1", str(address), str(register)])
    result.strip("\n")
    return result

def readl(address, register):
    """ read a word """
    result = check_output(["sudo", "i2cget", "-y", "1", str(address), str(register), "w"])
    result.strip("\n")
    return result

"""
def write(address, register, value):
    print address, register, value
    call(["i2cset", "-y", "1", str(address), str(register), str(value)])

def writel(address, register, value):
    print address, register, value
    call(["i2cset", "-y", "1",  str(address), str(register), str(value), "w"])
"""

def write(address, register, value):
    bus = smbus.SMBus(1)
    bus.write_byte_data(address, register, value)

def writel(address, register, value):
    bus = smbus.SMBus(1)
    bus.write_word_data(address, register, value)
