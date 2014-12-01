import sys
from subprocess import call, check_output

def read(address, register):
    result = check_output(["sudo", "i2cget", "-y", "1", str(address), str(register)])
    result.strip("\n")
    return result

def write(address, register, value):
    print address, register, value
    call(["i2cset", "-y", "1", str(address), str(register), str(value)])

def writel(address, register, value):
    print address, register, value
    call(["i2cset", "-y", "1",  str(address), str(register), str(value), "w"])



