import rospy, i2c, smbus

MOTOR_L = 0x0a
MOTOR_R = 0x0b
GYRO = 0x68

def to_signed_int(bits):
    if not (bits >> 15) & 1:
        return bits
    else:
        return -(((~bits) & 0xffff) + 1)

def init_motor(bus, motor):
    i2c.write(bus, motor, 0x01, 0x13)

def init_gyro(bus, gyro):
    i2c.write(bus, gyro, 0x6b, 0x00) 
    
def set_speed(bus, motor, speed):
    if (speed < 0):
        speed = ((abs(speed) ^ 0xffff) + 1) & 0xffff
    i2c.writel(bus, motor, 0x04, 0x0000)
    i2c.writel(bus, motor, 0x06, speed)
    i2c.writel(bus, motor, 0x90, 0x0004)
    i2c.writel(bus, motor, 0x80, 0xffff)
    i2c.write(bus, motor, 0x08, 0x00)

class Robot:
    def __init__(self, motor_left, motor_right, gyro):
        self.bus = smbus.SMBus(1)
        self.motor_left = motor_left
        self.motor_right = motor_right
        self.gyro = gyro
        init_motor(self.bus, self.motor_left)
        init_motor(self.bus, self.motor_right)
        init_gyro(self.bus, self.gyro) 

        self.speed_left = None
        self.speed_right = None

    def move(self, speed_left, speed_right):
        if self.speed_left != speed_left:
            set_speed(self.bus, self.motor_left, speed_left)
            self.speed_left = speed_left
        if self.speed_right != speed_right:
            set_speed(self.bus, self.motor_right, speed_right)
            self.speed_right = speed_right

    def get_angular_velocity(self):
        try:
            gyro_high = i2c.read(self.bus, self.gyro, 0x47)
            gyro_low = i2c.read(self.bus, self.gyro, 0x48)[2:]
            gyro_raw = int(gyro_high + gyro_low, 16)
            angular = (to_signed_int(gyro_raw) / 114)*.0174 # convert from raw data to radians per second	
            return angular
        except IOError as e:
            return None

    def get_linear_velocity(self):
        try:
            accel_high = i2c.read(self.bus, self.gyro, 0x3d)
            accel_low = i2c.read(self.bus, self.gyro, 0x3e)[2:]
            accel_raw = int(accel_high + accel_low, 16)
            linear = (to_signed_int(accel_raw))
            return linear
        except IOError as e:
            return None



    def stop(self):
        self.move(0,0)
