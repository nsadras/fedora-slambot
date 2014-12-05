import rospy, i2c, smbus, time

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
    """ 
    gyro scale register values
    FS_SEL | Full Scale Range   | LSB Sensitivity
    -------+--------------------+----------------
    0      | +/- 250 degrees/s  | 131 LSB/deg/s
    1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
    2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
    3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
    
    accel scale register values
    AFS_SEL | Full Scale Range | LSB Sensitivity
    --------+------------------+----------------
    0       | +/- 2g           | 8192 LSB/mg
    1       | +/- 4g           | 4096 LSB/mg
    2       | +/- 8g           | 2048 LSB/mg
    3       | +/- 16g          | 1024 LSB/mg  
    """
    i2c.write(bus, gyro, 0x6b, 0x00) # activate sensor (asleep by default) 
    i2c.write(bus, gyro, 0x1b, 0x00) # gyro scale register
    i2c.write(bus, gyro, 0x1c, 0x00) # accel scale register 
    
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
        self.velocity = 0
        self.last_sample = time.clock()

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
            gyro_low = i2c.read(self.bus, self.gyro, 0x48)
            gyro_raw = (gyro_high << 8) | gyro_low
            angular = to_signed_int(gyro_raw) / 131.
            return angular
        except IOError as e:
            return None

    def get_linear_velocity(self):
        try:
            accel_high = i2c.read(self.bus, self.gyro, 0x3d)
            accel_low = i2c.read(self.bus, self.gyro, 0x3e)
            accel_raw = (accel_high << 8) | accel_low 
            a = ((to_signed_int(accel_raw) - 63.7) / 16384.) * 9.81
            dt = time.clock() - self.last_sample
            self.velocity = self.velocity + dt*a
            self.last_sample = time.clock() 
            return self.velocity
        except IOError as e:
            return None

    def stop(self):
        self.move(0,0)
