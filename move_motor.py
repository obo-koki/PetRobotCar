from comm_method import read_ini
import Adafruit_PCA9685
import RPi.GPIO as GPIO

class MoveMotor:
    def __init__(self):
        self.read_ini_file()
        self.pwm_init()
        self.gpio_init()
        self.move(0,0) #stop

    def read_ini_file(self):
        read_default = read_ini("move_motor.ini", "DEFAULT")
        self.left_drct_pin1 = read_default.getint("left_drct_pin1")
        self.left_drct_pin2 = read_default.getint("left_drct_pin2")
        self.right_drct_pin1 = read_default.getint("right_drct_pin1")
        self.right_drct_pin2 = read_default.getint("right_drct_pin2")
        self.left_speed_port = read_default.getint("left_speed_port")
        self.right_speed_port = read_default.getint("right_speed_port")
        self.vel_to_pusle = read_default.getfloat("vel_to_pulse")
        self.wheel_dist = read_default.getfloat("wheel_dist")
        self.max_pulse = read_default.getint("max_pulse")    
        self.turn_const = read_default.getfloat("turn_const")

    def pwm_init(self):
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
    
    def gpio_init(self):
        GPIO.setmode(GPIO.BCM) # GPIO number in BCM mode
        GPIO.setwarnings(False)
        GPIO.setup(self.left_drct_pin1, GPIO.OUT)   
        GPIO.setup(self.left_drct_pin2, GPIO.OUT) 
        GPIO.setup(self.right_drct_pin1, GPIO.OUT)   
        GPIO.setup(self.right_drct_pin2, GPIO.OUT) 

    def calc_left_vel(self, lin_vel, ang_vel):
        return lin_vel - self.turn_const * self.wheel_dist / 2.0 * ang_vel

    def calc_right_vel(self, lin_vel, ang_vel):
        return lin_vel + self.turn_const * self.wheel_dist / 2.0 * ang_vel

    def drive_motor(self, left_pulse, right_pulse):
        left_pulse = max(-self.max_pulse, min(left_pulse, self.max_pulse))
        right_pulse = max(-self.max_pulse, min(right_pulse, self.max_pulse))

        left_pulse = int(left_pulse)
        right_pulse = int(right_pulse)

        if left_pulse >= 0:
            GPIO.output(self.left_drct_pin1, GPIO.LOW)
            GPIO.output(self.left_drct_pin2, GPIO.HIGH)
        else:
            GPIO.output(self.left_drct_pin1, GPIO.HIGH)
            GPIO.output(self.left_drct_pin2, GPIO.LOW)
            
        if right_pulse >= 0:
            GPIO.output(self.right_drct_pin1, GPIO.LOW)
            GPIO.output(self.right_drct_pin2, GPIO.HIGH)
        else:
            GPIO.output(self.right_drct_pin1, GPIO.HIGH)
            GPIO.output(self.right_drct_pin2, GPIO.LOW)
        
        self.pwm.set_pwm(self.left_speed_port, 0, abs(left_pulse))
        self.pwm.set_pwm(self.right_speed_port, 0, abs(right_pulse))

    def move(self, lin_vel, ang_vel): #m/s, rad/s
        self.lin_vel = float(lin_vel)
        self.ang_vel = float(ang_vel)

        left_vel = self.calc_left_vel(self.lin_vel, self.ang_vel)
        right_vel = self.calc_right_vel(self.lin_vel, self.ang_vel)


        left_pulse = left_vel * self.vel_to_pusle
        right_pulse = right_vel * self.vel_to_pusle

        self.drive_motor(left_pulse, right_pulse)
    
    def get_lin_vel(self):
        return self.lin_vel
    
    def get_ang_vel(self):
        return self.ang_vel

if __name__ == "__main__":
    import time
    move_motor = MoveMotor()
    time.sleep(1.0)
    try:
        while True:
            ang_vel = float(input())
            move_motor.move(0.0, ang_vel)
            time.sleep(1.0)
            move_motor.drive_motor(0, 0)

    except KeyboardInterrupt:
        print("Keyboard Interrupt")

    ##forward
    #print("forward")
    #move_motor.move(0.5, 0)
    #time.sleep(1.0)

    ##backward
    #print("backward")
    #move_motor.move(-0.5, 0)
    #time.sleep(1.0)

    ##turn right
    #print("turn right")
    #move_motor.move(0, 5)
    #time.sleep(1.0)

    ##turn left
    #print("turn left")
    #move_motor.move(0, -5)
    #time.sleep(1.0)

    #move_motor.move(0, 0)
    #print("end")