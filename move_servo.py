from comm_method import read_ini
import RPi.GPIO as GPIO
import Adafruit_PCA9685

class MoveServo:
    def __init__(self):
        self.read_ini_file()
        self.pwm_init()
        self.gpio_init()
        self.move(0) #Face front

    def read_ini_file(self):
        read_default = read_ini("move_servo.ini", "DEFAULT")
        self.servo_pwm_port = read_default.getint("servo_pwm_port")
        self.pulse_max = read_default.getint("pulse_max")
        self.pulse_min = read_default.getint("pulse_min")
    
    def pwm_init(self):
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
    
    def gpio_init(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
    
    def move(self, angle): #degree -90 ~ 90
        self.angle = max(-90, min(angle, 90))
        pulse = self.angle_to_pulse(self.angle)
        self.drive_servo(pulse)
    
    def move_diff(self, angle):
        self.angle += angle
        self.move(self.angle)

    def angle_to_pulse(self, angle):
        return (angle + 90.0) * (self.pulse_max - self.pulse_min) / 180.0 + self.pulse_min
    
    def drive_servo(self, pulse):
        pulse = int(pulse)
        self.pwm.set_pwm(self.servo_pwm_port, 0, pulse)
    
    def get_angle(self):
        return self.angle

if __name__ == "__main__":
    import time
    move_servo = MoveServo()
    #Face front
    print("front")
    move_servo.move(0)
    time.sleep(1.0)

    #Face right
    print("Right")
    move_servo.move(-90)
    time.sleep(1.0)

    #Face front
    print("front")
    move_servo.move(0)
    time.sleep(1.0)

    #Face left
    print("Left")
    move_servo.move(90)
    time.sleep(1.0)

    #Face front
    print("front")
    move_servo.move(0)
    time.sleep(1.0)

    print("end")