from comm_method import read_ini
import RPi.GPIO as GPIO

class HumanDetector:
    def __init__(self):
        self.read_ini_file()
        self.init_gpio()
        self.pre_result = False
    
    def read_ini_file(self):
        read_default = read_ini("human_detector.ini", "DEFAULT")
        self.pin_num = read_default.getint("pin_num")

    def init_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_num, GPIO.IN)
    
    def get_result(self):
        return (GPIO.input(self.pin_num) == GPIO.HIGH)
    
    def get_result_edge(self):
        result = False
        cur_result = (GPIO.input(self.pin_num) == GPIO.HIGH)
        if(self.pre_result == False and cur_result == True):
            result = True
        self.pre_result = cur_result
        return result

if __name__ == "__main__":
    from time import sleep
    human_detector = HumanDetector()
    try:
        while True:
            result = human_detector.get_result()
            print("human detect : ", result)
            sleep(1)
            pass

    except KeyboardInterrupt:
        print("Keyboard Interrupt")