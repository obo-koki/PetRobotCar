from comm_method import read_ini
from human_detector import HumanDetector
from image_recognizer import ImageRecognizer, RecognizeInfo
from move_motor import MoveMotor
from move_servo import MoveServo
from enum import Enum
from time import sleep
from pid import PID
import cv2 as cv

class State(Enum):
    STOP = 0
    GET_CLOSER = 1
    PLEASURE = 2
    ANGRY = 3

class MotionMaker:
    def __init__(self):
        self.read_ini_file()
        self.move_motor = MoveMotor()
        self.move_servo = MoveServo()
        self.human_detector = HumanDetector()
        self.hand_detector = ImageRecognizer("hand.xml")
        self.face_PID_control = PID(self.face_Kp, self.face_Ki, self.face_Kd, self.control_rate)
        self.turn_PID_control = PID(self.turn_Kp, self.turn_Ki, self.turn_Kd, self.control_rate)
        self.forward_PID_control = PID(self.forward_Kp, self.forward_Ki, self.forward_Kd, self.control_rate)

    def read_ini_file(self):
        read_default = read_ini("motion_maker.ini", "DEFAULT")
        self.hand_is_near = read_default.getint("hand_is_near")
        self.ideal_hand_size = read_default.getint("ideal_hand_size")
        self.control_rate = read_default.getfloat("control_rate")
        self.face_Kp = read_default.getfloat("face_Kp")
        self.face_Ki = read_default.getfloat("face_Ki")
        self.face_Kd = read_default.getfloat("face_Kd")
        self.turn_Kp = read_default.getfloat("turn_Kp")
        self.turn_Ki = read_default.getfloat("turn_Ki")
        self.turn_Kd = read_default.getfloat("turn_Kd")
        self.forward_Kp = read_default.getfloat("forward_Kp")
        self.forward_Ki = read_default.getfloat("forward_Ki")
        self.forward_Kd = read_default.getfloat("forward_Kd")
    
    def stop(self):
        self.move_motor.move(0, 0)
        self.move_servo.move(0)
        sleep(0.1)
    
    def get_closer(self, hand_info):
        # Face control
        hand_angle = hand_info.angle_x
        if not (hand_angle == None):
            servo_control_value = self.face_PID_control.get_control_value(-hand_angle)
            self.move_servo.move_diff(servo_control_value)
        
        # Turn control
        #face_angle = self.move_servo.get_angle()
        #ang_vel = self.turn_PID_control.get_control_value(-face_angle)
        #self.move_motor.move(0, ang_vel)

        ## Forward control
        #hand_size = hand_info.size
        #if hand_size == None:
            #lin_vel = 0.0
        #else:
            #distance = self.ideal_hand_size - hand_size
            #lin_vel = self.forward_PID_control.get_control_value(-distance)
        
        #self.move_motor.move(lin_vel, ang_vel)


    def pleasure(self):
        pre_angle = self.move_servo.get_angle()
        # Face right
        self.move_servo.move(-45)
        sleep(1)
        # Face left
        self.move_servo.move(45)
        sleep(1)
        self.move_servo.move(pre_angle)
        pass

    def angry(self):
        # Turn invert direction and Face front
        self.move_servo.move(0)
        self.move_motor.move(0.0, 1.57)
        sleep(1)
        self.stop()
        # Go back
        self.move_motor.move(-0.5, 0.0)
        sleep(0.5)
        pass
    
    def run(self, show_flag = False):
        state = State.STOP
        try:
            while True:
                if state == State.STOP:
                    self.stop()
                    state = State.GET_CLOSER

                elif state == State.GET_CLOSER:
                    hand_info = self.hand_detector.get_recognize_info(show_flag)
                    self.get_closer(hand_info)

                    human_result = self.human_detector.get_result_edge()
                    if human_result:
                        if hand_info.size > self.hand_is_near:
                            state = State.PLEASURE
                        else:
                            state = State.GET_CLOSER

                elif state == State.PLEASURE:
                    self.pleasure()
                    state = State.STOP

                elif state == State.ANGRY:
                    self.angry()
                    state = State.STOP

                sleep(self.control_rate)
        
        except KeyboardInterrupt:
            print("Keyboard Interrupt")
            self.stop()
            if show_flag:
                cv.destroyAllWindows()
            return

if __name__ == "__main__":
    motion_maker = MotionMaker()
    motion_maker.run(show_flag = True)