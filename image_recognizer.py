from comm_method import read_ini
import picamera
import picamera.array
import cv2 as cv

class RecognizeInfo:
    def __init__(self, detected=False, size=None, angle_x=None, angle_y=None):
        self.detected = detected
        self.size = size
        self.angle_x = angle_x
        self.angle_y = angle_y

class ImageRecognizer:
    def __init__(self, recognize_file_path):
        self.read_ini_file()
        self.init_camera()
        self.read_recognize_file(recognize_file_path)
    
    def read_ini_file(self):
        read_default = read_ini("image_recognizer.ini", "DEFAULT")
        self.camera_width = read_default.getint("camera_width")
        self.camera_height = read_default.getint("camera_height")
        self.camera_angle_x = read_default.getint("camera_angle_x")
        self.camera_angle_y = read_default.getint("camera_angle_y")

    def init_camera(self):
        camera = picamera.PiCamera()
        camera.resolution = (self.camera_width, self.camera_height)
        self.camera = camera

    def read_recognize_file(self, recognize_file_path):
        self.cascade = cv.CascadeClassifier(recognize_file_path)
    
    def get_angle_x(self, rect):
        center_x = rect[0] + rect[2] / 2.0
        return self.camera_angle_x / 2.0 - center_x / self.camera_width * self.camera_angle_x

    def get_angle_y(self, rect):
        center_y = rect[1] + rect[3] / 2.0
        return self.camera_angle_y / 2.0 - center_y / self.camera_height * self.camera_angle_y

    def get_recognize_info(self, show_flag=False):
        stream = picamera.array.PiRGBArray(self.camera)
        self.camera.capture(stream, 'bgr', use_video_port=True)
        grayimg = cv.cvtColor(stream.array, cv.COLOR_BGR2GRAY)
        rects = self.cascade.detectMultiScale(grayimg, scaleFactor=1.2, minNeighbors=2, minSize=(50, 50))

        recognize_info = RecognizeInfo()
        if len(rects) > 0:
            recognize_info.detected = True
            size_max = 0
            for rect in rects:
                size = rect[2] * rect[3]
                if size > size_max:
                    angle_x = self.get_angle_x(rect)
                    angle_y = self.get_angle_y(rect)
                    rect_max = rect
                    size_max = size

            cv.rectangle(stream.array, tuple(rect_max[0:2]), tuple(rect_max[0:2]+rect_max[2:4]), (0, 0, 255), thickness=3)

            recognize_info.size = size_max
            recognize_info.angle_x = angle_x
            recognize_info.angle_y = angle_y

        if show_flag:
            cv.imshow('camera', stream.array)
            key = cv.waitKey(1)

        stream.seek(0)
        stream.truncate()

        return recognize_info

if __name__ == "__main__":
    hand_recognizer = ImageRecognizer("hand.xml")
    try:
        while True:
            hand_info = hand_recognizer.get_recognize_info(show_flag=True)
            print("Hand detected: ", hand_info.detected, 
            " size: ", hand_info.size, " angle_x: ", hand_info.angle_x,
            " angle_y: ", hand_info.angle_y)

    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        cv.destroyAllWindows()