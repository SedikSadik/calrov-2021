### IMPORTS
##GUI
from os import stat
from posix import posix_spawn
from tkinter import *
from PIL import ImageTk, Image
import cv2
##SYSTEM IMPORT
import threading
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst
from time import sleep, time
from pymavlink import mavutil
from math import pi as PI
import numpy as np
import threading
import datetime
import os.path




### TINTER INITIALIZING
root=Tk()
root.title("CALROV GUI")
##Icon
icon = Image.open(os.path.abspath('./GUI/gui_images/calrov_logo.jpg'))
icon = ImageTk.PhotoImage(icon)
root.tk.call('wm','iconphoto',root._w, icon)

## TITLE TEXT
Title_label = Label(root, text = "CALROV")
Title_label.config(font =("Courier", 20))
Title_label.grid(row=0, column=0)

##Live Video Display Box
video_app = Frame(root, bg="white")
video_app.grid(row=1,column=0, rowspan=12)


video_label_yolo = Label(video_app)
video_label_yolo.grid(row=0, column=0, rowspan=5)
video_label_opencv = Label(video_app)
video_label_opencv.grid(row=6, column=0, rowspan=5)



yolo_fps_label = Label(video_app, text="Yolo Fps: 0")
yolo_fps_label.grid(row=5, column=0)

opencv_fps_label = Label(video_app, text="Opencv Fps: 0")
opencv_fps_label.grid(row=11, column=0)

### FUNCTION AND CLASS DEFINITIONS
class Video():

    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
    """

    def __init__(self, port=5600):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self._frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        """ Get Frame

        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """Check if frame is available

        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)

    def run(self):
        """ Get frame to update _frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame

        return Gst.FlowReturn.OK
def video_main():
    start_time = time()
    global recent_boxes
    if video_on:
        
        _, frame = cap.read()
        """cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) #BGR RGB dönüşümü
        height, width = (int(cv2image.shape[1]*img_scale),int(cv2image.shape[0]*img_scale))

        scaled_img = cv2.resize(cv2image,(height, width))
        """
        # detected_image, recent_boxes = yolo_detection(frame)
        # detected_image = cv2.cvtColor(detected_image,cv2.COLOR_BGR2RGB)
        # img = Image.fromarray(detected_image)
        frame = cv2.resize(frame, (416,416))
        img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        imgtk = ImageTk.PhotoImage(image=img)
        video_label_yolo.imgtk = imgtk
        video_label_yolo.configure(image=imgtk)
        # video_label_opencv.imgtk = imgtk
        # video_label_opencv.configure(image=imgtk)
        yolo_fps_label.config(text=f"FPS: {round(1.0/(time()-start_time))}")

        # pwm_decide_once(detected_image, recent_boxes)
    video_label_yolo.after(1,video_main)
class Vehicle():
    def __init__(self) -> None:
        pass
    def attitude_update() ->None:
        pass
    def depth_update() -> None:
        pass

###  GLOBAL VARIABLES AND OBJECTS
video = Video(port=4777)
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

recent_boxes = []

video_on = True




###BUTTONS
button_frame = Frame(root, bg="white")
button_frame.config(width=416,height=416)
toggle_stop_start = Button(button_frame, text='Toggle Stop/Start')
toggle_arm_disarm = Button(button_frame, text='Toggle Arm/Disarm')

servo_open = Button(button_frame, text='Open Servo')
servo_close = Button(button_frame, text='Close Servo')

yolo_video_button = Button(button_frame, command=threading.Thread(target=video_main).start, text='Toggle YOLO Display')
opencv_video_button = Button(button_frame, text='Toggle Opencv Display')

toggle_attitude = Button(button_frame, text='Toggle Attitude Display')
button6 = Button(button_frame, text='button1')

## BUtton Display
button_frame.grid(row=0, column=1, rowspan=8)



toggle_stop_start.grid(row=0)
toggle_arm_disarm.grid(row=1)

servo_open.grid(row=2)
servo_close.grid(row=3)

yolo_video_button.grid(row=4)
opencv_video_button.grid(row=5)

toggle_attitude.grid(row=6)
button6.grid(row=7)

### VEHICLE STATUS DISPLAY
status_frame = Frame(root)
status_frame.config( bg='gray')
status_frame.grid(row=8, column=1, rowspan=6)

roll_label = Label(status_frame, text="Roll: N/A")
pitch_label = Label(status_frame, text="Pitch: N/A")
yaw_label = Label(status_frame, text="Yaw: N/A")
depth_label = Label(status_frame, text="Depth: N/A")

servo_label = Label(status_frame, text="Servo: N/A")
recognition_label = Label(status_frame, text="Object Recognition: N/A")



## STATUS DISPLAY
roll_label.grid()
pitch_label.grid()
yaw_label.grid()
depth_label.grid()


servo_label.grid()
recognition_label.grid()


if __name__ == "__main__":
    root.mainloop()
