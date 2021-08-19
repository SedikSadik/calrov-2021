import threading
from tkinter import *
from PIL import ImageTk, Image
import cv2
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst
import numpy as np
from time import sleep, time
from pymavlink import mavutil
from math import pi as PI
import threading
import datetime
import os.path
from random import random

master = mavutil.mavlink_connection("udpin:192.168.2.1:14550")
master.wait_heartbeat()
print("Successful Connection!")


cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

root=Tk()
root.title("CALROV GUI")

#Icon
icontmp = Image.open(os.path.abspath('./GUI/gui_images/calrov_logo.jpg'))
icon = ImageTk.PhotoImage(icontmp)
root.tk.call('wm','iconphoto',root._w, icon)
# TITLE TEXT
l = Label(root, text = "CALROV")
l.config(font =("Courier", 14))
l.grid(row=0, column=0, columnspan=4)

#Live Video Display
app = Frame(root, bg="white")
video_label = Label(app)
app.grid(row=1,column=0, columnspan=4)
video_label.grid()

fps_label = Label(root, text="Fps: 0")
fps_label.grid(row=6, column=1)


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

video = Video(port=4777)
recent_boxes = []

def pwm_decide_once(detected_image,recent_boxes):
    try:

        tlx,tly,w,h= recent_boxes[0]
        imgWidth, imgHeight, _ = detected_image.shape
        
        imgMidx, imgMidy = imgWidth/2 , imgHeight/2
        print(imgWidth)
        if tlx<imgMidx<tlx+w:
            print("in the middle")
        elif imgMidx>tlx+w:
            print("on the right")
        elif imgMidx<tlx:
            print("on the left")
    except:
        print("not found")

def video_main():
    global recent_boxes
    if video_update and video.frame_available:
        
        frame = video.frame()
    
        """cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) #BGR RGB dönüşümü
        height, width = (int(cv2image.shape[1]*img_scale),int(cv2image.shape[0]*img_scale))

        scaled_img = cv2.resize(cv2image,(height, width))
        """
        detected_image, recent_boxes = yolo_detection(frame)
        detected_image = cv2.cvtColor(detected_image,cv2.COLOR_BGR2RGB)
        img = Image.fromarray(detected_image)
        
        #img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk
        video_label.configure(image=imgtk)
        pwm_decide_once(detected_image, recent_boxes)
    video_label.after(1,video_main)

cv2.createButton()
###Attitude Info
roll_label = Label(root, text= 'Roll: Default',font =("Courier", 14))
pitch_label = Label(root, text= 'Pitch: Default',font =("Courier", 14))
yaw_label = Label(root, text= 'Pitch: Default',font =("Courier", 14))
##Attitude Config

#Packing Attitude
roll_label.grid(row=2, column=0)
pitch_label.grid(row=2, column=1)
yaw_label.grid(row=2, column=2)




##Message Interval
def request_message_interval(message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # MAVLINK mesaj tanimi
        1e6 / frequency_hz, # Istenen frekans
        0, # Hedef, (0=arac)
        0, 0, 0, 0)

# Configure AHRS2 message to be sent at 1Hz
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 10)

# Configure ATTITUDE message to be sent at 2Hz
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10)


def attitude_tk():
    
    '''
    global roll_value
    global pitch_value
    global yaw_value
    '''
    if attitude_update:
        try:
            
            rcvpacket = master.recv_match().to_dict()

            if rcvpacket['mavpackettype']=='ATTITUDE' or rcvpacket['macpackettype']=='AHRS2':
                roll_value= int(100* rcvpacket['roll'])
                pitch_value = int(100* rcvpacket['pitch'])
                yaw_value = int(100* rcvpacket['yaw'])

                roll_label.config(text=f'Roll: {roll_value}')
                pitch_label.config(text=f'Pitch: {pitch_value}')
                yaw_label.config(text=f'Yaw: {yaw_value}')
                    #tuple halinde istenen verilerin alınması
        except:
            pass
    
    '''
    roll_label.config(text=str(datetime.datetime.now()))
    pitch_label.config(text='')
    yaw_label.config(text='hehe')
    roll_label.after(1, attitude_tk)
    '''
    
    roll_label.after(1, attitude_tk)

### YOLO and DETECTION
## Loading Yolo
net = cv2.dnn.readNet(os.path.abspath('./GUI/Yolo_files/yolov4-tiny.weights'),os.path.abspath('./GUI/Yolo_files/yolov4-tiny.cfg'))
detection_classes = []
with open(os.path.abspath('./GUI/Yolo_files/coco.names'), 'r') as f:
    detection_classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0]-1] for i in net.getUnconnectedOutLayers()]

#print(detection_classes)

attitude_update=False
video_update = False

def yolo_detection(raw_image):
    """Take in as input a cv2 image"""
    class_ids = []
    confidences = []
    boxes = []
    height , width, _ = raw_image.shape
    blob = cv2.dnn.blobFromImage(raw_image, 0.00392, (416,416), (0,0,0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                center_x = int(detection[0]*width)
                center_y = int(detection[1]*height)
                w = int(detection[2]*width)
                h = int(detection[3]*height)
                ##Rectangle Draw
                topleft_x = int(center_x-(w/2))
                topleft_y = int(center_y-(h/2))

                boxes.append([topleft_x,topleft_y,w,h])
                confidences.append(float(confidence))
                class_ids.append(class_id)
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    #DISPLAY DETECTION
    total_detections = len(boxes)
    for i in range(total_detections):
        if i in indexes:
            topleft_x, topleft_y, w,h = boxes[i]
            label = detection_classes[class_ids[i]]
            cv2.rectangle(raw_image, (topleft_x,topleft_y), (topleft_x+w,topleft_y+h), (0,100,255), 1)
            cv2.putText(raw_image, label, (topleft_x, topleft_y),cv2.FONT_HERSHEY_COMPLEX,1,(0,165,255))


    return raw_image, boxes


def send_pwm(x =0, y=0 , z = 500, roll=0 , buttons=0):
    """Send manual pwm to the axis of a joystick. 
    Relative to the vehicle
    x for right-left motion
    y for forward-backwards motion
    z for up-down motion
    r for the yaw axis
        clockwise is -1000
        counterclockwise is 1000
    buttons is an integer with 
    """
    master.mav.manual_control_send(master.target_system, x,y,z,roll,buttons)

fps_label = Label(root, text="Fps: 0")
fps_label.grid(row=7, column=1)

Current_task = Label(root, text="Gorev Cubugu")

Current_task.grid(row=5, column=1, columnspan=3)
master.arducopter_arm()
master.motors_armed_wait()
moving = False
def pwm_movement():
    start_time = time()
    while time()<start_time+10:
        try:
            tlx,tly,w,h= recent_boxes[0]
            if tlx<208<tlx+w:
                
                Current_task.config(text="EFE BURADA, ONU YAKALA!!!!")
                send_pwm(x=500, z=500, roll=0)
            elif tlx+w<208:
                Current_task.config(text="EFE SOLDA")
                send_pwm( x=300, z=500,roll=-100)
            elif tlx>208:
                Current_task.config(text="EFE sagda")
                send_pwm(x=300, roll=100)
            else:
                send_pwm(roll=100)
                Current_task.config(text="meh")
        except:
            pass


def toggle_video():
    global video_update
    video_update = not video_update
def toggle_attitude():
    global attitude_update
    attitude_update = not attitude_update
def toggle_movement():
    global moving
    moving = not moving
def reset_function():
    pass

def toggle_arm():
    if master.motors_armed():
        master.arducopter_disarm()
        master.motors_disarmed_wait()
    else:
        master.arducopter_arm()
        master.motors_armed_wait()

###Threads
reset_button = Button(root, command=threading.Thread(target=pwm_movement).start, text="pwmmovemet")
video_button = Button(root, command=threading.Thread(target=video_main).start, text='Video Start')
attitude_button = Button(root, command=threading.Thread(target=attitude_tk).start, text="Attitude Start")
toggle_video_button = Button(root, command=toggle_video, text="Toggle Video")
toggle_attitude_button = Button(root, command=toggle_attitude, text="Toggle Attitude")
arm_button = Button(root, command=toggle_arm, text='arm-disarm')
#efe_button = Button(root, command=pwm_movement, text="Efeyi ara")


#Button Placement
reset_button.grid(row=4, column=0)
video_button.grid(row=4, column=1)
attitude_button.grid(row=4, column=2)
toggle_attitude_button.grid(row=5, column=0)
toggle_video_button.grid(row=5,column=1)
arm_button.grid(row=5, column=2)
#efe_button.grid(row=6, column=0)


"""videothread = threading.Thread(target=video_main)
tk_main_thread = threading.Thread(target=attitude_tk)
root_thread = threading.Thread(target=root.mainloop)
"""
#
if __name__ == '__main__':
    root.mainloop()
    