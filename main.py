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
from random import random, randint
import sys

master = mavutil.mavlink_connection("udpin:192.168.2.1:14550")
master.wait_heartbeat()
print("Successful connection")
boot_time = time()
#-------------------------------
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
video_on = True
def yoloVideo():
    frame_start_time = time()
    
    global recent_boxes

    if video_on:
        
        if video.frame_available():
            frameYolo = video.frame()
            frameYolo = cv2.resize(frameYolo, (416,416))
            """cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) #BGR RGB dönüşümü
            height, width = (int(cv2image.shape[1]*img_scale),int(cv2image.shape[0]*img_scale))
            scaled_img = cv2.resize(cv2image,(height, width))
            """
            detected_image, recent_boxes = yolo_detection(frameYolo)
            
            detected_image = cv2.cvtColor(detected_image , cv2.COLOR_BGR2RGB)
            imgYolo = Image.fromarray(detected_image)

            #img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            imgtkYolo = ImageTk.PhotoImage(image=imgYolo)
            yolo_video.imgtk = imgtkYolo
            yolo_video.configure(image=imgtkYolo)
            pwm_decide_once(detected_image, recent_boxes)
            
            yolo_fps = 1.0/(time()-frame_start_time)
            yolo_fps_label.config(text=f"Fps: {yolo_fps}")
        yolo_video.after(1,yoloVideo)
    else:
        yolo_video_thread.join()


lower_hsv = np.array([21, 76, 106])
upper_hsv = np.array([180, 184, 147])
def opencvVideo():
    opencv_start_time = time()
    if video_on:
        
        if video.frame_available():
            frameCV = video.frame_available()
            frameCV = cv2.resize(frameCV, (416,416))
            hsvCV = cv2.cvtColor(frameCV, cv2.COLOR_BGR2HSV)
            
            
            maskCV = cv2.inRange(hsvCV, lower_hsv, upper_hsv)

            resultArrayCV = cv2.bitwise_and(frameCV, frameCV, mask=maskCV)

            RGBresultArrayCV = cv2.cvtColor(resultArrayCV, cv2.COLOR_BGR2RGB)
            resultCV = Image.fromarray(RGBresultArrayCV)
            imgtkCV = ImageTk.PhotoImage(image=resultCV)
            
            opencv_video.imgtk = imgtkCV
            opencv_video.configure(image=imgtkCV)
            #pwm_decide_once(detected_image, recent_boxes)
            
            opencv_fps = 1.0/(time()-opencv_start_time)
            opencv_fps_label.config(text=f"Fps: {opencv_fps}")
        opencv_video.after(1,opencvVideo)
    else:
        opencv_video_thread.join()


#-------------------


def send_pwm(x =0, y=0 , z = 500, yaw=0 , buttons=0):
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
    master.mav.manual_control_send(master.target_system, x,y,z,yaw,buttons)

def mode_set(mode_name):

    if mode_name not in master.mode_mapping():
        print('Unknown mode : {}'.format(mode_name))
        print('Try:', list(master.mode_mapping().keys()))
        sys.exit(1)
    flight_mode_label.config(text=f"Flight Mode: {mode_name}")
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    

def pwm_decide_once(detected_image,recent_boxes):
    try:
        #detection coordinates
        tlx,tly,w,h= recent_boxes[0]
        detectedMidx = tlx+w/2
        detectedMidy = tly+h/2

        #image corrdinates
        imgWidth, imgHeight, _ = detected_image.shape
        imgWidth_third = imgWidth/3
        imgWidth_two_third = 2*imgWidth_third
        
        if detectedMidx<imgWidth_third:             #left
            send_pwm(yaw=-600, z=75)
            current_activity_label.config(text="Object found on the left",font =("Courier", 10))
        elif detectedMidx<imgWidth_two_third:       #middle
            send_pwm(x=750, z=75)
            current_activity_label.config(text="Object found in the middle",font =("Courier", 10))
            
        else:                                       #right
            send_pwm(yaw=600, z=75)
            current_activity_label.config(text="Object found on the right",font =("Courier", 10))
    except:
        send_pwm(yaw=600, z=75)
        current_activity_label.config(text="Object NOt Found", font =("Courier", 10))
video_on = True

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

status_update = True
def statusUpdate():
    if status_update:
        
        try:
            rcvpacket = master.recv_match().to_dict()
        except:
            rcvpacket = None
        r,p,y,d = generatorfunc().pop(0), generatorfunc().pop(0), generatorfunc().pop(0), generatorfunc().pop(0)
        roll_label.config(text=f'Roll: {r}')
        pitch_label.config(text=f'Pitch: {p}')
        yaw_label.config(text=f'Yaw: {y}')
        depth_label.config(text=f'depth: {d}')
        # if rcvpacket['mavpackettype']=='ATTITUDE' or rcvpacket['macpackettype']=='AHRS2':
        #     roll_value= int(100* rcvpacket['roll'])
        #     pitch_value = int(100* rcvpacket['pitch'])
        #     yaw_value = int(100* rcvpacket['yaw'])

        #     roll_label.config(text=f'Roll: {roll_value}')
        #     pitch_label.config(text=f'Pitch: {pitch_value}')
        #     yaw_label.config(text=f'Yaw: {yaw_value}')
        #     #tuple halinde istenen verilerin alınması
        roll_label.after(1, statusUpdate)
    else:
        status_update_thread.join()

def generatorfunc():
    vals = []
    vals.append(randint(0,360))
    vals.append(randint(0,360))
    vals.append(randint(0,360))
    vals.append(randint(0,10))
    return vals



def set_target_depth(depth):
    
    master.mav.set_position_target_global_int_send(
        int(1e3 * (time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=0xdfe,  # ignore everything except z position
        lat_int=0, lon_int=0, alt=depth, # (x, y WGS84 frame pos - not used), z [m]
        vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )
def resetAll():
    global video_on
    global status_update
    video_on = False
    status_update = False

#------------------------------------------
net = cv2.dnn.readNet(os.path.abspath('Yolo_files/yolo-custom_3000.weights'),os.path.abspath('Yolo_files/yolo-custom.cfg'))
detection_classes = []
with open(os.path.abspath('Yolo_files/obj.names'), 'r') as f:
    detection_classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0]-1] for i in net.getUnconnectedOutLayers()]

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
            if confidence > 0.3:
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
            recognition_label.config(text=f"{label} bulundu")
            cv2.rectangle(raw_image, (topleft_x,topleft_y), (topleft_x+w,topleft_y+h), (0,100,255), 1)
            cv2.putText(raw_image, label, (topleft_x, topleft_y),cv2.FONT_HERSHEY_COMPLEX,1,(0,165,255))


    return raw_image, boxes

#--------------------------------------------
def toggleVideo():
    global video_on
    video_on = not video_on
# def toggle_attitude():
#     global attitude_update
#     attitude_update = not attitude_update
"""def toggle_movement():
    global moving
    moving = not moving"""
def toggleStart():
    master.arducopter_arm()
    master.motors_armed_wait()
    arm_status_label.config(text="Vehicle Status: Armed")

def toggleArm():
    if master.motors_armed():
        master.arducopter_disarm()
        master.motors_disarmed_wait()
        arm_status_label.config(text="Vehicle Status: Disarmed")
    else:
        master.arducopter_arm()
        master.motors_armed_wait()
        arm_status_label.config(text="Vehicle Status: Armed")


# master = mavutil.mavlink_connection("udpin:192.168.2.1:14550")
# master.wait_heartbeat()
# print("Successful Connection!")





###  GLOBAL VARIABLES AND OBJECTS
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

recent_boxes = []

video_on = True

root=Tk()
root.title("CALROV GUI")
##Icon
icon = Image.open(os.path.abspath('gui_images/calrov_logo.jpg'))
icon = ImageTk.PhotoImage(icon)
root.tk.call('wm','iconphoto',root._w, icon)

## TITLE TEXT
Title_label = Label(root, text = "CALROV TALAY")
Title_label.config(font =("Courier", 20))
Title_label.grid(row=0, column=0, columnspan=4)

##Live Video Display Box
video_app = Frame(root, bg="white")
video_app.grid(row=1,column=0,columnspan=2)


yolo_video = Label(video_app)
yolo_video.grid(row=1, column=0)
opencv_video = Label(video_app)
opencv_video.grid(row=1, column=1)



yolo_fps_label = Label(video_app, text="Yolo Fps: 0")
yolo_fps_label.grid(row=2, column=0)

opencv_fps_label = Label(video_app, text="Opencv Fps: 0")
opencv_fps_label.grid(row=2 , column=1)

###THREADS
yolo_video_thread = threading.Thread(target=yoloVideo)
opencv_video_thread  = threading.Thread(target=opencvVideo)
status_update_thread = threading.Thread(target=statusUpdate)
###BUTTONS


button_frame = Frame(root, bg="white")
button_frame.config(width=416,height=416)
toggle_stop_start = Button(button_frame, text='Toggle Stop/Start',
                            command=toggleStart)
toggle_arm_disarm = Button(button_frame, text='Toggle Arm/Disarm', 
                            command=toggleArm)

servo_open = Button(button_frame, text='Open Servo')
servo_close = Button(button_frame, text='Close Servo')

yolo_video_button = Button(button_frame, command=yolo_video_thread.start, text='Toggle YOLO Display')
opencv_video_button = Button(button_frame, text='Toggle Opencv Display', command = opencv_video_thread.start)

status_update_button = Button(button_frame, text='Status Update', command=status_update_thread.start)
resetButton = Button(button_frame, text='Stop All Activity', command=resetAll)

## BUtton Display
button_frame.grid(row=3, column=0)

toggle_stop_start.grid()
toggle_arm_disarm.grid()

servo_open.grid()
servo_close.grid()

yolo_video_button.grid()
opencv_video_button.grid()

status_update_button.grid()
resetButton.grid()

### VEHICLE STATUS DISPLAY
status_frame = Frame(root)
status_frame.config( bg='gray')
status_frame.grid(row=3, column=1)

arm_status_label = Label(status_frame, text="Vehile Status:")
flight_mode_label = Label(status_frame, text="Flight Mode: ")


roll_label = Label(status_frame, text="Roll: N/A")
pitch_label = Label(status_frame, text="Pitch: N/A")
yaw_label = Label(status_frame, text="Yaw: N/A")
depth_label = Label(status_frame, text="Depth: N/A")

servo_label = Label(status_frame, text="Servo: N/A")
recognition_label = Label(status_frame, text="Object Recognition: N/A")

current_activity_label = Label(status_frame, text="Current Activity: ")

## STATUS DISPLAY
arm_status_label.grid()
flight_mode_label.grid()

roll_label.grid()
pitch_label.grid()
yaw_label.grid()
depth_label.grid()


servo_label.grid()
recognition_label.grid()

current_activity_label.grid()
###BUTTONS

if __name__ == "__main__":
    toggleStart()
    
    DEPTH_HOLD = 'MANUAL'
    DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]

    while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
        master.set_mode(DEPTH_HOLD)
    #set_target_depth(-1)
    servo_label.config(text="Servo Not Attached")
    root.mainloop()