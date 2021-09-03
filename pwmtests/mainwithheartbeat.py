import threading
from tkinter import *
from PIL import ImageTk, Image
import cv2
import gi
from pymavlink.quaternion import QuaternionBase
gi.require_version("Gst", "1.0")
from gi.repository import Gst
import numpy as np
import time
from pymavlink import mavutil
import math
import threading
import os.path
import sys

master = mavutil.mavlink_connection("udpin:192.168.2.1:14550")
boot_time = time.time()
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

    def frame_available(self)->bool:
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

# cap = cv2.VideoCapture(0)
# if not cap.isOpened():
#     print("Cannot open camera")
#     exit()

recent_boxes = []
def videoMain():
    video_on.wait()

    while video_on.is_set():
        
        if video.frame_available():
            video_frame = video.frame()
            frameCV = cv2.resize(frameCV, (416,416))

            yolo_thread_in_main = threading.Thread(target=yoloVideo, args=(video_frame,))
            opencv_thread_in_main = threading.Thread(target=opencvVideo, args=(video_frame,))

            yolo_thread_in_main.start()
            opencv_thread_in_main.start()

            yolo_thread_in_main.join()
            opencv_thread_in_main.join()
            del yolo_thread_in_main
            del opencv_thread_in_main
    videoMain()
        

def yoloVideo(frameYolo:np.ndarray):
    frame_start_time: float = time.time()
    
    global recent_boxes
    
    """cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) #BGR RGB dönüşümü
    height, width = (int(cv2image.shape[1]*img_scale),int(cv2image.shape[0]*img_scale))

    scaled_img = cv2.resize(cv2image,(height, width))
    """
    try:
        tmp_boxes = yoloDetection(frameYolo)
        
        ##Draw 
        if tmp_boxes and type(tmp_boxes[0])==list and  len(tmp_boxes[0])==4:
            recent_boxes = tmp_boxes
            w = recent_boxes[0][2]
            h=  recent_boxes[0][3]
            
            cv2.rectangle(frameYolo,
            (int(recent_boxes[0][0]-w/2),int(recent_boxes[0][1]-h/2)),
            (int(recent_boxes[0][0]+w/2),int(recent_boxes[0][1]+h/2)),
            (0,0,0), thickness=1)
            recognition_label.config(text=f"Object is {208-recent_boxes[0][0]} pixels from the center")
    except:
        pass

    detected_image = cv2.cvtColor(frameYolo , cv2.COLOR_BGR2RGB)
    imgYolo = Image.fromarray(detected_image)

    #img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    imgtkYolo = ImageTk.PhotoImage(image=imgYolo)
    yolo_video.imgtk = imgtkYolo
    yolo_video.configure(image=imgtkYolo)
    
    
    ##FPS
    yolo_fps = 1.0/(time.time()-frame_start_time)
    yolo_fps_label.config(text=f"Fps: {yolo_fps}")

lower_hsv = np.array([40, 76, 85])
upper_hsv = np.array([76, 164, 172])

def opencvVideo(frameCV):

    opencv_start_time = time.time()
    
    
    hsvCV = cv2.cvtColor(frameCV, cv2.COLOR_BGR2HSV)
    
    
    maskCV = cv2.inRange(hsvCV, lower_hsv, upper_hsv)

    resultArrayCV = cv2.bitwise_and(frameCV, frameCV, mask=maskCV)

    RGBresultArrayCV = cv2.cvtColor(resultArrayCV, cv2.COLOR_BGR2RGB)
    resultCV = Image.fromarray(RGBresultArrayCV)
    imgtkCV = ImageTk.PhotoImage(image=resultCV)
    
    opencv_video.imgtk = imgtkCV
    opencv_video.configure(image=imgtkCV)
    #pwm_decide_once(detected_image, recent_boxes)
    
    opencv_fps = 1.0/(time.time()-opencv_start_time)
    opencv_fps_label.config(text=f"Fps: {opencv_fps}")

def toggleVideo():
    if video_on.is_set():
        video_on.clear()
    else:
        video_on.set()

##  SENSOR INPUT-----------------
def statusUpdate():
    status_update.wait()

    while status_update.is_set():

        try:
            rcvpacket = master.recv_match(type=['ATTITUDE', 'AHRS2']).to_dict()
            
            roll_value= int(100* rcvpacket['roll']/math.pi)
            pitch_value = int(100* rcvpacket['pitch']/math.pi)
            yaw_value = int(100* rcvpacket['yaw']/math.pi)

            roll_label.config(text=f'Roll: {roll_value}')
            pitch_label.config(text=f'Pitch: {pitch_value}')
            yaw_label.config(text=f'Yaw: {yaw_value}')

            #depth_packet = master.recv_match(type=)
            
        except:

            rcvpacket = None
        
       
        
    statusUpdate()    




#VEHICLE CONFIG------------------


def flightModeSet(mode_name):

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

def cameraGimbalSet(tilt, roll=0, pan=0):
    """
    Gimbal istenen pozisyona taşınır.
    Args:
        tilt (float): santiderece cinsinden tilt (0 nötr)
        roll (float, isteğe bağlı): santiderece cinsinden roll (0 nötr)
        pan  (float, isteğe bağlı): santiderece cinsinden pan (0 nötr)
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        1,
        tilt,
        roll,
        pan,
        0, 0, 0,
        mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)


def requestMessageInterval(message_id: int, frequency_hz: float):
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


def setTargetDepth(depth):
    
    master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=0xdfb,  # ignore everything except z position 
        lat_int=0, lon_int=0, alt=depth, # (x, y WGS84 frame pos - not used), z [m]
        vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )


def setTargetAttitude(roll=0, pitch=0, yaw=0):
    """ Sets the target attitude while in depth-hold mode.

    'roll', 'pitch', and 'yaw' are angles in degrees.

    """
    # https://mavlink.io/en/messages/common.html#ATTITUDE_TARGET_TYPEMASK
    # 1<<6 = THROTTLE_IGNORE -> allow throttle to be controlled by depth_hold mode
    bitmask = 1<<6

    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        bitmask,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
    )



#VEHICLE CONTROL------------------

class OtonomVehicle():
    def __init__(self) -> None:
        self.frameDim = 416
        self.frameMid = 208
        self.kp = 1.0
        self.ki = 0.002
        self.kd = -0.5
        self.preTotal = 0
        self.lastPositionMid = 0
        
        self.locationRight = True
        self.turn = 1
        
        self.phase_finding_start = threading.Event()
        self.phase_locationtest_start = threading.Event()
        self.phase_alignment_start = threading.Event()
        self.phase_end_start = threading.Event()

        self.phase_finding_start.clear()
        self.phase_locationtest_start.clear()
        self.phase_alignment_start.clear()
        self.phase_end_start.clear()

        self.phase_finding_thread = threading.Thread(target=self.phaseOne)
        self.phase_finding_thread.start()

        self.phase_locationtest_thread = threading.Thread(target=self.phaseTwo)
        self.phase_locationtest_thread.start()

        self.phase_alignment_thread = threading.Thread(target=self.phaseThree)
        self.phase_alignment_thread.start()

        self.phase_end_thread = threading.Thread(target=self.phaseFour)
        self.phase_end_thread.start()
    
    @property
    def proportionalYawValue(self):
        return self.kp * (208-recent_boxes[0][0])
    
    @property
    def integralYawValue(self):
        self.preTotal += self.ki * (208-recent_boxes[0][0])
        if self.preTotal>50:
            self.preTotal=50

        return self.preTotal
    
    @property
    def derivativeYawValue(self):
        currentMid = recent_boxes[0][0] 
        Diff = currentMid - self.lastPositionMid
        self.lastPositionMid = currentMid
        return Diff

    def phaseOne(self) -> None:
        """Spin until you find the frame"""
        self.phase_finding_start.wait()  ## IF Phase one is active
        current_activity_label.config(text="Phase 1: Find the Frame")
        while self.phase_finding_start.is_set():
            if len(recent_boxes[0]) != 0: ##recent_boxes has a value
                sendPwm(yaw=self.proportionalYawValue+self.integralYawValue+self.derivativeYawValue)
                if 178 < recent_boxes[0] < 238: ##Close to center 60 pixel leniency
                    self.phase_finding_start.clear()
                    self.phase_locationtest_start.set()
            else:
                sendPwm(yaw=400)

        self.phaseOne()
    
    def phaseTwo(self) -> None:
        """Determines the turn direction"""
        self.phase_locationtest_start.wait() ## IF Phase two is active
        current_activity_label.config(text="Phase 2: Find your turn Direction")
        while self.phase_locationtest_start.is_set():
            startTime = time.time()
            startRatio = recent_boxes[0][2]/recent_boxes[0][3]

            while time.time()<startTime+6:
                sendPwm(y=500*self.turn, yaw=self.proportionalYawValue) ##c-clockwise
            endRatio = recent_boxes[0][2]/recent_boxes[0][3]
            if endRatio>startRatio:
                self.turn = 1 ## c-clockwise turn around the frame
            else:
                self.turn = -1 ## clockwise turn
            self.phase_locationtest_start.clear()
            self.phase_alignment_start.set()

        self.phaseTwo()
            

    def phaseThree(self)-> None:
        self.phase_alignment_start.wait()  ## IF Phase three is active
        current_activity_label.config(text="Phase 3: Alignment")
        while self.phase_alignment_start.is_set():
            if recent_boxes[0][2]/recent_boxes[0][3]>1.3 and recent_boxes[0][0]<238 and recent_boxes[0][0]>178: ##at most 60 pixel fail
                self.phase_alignment_start.clear()
                self.phase_end_start.set()
            else:
                sendPwm(y= 400*self.turn, yaw=self.proportionalYawValue)
          

        self.phaseThree()

    def phaseFour(self) -> None:
        self.phase_end_start.wait()   ## IF Phase four is active
        current_activity_label.config(text="Phase 4: Straight ahaid, End")
        while self.phase_end_start.is_set():
            #sendPwm(x=400) #If yaw makes it worse somehow
            sendPwm(x=400, yaw=self.proportionalYawValue/3) ## Yaw tries to correct mistakes
           
            
            

        self.phaseFour()

Vehicle = OtonomVehicle()


def sendPwm(x =0, y=0 , z = 500, yaw=0 , buttons=0):
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

#------------------------------------------
net = cv2.dnn.readNet(os.path.abspath('Yolo_files/yolo-custom_3000.weights'),os.path.abspath('Yolo_files/yolo-custom.cfg'))
detection_classes = []
with open(os.path.abspath('Yolo_files/obj.names'), 'r') as f:
    detection_classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0]-1] for i in net.getUnconnectedOutLayers()]

def yoloDetection(raw_image) ->list:
    """Take in as input a cv2 image"""
    class_ids = []
    confidences = []
    boxes = []
    height = raw_image.shape[0]
    width  = raw_image.shape[1]
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
                
                boxes.append([center_x, center_y,w,h])
                confidences.append(float(confidence))
                class_ids.append(class_id)
    
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    if len(indexes)>0:
        new_boxes = [boxes[index] for index in indexes[0]]
        return new_boxes
    



#THREADED--------------------------------------------


def toggleArm():
    if master.motors_armed():
        master.arducopter_disarm()
        master.motors_disarmed_wait()
        arm_status_label.config(text="Vehicle Status: Disarmed")
    else:
        master.arducopter_arm()
        master.motors_armed_wait()
        arm_status_label.config(text="Vehicle Status: Armed")

def startAllThreads():
    status_update_thread.start()
    video_main_thread.start()

def toggleOnOff():
    if not video_on.is_set():
        video_on.set()
        status_update.set()
    else:
        video_on.clear()
        status_update.clear()
        




##ROOT
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
def eventReverser(Event:threading.Event):
    if Event.is_set():
        Event.clear()
    else:
        Event.set()

# yolo_video_thread = threading.Thread(target=yoloVideo)
# opencv_video_thread  = threading.Thread(target=opencvVideo)

status_update_thread = threading.Thread(target=statusUpdate)
video_main_thread = threading.Thread(target=videoMain)

status_update = threading.Event()
video_on = threading.Event()

###BUTTONS


button_frame = Frame(root, bg="white")
button_frame.config(width=416,height=416)
button_frame.grid(row=3, column=0)

startThreadsButton = Button(button_frame, text="Start all threads", 
                        command=startAllThreads)
startThreadsButton.grid()

startVideoButton = Button(button_frame, text='Toggle Video',
                        command=toggleVideo )
startVideoButton.grid()

toggle_arm_disarm = Button(button_frame, text='Toggle Arm/Disarm',
                        command=toggleArm)
toggle_arm_disarm.grid()


#servo_open = Button(button_frame, text='Stop All Activity')
# servo_close = Button(button_frame, text='Close Servo')

# yolo_video_button = Button(button_frame, command=yolo_video_thread.start, text='Toggle YOLO Display')
# opencv_video_button = Button(button_frame, text='Toggle Opencv Display', command = opencv_video_thread.start)

#status_update_button = Button(button_frame, text='Status Update', command=status_update_thread.start)

toggleButton = Button(button_frame, text='Toggle All Activity', command=toggleOnOff)
toggleButton.grid()
#BUtton Display





# servo_open.grid()
# servo_close.grid()

# yolo_video_button.grid()
# opencv_video_button.grid()

#status_update_button.grid()


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

manipulator_servo_label = Label(status_frame, text="Servo: N/A")
recognition_label = Label(status_frame, text="Object Recognition: N/A")

current_activity_label = Label(status_frame, text="Current Activity: ")

## STATUS DISPLAY
arm_status_label.grid()
flight_mode_label.grid()

roll_label.grid()
pitch_label.grid()
yaw_label.grid()
depth_label.grid()


manipulator_servo_label.grid()
recognition_label.grid()

current_activity_label.grid()




mavlink = mavutil.mavlink


class WriteLockedFile(object):
    """ A file with thread-locked writing. """
    def __init__(self, file):
        self._base_file = file
        self._write_lock = threading.Lock()
        
    def write(self, *args, **kwargs):
        with self._write_lock:
            self._base_file.write(*args, **kwargs)
    
    def __getattr__(self, name):
        return getattr(self._base_file, name)
    
    def __dir__(self):
        return dir(self._base_file) + ["_base_file", "_write_lock"]


class mavactive(object):
    """ A class for managing an active mavlink connection. """
    def __init__(self, connection, type_=mavlink.MAV_TYPE_GENERIC, autopilot=mavlink.MAV_AUTOPILOT_INVALID, base_mode=0, custom_mode=0,
                 mavlink_version=0, heartbeat_period=0.95):
        """ Initialises the program state and starts the heartbeat thread. """
        self.connection = connection
        self.type = type_
        self.autopilot = autopilot
        self.base_mode = base_mode
        self.custom_mode = custom_mode
        self.mavlink_version = mavlink_version
        self.heartbeat_period = heartbeat_period
        
        # replace internal file with a thread-safe one
        self.connection.mav.file = WriteLockedFile(self.connection.mav.file)
        # set up the kill event and initialise the heartbeat thread
        self._kill = threading.Event()
        self._birth()

    def _birth(self):
        """ Creates and starts the heartbeat thread. """
        self._kill.clear()
        self.heartbeat_thread = threading.Thread(target=self.heartbeat_repeat)
        self.heartbeat_thread.start()

    @property
    def is_alive(self):
        return not self._kill.is_set()

    def heartbeat_repeat(self):
        """ Sends a heartbeat to 'self.connection' with 'self.heartbeat_period'. """
        while self.is_alive:
            self.connection.mav.heartbeat_send(
                self.type,
                self.autopilot,
                self.base_mode,
                self.custom_mode,
                self.mavlink_version
            )
            time.sleep(self.heartbeat_period)

    def kill(self):
        """ Stops the heartbeat, if not already dead. """
        if not self.is_alive:
            return # already dead

        self._kill.set()
        self.heartbeat_thread.join()
        del self.heartbeat_thread

    def revive(self):
        """ Starts the heartbeat, if not already alive. """
        if self.is_alive:
            return # already alive

        self._birth()

    def __del__(self):
        """ End the thread cleanly on program end. """
        self.kill()

heartbeatPulser = mavactive(master)
###MAIN SETUP
def main():
    heartbeatPulser.heartbeat_repeat()
    ###Vehicle Armed
    arm_status_label.config(text="Vehicle Status Disarmed")
    master.arducopter_arm()
    master.motors_armed_wait()
    arm_status_label.config(text="Vehicle Status: Armed")
    ###Flight Mode Depth Hold
    FLIGHT_MODE = 'ALT_HOLD'
    FLIGHT_MODE_ID = master.mode_mapping()[FLIGHT_MODE]
    while not master.wait_heartbeat().custom_mode == FLIGHT_MODE_ID:
        master.set_mode(FLIGHT_MODE)
    flight_mode_label.config(text="Flight Mode: ALT_HOLD")

    ###Camera Servo
    cameraGimbalSet(500) ##Camera Jerk
    cameraGimbalSet(0)
    ##Manipulator Servo
    manipulator_servo_label.config(text="Servo Not Attached")
    
    ##Message Interval
    requestMessageInterval()

    ##Target Depth and Attitude
    setTargetDepth(-1)
    #setTargetAttitude()

    ##Start Phase 1
    Vehicle.phase_finding_start.set()
    ##Start GUI
    root.mainloop()

if __name__ == "__main__":

    #root.mainloop()
    main()