##LIBRARIES
import threading
from tkinter import *
from PIL import ImageTk, Image
import cv2
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst
import numpy as np
from time import sleep
from pymavlink import mavutil
from math import pi as PI
import threading

##INITIALIZING
master = mavutil.mavlink_connection("udpin:192.168.2.1:14550")
print("Bağlantı kuruldu!")
root = Tk()
root.title('CALROV ARAYÜZ')
#root.geometry("640x480")
#root.iconphoto(False, PhotoImage(file="/home/violetcheese/Desktop/CALROV/GUI/calrov_icon2.ico"))
#root.iconbitmap("/home/violetcheese/Desktop/CALROV/GUI/calrov_icon2.ico")
app = Frame(root, bg="white")
app.grid()
lmain = Label(app)
lmain.grid()

##DEFINITIONS
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

request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 5)
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 30)

def sensor_data():
    waiting =True
    while waiting: #Mesaj Alınanan kadar bekle
        try:
            global latest_attitude #son alınan verinin kullanılması
            rcvpacket = master.recv_match().to_dict()
            if rcvpacket['mavpackettype']=='ATTITUDE':
                latest_attitude = rcvpacket  #Attitude verisinin güncellenmesi
                waiting =False
                roll_perc= 100* rcvpacket['roll']/PI
                pitch_perc = 100* rcvpacket['pitch']/PI
                yaw_perc = 100* rcvpacket['yaw']/PI

                return roll_perc, pitch_perc, yaw_perc #tuple halinde istenen verilerin alınması
        except:
            pass # Fonksiyon veri alana kadar tekrardan deneyecektir. 




def main():
    #Frame Test
    conlost = cv2.imread("/home/violetcheese/Desktop/CALROV/GUI/gui_images/todo.webp")
            #Bağlantı kopması halinde bağlantının kesildiği ile ilgili bir görsel gösterilecektir. 
    frame = video.frame()
    if not video.frame_available():
        frame = conlost  # görüntü eksikliğinde bağlantı kopukluğu ekrana yansıtılır. 
    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) #BGR RGB dönüşümü

    img = Image.fromarray(cv2image)
    
    imgtk = ImageTk.PhotoImage(image=img)
    lmain.imgtk = imgtk
    lmain.configure(image=imgtk)
    lmain.after(1, main) # bir sonraki video frame parçasının gösterilmesi için main tekrar başlar
if __name__ == '__main__':
    # Video objesi oluşturuldu
    # Port Seçildi
    video = Video(port=4777)
    main()
    root.mainloop()
    
