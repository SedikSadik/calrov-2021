import threading
import cv2
import gi
import time
import math
import os.path
import sys
import tkinter as tk
from PIL import ImageTk, Image
from pymavlink.quaternion import QuaternionBase
from pymavlink import mavutil
import numpy as np
gi.require_version("Gst", "1.0")
from gi.repository import Gst
import cProfile


from rewrite.videoclass import Video
# from .videoclass import Video
# from .videoclass import Video
from rewrite.vehicle_classes import OtonomVehicle, Driver
from rewrite.video_main import video_main
from rewrite.yolovideo import yoloDetection, yolo_video
from rewrite.gui import CALROV_GUI
from rewrite.heartbeat import mavactive, mavlink
from rewrite.functions import set_all_events,set_all_threads



net = cv2.dnn.readNet(os.path.abspath('Yolo_files/yolov4-tiny.weights'),
                      os.path.abspath('Yolo_files/yolov4-tiny.cfg'))
detection_classes = []
with open(os.path.abspath('Yolo_files/coco.names'), 'r') as f:
    detection_classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0]-1] for i in net.getUnconnectedOutLayers()]

camera = cv2.VideoCapture(0)
tmp_video_on = threading.Event()
tmp_video_on.set()
def tmpfunc():
    print("Running tmp func")

master= None#mavutil.mavlink_connection("udpin:192.168.2.1:14550")
Talay = OtonomVehicle(master)
video = Video(port=4777)
driver = Driver(Talay)
gui = CALROV_GUI(set_all_threads,set_all_events,Talay, title="CALROV TALAY")

Talay.__setattr__("video_thread",threading.Thread(target=video_main, args=(camera, yolo_video,Talay, net,output_layers, gui.yolo_video,gui.yolo_fps_label)))

gui.start_threads_button.configure(command=lambda: set_all_threads(Talay.video_thread, Talay.status_update_thread))
Talay.video_on_event.set()
driver.phase_depth_start.set()
def main():
    # ##HEARTBEAT
    # heartbeatPulser = mavactive(Talay.connection)
    # ###Vehicle Armed
    # Talay.disarm(gui.arm_status_label)
    # Talay.arm(gui.arm_status_label)
    # ###Flight Mode Depth Hold
    # FLIGHT_MODE = 'ALT_HOLD'
    # FLIGHT_MODE_ID = Talay.connection.mode_mapping()[FLIGHT_MODE]
    # while not Talay.connection.wait_heartbeat().custom_mode == FLIGHT_MODE_ID:
    #     Talay.connection.set_mode(FLIGHT_MODE)
    # gui.flight_mode_label.config(text="Flight Mode: ALT_HOLD")
    
    # ###Camera Servo
    # Talay.cameraGimbalSet(500) ##Camera Jerk
    # Talay.cameraGimbalSet(0)
    # ##Manipulator Servo
    # gui.manipulator_servo_label.config(text="Servo Attached")
    
    # ##Message Interval
    # #requestMessageInterval()

    # ##Target Depth and Attitude
    # Talay.setTargetDepth(-1)
    # #setTargetAttitude()

    # ##Start Phase 1
    # #Vehicle.phase_finding_start.set()
    # ##Start GUI
    gui.root.mainloop()

main()