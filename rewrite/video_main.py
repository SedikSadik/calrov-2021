from rewrite.vehicle_classes import OtonomVehicle
import threading
import tkinter
import cv2
from .videoclass import Video

def video_main(video_source:cv2.VideoCapture or Video,
    video_event: threading.Event,
    yolo_video_function,
    vehicle:OtonomVehicle,
    yolo_net:cv2.dnn.readNet,
    output_layers:list,
    yolo_video_app : tkinter.Frame,
    yolo_fps_label:tkinter.Label,
    opencv_video_function=None,
    resolution: tuple = (416,416)):
    
    if isinstance(video_source, (Video, cv2.VideoCapture)):
        raise TypeError("Only VideoCapture or Video instances are accepted.")
    print("Waiting for video to be on...")
    video_event.wait()
    while video_event.is_set():
        success, video_frame = video_source.read() 
        
        if success:
            video_frame = cv2.resize(video_frame, resolution)
            

            yolo_thread_in_main = threading.Thread(target=yolo_video_function,
                 args=(vehicle, yolo_net,output_layers, yolo_video_app, video_frame, yolo_fps_label))
            yolo_thread_in_main.start()

            if opencv_video_function is not None and isinstance(opencv_video_function, function):


                opencv_thread_in_main = threading.Thread(target=opencv_video_function, args=(video_frame,))
                opencv_thread_in_main.start()
                opencv_thread_in_main.join()
                del opencv_thread_in_main

            yolo_thread_in_main.join()
            del yolo_thread_in_main
            

    video_main(video_source, video_event, yolo_video_function, opencv_video_function)
