import threading
import cv2
from .videoclass import Video

def video_main(video_source, video_event: threading.Event, yolo_video_function, opencv_video_function, resolution: tuple = (416,416)):
    if isinstance(video_source, (Video, cv2.VideoCapture)):
        raise TypeError("Only VideoCapture or Video instances are accepted.")
    print("Waiting for video...")
    video_event.wait()
    while video_event.is_set():
        success, video_frame = video_source.read() 
        if success:
            video_frame = cv2.resize(video_frame, resolution)
            

            yolo_thread_in_main = threading.Thread(target=yolo_video_function, args=(video_frame,))
            opencv_thread_in_main = threading.Thread(target=opencv_video_function, args=(video_frame,))

            yolo_thread_in_main.start()
            opencv_thread_in_main.start()

            yolo_thread_in_main.join()
            opencv_thread_in_main.join()
            del yolo_thread_in_main
            del opencv_thread_in_main

    video_main(video_source, video_event, yolo_video_function, opencv_video_function)
