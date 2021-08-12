import numpy as np
import cv2
cap = cv2.VideoCapture(0)
import os


perspective_names = ["true_center/", 'center_down/', 'center_up/', "left_center/", "left_up/", "left_down/", "right_up/", "right_center/", "right_down/","Joker/"]
attempt_id="attempt1/"
parent_path = "/home/violetcheese/Documents/CALROV/yolo_photos/"
os.mkdir(parent_path+attempt_id)
for perspective_name in perspective_names:
    os.mkdir(parent_path+attempt_id+"perspective-"+perspective_name)
    for batch in range(1):
        dirpath = parent_path+attempt_id+"perspective-"+perspective_name+"batch"+str(batch)
        
        os.mkdir(dirpath)
        for frame_count in range(50):
            ret, frame = cap.read()
            os.chdir(dirpath)
            frame_name = f"frame{frame_count}.jpg"
            

            cv2.resize(frame, (416,416))
            cv2.imwrite(frame_name, frame)
            print(frame_name)
            (perspective_names.index(perspective_name)+1)
    input(f"Next perspective: {perspective_names[perspective_names.index(perspective_name)+1]}")
