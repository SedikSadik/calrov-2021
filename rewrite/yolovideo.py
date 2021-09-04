import numpy as np
import cv2
import time
from tkinter import Label
import os

def yoloDetection(net :cv2.dnn.readFromDarknet,  output_layers,raw_image:np.ndarray, currentlyDetected:bool) ->list:
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
            scores = detection[5:] ##1 li liste
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
        #recent_boxes_lock.acquire()
        Otonomvehicle.recent_boxes = [boxes[index] for index in indexes[0]]
        #recent_boxes_lock.release()
        currentlyDetected = True
    else:
        currentlyDetected = False
    

def yolo_video(, frame_yolo:np.ndarray, recent_boxes: list, yolo_fps_label=None):
    frame_start_time: float = time.time()
    
    if yolo_fps_label is None:
        yolo_fps = 1.0/(time.time()-frame_start_time)
        yolo_fps_label.config(text=f"Fps: {yolo_fps}")


# net = cv2.dnn.readNet(os.path.abspath('Yolo_files/yolo-custom_3000.weights'),os.path.abspath('Yolo_files/yolo-custom.cfg'))
# detection_lasses = []
# with open(os.path.abspath('Yolo_files/obj.names'), 'r') as f:
#     detection_classes = [line.strip() for line in f.readlines()]
# layer_names = net.getLayerNames()
# output_layers = [layer_names[i[0]-1] for i in net.getUnconnectedOutLayers()]
# currentlyDetected =False