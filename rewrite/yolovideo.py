import tkinter
import numpy as np
import cv2
import time
from tkinter import Label
import os
from .vehicle_classes import OtonomVehicle
from PIL import ImageTk , Image

##DEFINITIONS

def yoloDetection(vehicle:OtonomVehicle, net :cv2.dnn.readNet,  output_layers,raw_image:np.ndarray) ->None:
    """Take in as input a cv2 image, Changes the vehicle recentboxes and current detection attributes."""
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
        #recent_boxes_lock.acquire()
        vehicle.recent_boxes = [boxes[index] for index in indexes[0]]
        #recent_boxes_lock.release()
        vehicle.currentlyDetected.set()
    else:
        vehicle.currentlyDetected.clear()
    

def yolo_video(vehicle:OtonomVehicle, yolo_net:cv2.dnn.readNet ,output_layers:list, yolo_video_app: tkinter.Frame, yolo_frame:np.ndarray, yolo_fps_label : tkinter.Label=None):
    frame_start_time: float = time.time()
    
    yoloDetection(vehicle, yolo_net,output_layers,yolo_frame )
    tmp_yolo_boxes = vehicle.recent_boxes[0]
    if vehicle.currentlyDetected.is_set() and type(tmp_yolo_boxes)==list:
        cx,cy,w,h = tmp_yolo_boxes[0]
        cy = tmp_yolo_boxes[1]
        w = tmp_yolo_boxes[2]
        h = tmp_yolo_boxes[3]
        yolo_frame = cv2.
    color_converted_yolo = cv2.cvtColor(yolo_frame, cv2.COLOR_BGR2RGB)
    imgYolo = Image.fromarray(color_converted_yolo)

    #img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    imgtkYolo = ImageTk.PhotoImage(image=imgYolo)
    yolo_video_app.imgtk = imgtkYolo
    yolo_video_app.configure(image = imgtkYolo)


    if yolo_fps_label is not None and isinstance(yolo_fps_label, tkinter.Label):
        yolo_fps = 1.0/(time.time()-frame_start_time)
        yolo_fps_label.config(text=f"Fps: {yolo_fps}")
        


# net = cv2.dnn.readNet(os.path.abspath('Yolo_files/yolo-custom_3000.weights'),os.path.abspath('Yolo_files/yolo-custom.cfg'))
# detection_lasses = []
# with open(os.path.abspath('Yolo_files/obj.names'), 'r') as f:
#     detection_classes = [line.strip() for line in f.readlines()]
# layer_names = net.getLayerNames()
# output_layers = [layer_names[i[0]-1] for i in net.getUnconnectedOutLayers()]
# currentlyDetected =False