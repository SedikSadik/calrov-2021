# import yolo_detection 
import cv2
import numpy as np
import os
import time
import cProfile
import cythonized

net = cv2.dnn.readNet(os.path.abspath('./GUI/Yolo_files/yolov4-tiny.weights'),os.path.abspath('./GUI/Yolo_files/yolov4-tiny.cfg'))
detection_classes = []
with open(os.path.abspath('./GUI/Yolo_files/coco.names'), 'r') as f:
    detection_classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0]-1] for i in net.getUnconnectedOutLayers()]

cap = cv2.VideoCapture(0)
_, frame = cap.read()
#def yolo_detection(raw_image, net, output_layers, detection_classes)


# print(cythonized.yolo_detection(frame,net,  output_layers, detection_classes))
Start = time.time()
i = 0
while i <50:

    boxes = cythonized.yolo_detection(frame,net,  output_layers, detection_classes)
    # boxes = cythonized.yolo_detection(frame,net,  output_layers, detection_classes)
    # boxes = cythonized.yolo_detection(frame,net,  output_layers, detection_classes)
    # boxes = cythonized.yolo_detection(frame,net,  output_layers, detection_classes)
    # boxes = cythonized.yolo_detection(frame,net,  output_layers, detection_classes)
    i+=1
Stop = time.time()
print(Stop-Start)
