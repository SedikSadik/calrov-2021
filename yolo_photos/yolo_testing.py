import os
import cv2
import numpy as np
from time import time as TM
### YOLO and DETECTION
## Loading Yolo
net = cv2.dnn.readNet('/home/violetcheese/Documents/CALROV/GUI/Yolo_files/yolov3.weights','/home/violetcheese/Documents/CALROV/GUI/Yolo_files/yolov3.cfg')
detection_classes = []
with open('/home/violetcheese/Documents/CALROV/GUI/Yolo_files/coco.names', 'r') as f:
    detection_classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0]-1] for i in net.getUnconnectedOutLayers()]

#print(detection_classes)


attitude_update=False
video_update = False

def yolo_detection(raw_image):
    """Take in as input a cv2 image"""
    class_ids = []
    confidences = []
    boxes = []
    height , width ,c= raw_image.shape
    blob = cv2.dnn.blobFromImage(raw_image, 0.00392, (416,416), (0,0,0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.4:
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
            cv2.rectangle(raw_image, (topleft_x,topleft_y), (topleft_x+w,topleft_y+h), (0,100,255), 1)
            cv2.putText(raw_image, label, (topleft_x, topleft_y),cv2.FONT_HERSHEY_COMPLEX,1,(0,165,255))


    return raw_image



for subdir, dirs, files in os.walk('/home/violetcheese/Documents/CALROV/yolo_photos'):
    for file in files:
        #print os.path.join(subdir, file)
        filepath = subdir + os.sep + file
        print(file)
        os.chdir("/home/violetcheese/Documents/CALROV/yolo_photos/yolo_detections")
        
        if filepath.endswith(".jpg"):
            
            cv2.imwrite(f"detected{TM()}.jpg", yolo_detection(cv2.imread(filepath)) )
            print(f"detected{TM()}.jpg"+"  \n Done")