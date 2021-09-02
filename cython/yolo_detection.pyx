import cv2
import os
import numpy as np



def yolo_detection(raw_image, net, output_layers, detection_classes):
    """Take in as input a cv2 image"""
    class_ids = []
    confidences = []
    boxes = []
    height , width = raw_image.shape[:2]
    blob = cv2.dnn.blobFromImage(raw_image, 0.00392, (416,416), (0,0,0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.3:
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
    #print(indexes)
    #DISPLAY DETECTION
    new_boxes = [boxes[index] for index in indexes[0] if indexes[0] is not None]
    
            

    return new_boxes
