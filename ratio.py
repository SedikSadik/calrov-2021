import timeit
import cv2
import os
import sys
import numpy as np


net = cv2.dnn.readNet(os.path.abspath('./GUI/Yolo_files/yolo-custom_3000.weights'),os.path.abspath('./GUI/Yolo_files/yolo-custom.cfg'))
detection_classes = []
with open(os.path.abspath('./GUI/Yolo_files/obj.names'), 'r') as f:
    detection_classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0]-1] for i in net.getUnconnectedOutLayers()]

def yolo_detection(raw_image):
    """Take in as input a cv2 image, outputs the detected image and the bounding boxes"""

    confidences = []
    boxes = []
    height , width = raw_image.shape[:2]
    blob = cv2.dnn.blobFromImage(raw_image, 0.00392, (416,416), (0,0,0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)
    
    for out in outs:
        for detection in out:
            
            scores = detection[5:]
            #print(scores)
            #class_id = np.argmax(scores)
            #print(class_id)
            confidence = scores[0]
            
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
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    #DISPLAY DETECTION
    #total_detections = len(boxes)
    #print(indexes[0])
    new_boxes = [boxes[index] for index in indexes[0]]
    
    # for i in range(total_detections):
    #     if i in indexes:
    #         topleft_x, topleft_y, w,h = boxes[i]
    #         label = detection_classes[class_ids[i]]
    #         #recognition_label.config(text=f"{label} bulundu")
    #         cv2.rectangle(raw_image, (topleft_x,topleft_y), (topleft_x+w,topleft_y+h), (0,100,255), 1)
    #         cv2.putText(raw_image, label, (topleft_x, topleft_y),cv2.FONT_HERSHEY_COMPLEX,1,(0,165,255))
    label = "otonomkapi"
    for topleft_x, topleft_y, w,h in new_boxes:
        
        cv2.rectangle(raw_image, (topleft_x,topleft_y), (topleft_x+w,topleft_y+h), (0,100,255), 1)
        cv2.putText(raw_image, label, (topleft_x, topleft_y),cv2.FONT_HERSHEY_COMPLEX,1,(0,165,255))
    return raw_image, new_boxes

def ratioToAngle(ratio:float) -> float:
    return 133.2469 - (139.1493*ratio) + (162.7091*(ratio**2)) - (86.84221*(ratio**3))
def ratioToAngle2(ratio:float) -> float:
    return -795.4162 + (91.44677+795.4162)/(1 + (ratio/2.579814)**(3.952141))

image = cv2.imread("/home/violetcheese/Documents/CALROV/photos/1 aşşağı 50 derece 3.jpg")
detectedImage , detectionBoxes = yolo_detection(image)
w = detectionBoxes[0][2]
h= detectionBoxes[0][3]
ratio = w/h
angle = ratioToAngle(ratio)
cv2.putText(detectedImage, str(angle), (20, image.shape[0]-30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255,255,0), thickness=2)
# print(f"width: {w}, height: {h}, ratio: {w/h}")
# print(detectionBoxes)
while True:
    
    
    cv2.imshow("Detected image with Angle", detectedImage)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()