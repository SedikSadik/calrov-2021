import cv2
import numpy as np
import time

### CAPTURE OBJECT
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()



### YOLO and DETECTION
## Loading Yolo
net = cv2.dnn.readNet('/home/violetcheese/Documents/CALROV/GUI/Yolo Files/yolov3.weights','/home/violetcheese/Documents/CALROV/GUI/Yolo Files/yolov3.cfg')
detection_classes = []

with open('/home/violetcheese/Documents/CALROV/GUI/Yolo Files/coco.names', 'r') as f:
    detection_classes = [line.strip() for line in f.readlines()]

layer_names = net.getLayerNames()
output_layers = [layer_names[i[0]-1] for i in net.getUnconnectedOutLayers()]


## LOAD IMAGES
""" img = cv2.imread('/home/violetcheese/Documents/CALROV/GUI/gui_images/CALROV.jpeg')
img = cv2.resize(img, None, fx=0.4, fy=0.4) """
## DETECT OBJECTS
""" blob = cv2.dnn.blobFromImage(img, 0.00392, (416,416), (0,0,0), True, crop=False) """
'''
for b in blob:
    for n, imgblob in enumerate(b):
        cv2.imshow(str(n), imgblob)
'''



if __name__ == '__main__':
    while True:
        ret, img = cap.read()
        if ret:
            img = cv2.resize(img, None, fx=0.4, fy=0.4)
            class_ids = []
            confidences = []
            boxes = []
            height , width, channels = img.shape
            blob = cv2.dnn.blobFromImage(img, 0.00392, (416,416), (0,0,0), True, crop=False)
            net.setInput(blob)
            outs = net.forward(output_layers)

            ##List of Detections
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
                    cv2.rectangle(img, (topleft_x,topleft_y), (topleft_x+w,topleft_y+h), (0,100,255), 1)
                    cv2.putText(img, label, (topleft_x, topleft_y),cv2.FONT_HERSHEY_COMPLEX,1,(0,165,255))
        


            cv2.imshow("CALROV PROTOTYPE 3", img)
            #time.sleep(1)
        if cv2.waitKey(1) == ord('q'):
            break