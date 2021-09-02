import cv2
import numpy as np
import os
import time
import cProfile
cap = cv2.VideoCapture(0)


net = cv2.dnn.readNet(os.path.abspath('Yolo_files/yolo-custom_3000.weights'),os.path.abspath('Yolo_files/yolo-custom.cfg'))
detection_classes = []
with open(os.path.abspath('Yolo_files/obj.names'), 'r') as f:
    detection_classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0]-1] for i in net.getUnconnectedOutLayers()]
print(type(output_layers))
# def yolo_detection(raw_image:np.ndarray):
#     """Take in as input a cv2 image"""
#     class_ids = []
#     confidences = []
#     boxes = []
#     height , width = raw_image.shape[:2]
#     blob = cv2.dnn.blobFromImage(raw_image, 0.00392, (416,416), (0,0,0), True, crop=False)
    
#     net.setInput(blob)
#     outs = net.forward(output_layers)
#     #print(type(blob))
#     print(type(net))
#     for out in outs:
        
#         for detection in out:
            
#             scores = detection[5:]
#             class_id = np.argmax(scores)
#             confidence = scores[class_id]
            
#             if confidence > 0.3:
#                 center_x = int(detection[0]*width)
#                 center_y = int(detection[1]*height)
#                 w = int(detection[2]*width)
#                 h = int(detection[3]*height)
#                 ##Rectangle Draw
#                 topleft_x = int(center_x-(w/2))
#                 topleft_y = int(center_y-(h/2))

#                 boxes.append([topleft_x,topleft_y,w,h])
#                 confidences.append(float(confidence))
#                 class_ids.append(class_id)
#     indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
#     print(type(indexes))
#     #DISPLAY DETECTION
#     total_detections = len(boxes)
#     new_boxes = [boxes[index] for index in indexes[0] if indexes[0] is not None]
#     for i in range(total_detections):
#         if i in indexes:
#             topleft_x, topleft_y, w,h = boxes[i]
#             label = detection_classes[class_ids[i]]
#             #recognition_label.config(text=f"{label} bulundu")
#             cv2.rectangle(raw_image, (topleft_x,topleft_y), (topleft_x+w,topleft_y+h), (0,100,255), 1)
#             cv2.putText(raw_image,f"w: {w}, label:{label}", (topleft_x, topleft_y),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,(0,165,255))
#     #print(new_boxes)

#     return raw_image, new_boxes

# ret, frame = cap.read()
# image = cv2.imread("/home/violetcheese/Documents/CALROV/photos/ 1 üst 50 derece 3.jpg")
# b = cythonized.yolo_detection(image)
#print(cProfile.run("yolo_detection(image)"))
PERSON_WIDTH_CM = 48
FOCAL_LENGTH_PX = 462.92

while True:
    start = time.time()
    ret, frame = cap.read()
    try:
        if ret:
            frame = cv2.resize(frame, (416,416))
            boxes = cythonized.yolo_detection(frame)
            
            width = boxes[0][2]
            DISTANCE  = (FOCAL_LENGTH_PX*PERSON_WIDTH_CM)/width
            cv2.putText(frame,f"{str(round(DISTANCE))} cm", (20,20), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,(0,0,0))
            cv2.imshow("detected", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    except:
        pass
    print(1.0/ (time.time()- start))