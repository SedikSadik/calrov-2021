import cv2
import os 
import numpy as np




net = cv2.dnn.readNet(os.path.abspath('./GUI/Yolo_files/yolo-custom_3000.weights'),os.path.abspath('./GUI/Yolo_files/yolo-custom.cfg'))
detection_classes = []
with open(os.path.abspath('./GUI/Yolo_files/obj.names'), 'r') as f:
    detection_classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0]-1] for i in net.getUnconnectedOutLayers()]

def yolo_detection(raw_image):
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

    for topleft_x, topleft_y, w,h in new_boxes:
        label = "otonomkapi"
        cv2.rectangle(raw_image, (topleft_x,topleft_y), (topleft_x+w,topleft_y+h), (0,100,255), 1)
        cv2.putText(raw_image, label, (topleft_x, topleft_y),cv2.FONT_HERSHEY_COMPLEX,1,(0,165,255))
