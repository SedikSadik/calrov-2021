import cv2
cimport numpy
import numpy

#unsigned char [:,:,:] raw_image
cpdef list yolo_detection(raw_image, net: cv2.dnn_Net, list output_layers,list detection_classes):
    """Take in as input a cv2 image"""
    cdef list class_ids = []
    cdef list confidences = []
    cdef list boxes = []
    cdef int height = raw_image.shape[0]
    cdef int width = raw_image.shape[1]
    
    cdef numpy.ndarray blob 
    blob = cv2.dnn.blobFromImage(raw_image, 0.00392, (416,416), (0,0,0), True, crop=False)
    net.setInput(blob)
    cdef list outs = net.forward(output_layers)
    cdef numpy.ndarray out
    cdef numpy.ndarray detection
    cdef numpy.ndarray scores
    cdef int class_id 
    cdef float confidence
    cdef int center_x, center_y, w, h, topleft_x, topleft_y
    
    
    for out in outs:
        for detection in out:
            
            scores = detection[5:]
            
            class_id = numpy.argmax(scores)
            
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
    cdef list new_boxes 
    if len(indexes) !=0:

        new_boxes = [boxes[index] for index in indexes[0] if indexes[0] is not None]
        return new_boxes
    else:
        return [[]]
