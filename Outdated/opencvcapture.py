import numpy as np
import cv2 as cv
cap = cv.VideoCapture(1)
from time import time

if not cap.isOpened():
    print("Cannot open camera")
    exit()
face_cascade = cv.CascadeClassifier(cv.data.haarcascades+'haarcascade_frontalface_default.xml')
global frame_count
frame_count=0
start_time = time()

fps = frame_count / (time()-start_time)
while True:
    
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        #print("Can't receive frame (stream end?). Exiting ...")
        
        break
    frame_count += 1
    # Our operations on the frame come here
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)

    for (x,y,w,h) in faces:
        cv.rectangle(frame, (x,y), (x+w, y+h), (255,0,0), 2)
    # Display the resulting frame
    fps = float(frame_count / (time()-start_time))
    fps_text = 'FPS: {:.2f}'.format(fps)
    fpsphoto = cv.putText(frame, fps_text, (25,25), cv.FONT_HERSHEY_SIMPLEX, 1, (255,0,0),)
    cv.imshow('frame', fpsphoto)
    
    if cv.waitKey(1) == ord('q'):
        break 
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()