import cv2
import numpy as np
import os

image=cv2.imread('/home/violetcheese/Desktop/photo_naming/mixbatches/batch8frame6939.jpg')
imgContour = image.copy()
lower = [50, 110, 60]
upper = [90, 160, 110]
def empty():
    pass
cv2.namedWindow("Parametres")
cv2.resizeWindow("Parametres", 416,416)
cv2.createTrackbar("Threshold1", "Parametres", 120, 255, empty)
cv2.createTrackbar("Threshold2", "Parametres", 255, 255, empty)
def getContours(img, imgContour):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    cnt_areas = [cv2.contourArea(cnt) for cnt in contours]
    cnt_areas.sort()
    max_area_cnt = cnt_areas[-1]
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area>1000:
            cv2.drawContours(cnt, contours, -1, (255,0,255),3)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02*peri, True)
            x,y,w,h = cv2.boundingRect(approx)
            cv2.rectangle(imgContour, (x,y), (x+w,y+h),(0,255,0),2)



while True:
    threshold1 = cv2.getTrackbarPos("Threshold1", "Parametres")
    threshold2 = cv2.getTrackbarPos("Threshold2", "Parametres")

    # create NumPy arrays from the boundaries
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")
    # find the colors within the specified boundaries and apply
    # the mask
    mask = cv2.inRange(image, lower, upper)
    output = cv2.bitwise_and(image, image, mask = mask)

    imgBlur = cv2.GaussianBlur(output, (7,7),1)
    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
    imgCanny = cv2.Canny(imgBlur, threshold1, threshold2)

    kernel = np.ones((5,5))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
    getContours(imgDil, output)
    imgDilToBGR = cv2.cvtColor(imgDil, cv2.COLOR_GRAY2BGR)
    # show the images
    cv2.imshow("images", np.hstack([imgDilToBGR, output]))
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break