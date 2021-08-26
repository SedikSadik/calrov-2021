
import cv2
import numpy as np
import os


def nothing(x):
    pass
def getContours(img, imgContour):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area>3000:
            cv2.drawContours(cnt, contours, -1, (255,0,255), 7)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02*peri, True)
            x,y,w,h = cv2.boundingRect(approx)
            cv2.rectangle(imgContour, (x,y), (x+w,y+h),(0,255,0),2)
def stackImages(scale, imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0],list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0,cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y],(0,0),None,scale, scale )
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)

                if len(imgArray[x][y].shape) == 2:
                    imgArray[x][y]= cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0,0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) ==2:
                imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver
#cap = cv2.VideoCapture(0)
cv2.namedWindow("Trackbars")

cv2.createTrackbar("L - H", "Trackbars", 42, 179, nothing)
cv2.createTrackbar("L - S", "Trackbars", 76, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", 106, 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", 62, 179, nothing)
cv2.createTrackbar("U - S", "Trackbars", 184, 255, nothing)
cv2.createTrackbar("U - V", "Trackbars", 147, 255, nothing)
cv2.createTrackbar("threshold1", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("threshold2", "Trackbars", 255, 255, nothing)


lower_hsv = [21,91,108]
upper_hsv = [62, 186, 142]

def hsv_mask(image_list):
    result_image_list = []
    for img in image_list:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")
        threshold1 = cv2.getTrackbarPos("threshold1", "Trackbars")
        threshold2 = cv2.getTrackbarPos("threshold2", "Trackbars")
        lower_blue = np.array([l_h, l_s, l_v])
        upper_blue = np.array([u_h, u_s, u_v])
        
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        result = cv2.bitwise_and(img, img, mask=mask)
        result_image_list.append(result)
    return result_image_list

images = []
for dirpath, dirname, filenames in os.walk("example_photos"):
    for filename in filenames:
        print(filename)
        img = cv2.imread(f"example_photos/{filename}")
        print(img.shape)
        #img_416 = cv2.resize(img, (416,416))
        images.append(img)
print(len(images))

# three_images = images[:3]
# img1,img2,img3 = three_images[0], three_images[1],three_images[2]
# while True:
#     # imgContour = result.copy()
#     # imgBlur = cv2.GaussianBlur(result, (7,7),1)
#     # imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
#     # imgCanny = cv2.Canny(imgBlur, threshold1, threshold2)

#     # kernel = np.ones((5,5))
#     # imgDil = cv2.dilate(imgCanny, kernel, iterations=1)

#     # getContours(imgDil, imgContour)

#     imageMasks = hsv_mask(three_images)
#     imgStack = stackImages(0.5, (three_images, 
#                                  imageMasks))
#     cv2.imshow("image", imgStack)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break


#cap.release()
cv2.destroyAllWindows()