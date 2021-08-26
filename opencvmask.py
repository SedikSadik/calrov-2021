import cv2
import numpy as np
import os

# photos = []
# for i in range(25):
#     with open(f"example_photos/batch16frame{2386+i}.jpg",'r') as img:
#         photos.append(img)
lower_hsv = np.array([40, 76, 85])
upper_hsv = np.array([76, 164, 172])
for dirpath, dirname, filenames in os.walk("example_photos"):
    for file in filenames:

        photo = cv2.imread(f'example_photos/{file}')
        hsv = cv2.cvtColor(photo, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_hsv, upperb=upper_hsv)
        final = cv2.bitwise_and(photo, photo, mask=mask)
        cv2.imwrite(f"masks/{file}.jpg", final)

