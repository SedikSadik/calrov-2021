import cv2
import numpy as np
color = np.uint8([[[90,140,120]]])
print(cv2.cvtColor(color,cv2.COLOR_BGR2HSV))