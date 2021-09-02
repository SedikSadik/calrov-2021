import cv2
import numpy as np
import math

def ratioToAngle(ratio:float) -> float:
    return 133.2469 - (139.1493*ratio) + (162.7091*(ratio**2)) - (86.84221*(ratio**3))
def ratioToAngle2(ratio:float) -> float:
    return -795.4162 + (91.44677+795.4162)/(1 + (ratio/2.579814)**(3.952141))
def ratioToAngle3(ratio:float)-> float:
    return 685.079 - 2449.909*ratio + 3633.672*ratio**2 - 2315.194*ratio**3 + 518.7528*ratio**4
turn = -1
print(-250*turn)