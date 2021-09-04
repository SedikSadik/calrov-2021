import cProfile
from sys import setdlopenflags
import threading
import os
import cv2
import time
from pymavlink import mavutil
import random


master = None

def send_pwm(x =0, y=0 , z = 500, yaw=0 , buttons=0):
    """Send manual pwm to the axis of a joystick. 
    Relative to the vehicle
    x for right-left motion
    y for forward-backwards motion
    z for up-down motion
    r for the yaw axis
        clockwise is -1000
        counterclockwise is 1000
    buttons is an integer with 
    """
    master.mav.manual_control_send(master.target_system,
                                    x,y,z,yaw,buttons)


class OtonomVehicle():
    def __init__(self, connection):
        self.connection = connection
        
        self.kp = 1.0
        self.ki = 0.002
        self.kd = -0.2
        self.preTotal = 0
        self.lastPositionMid = 0
        
        self.turn = 1

        self.currentlyDetected = False
        self.recent_boxes = [[1,2,430,40]]

        
        self.phase_depth_start = threading.Event()
        self.phase_finding_start = threading.Event()
        self.phase_locationtest_start = threading.Event()
        self.phase_alignment_start = threading.Event()
        self.phase_end_start = threading.Event()

        self.phase_depth_start.clear
        self.phase_finding_start.clear()
        self.phase_locationtest_start.clear()
        self.phase_alignment_start.clear()
        self.phase_end_start.clear()

        self.phase_depth_thread = threading.Thread(target=self.phaseOne)
        self.phase_depth_thread.start()

        self.phase_finding_thread = threading.Thread(target=self.phaseTwo)
        self.phase_finding_thread.start()

        self.phase_locationtest_thread = threading.Thread(target=self.phaseThree)
        self.phase_locationtest_thread.start()

        self.phase_alignment_thread = threading.Thread(target=self.phaseFour)
        self.phase_alignment_thread.start()

        self.phase_end_thread = threading.Thread(target=self.phaseFive)
        self.phase_end_thread.start()
    
    @property
    def proportionalYawValue(self):
        return self.kp * (208-self.recent_boxes[0][0])
    
    @property
    def integralYawValue(self):
        self.preTotal += self.ki * (208-self.recent_boxes[0][0])
        if self.preTotal>50:
            self.preTotal=50

        return self.preTotal
    
    @property
    def derivativeYawValue(self):
        currentMid = self.recent_boxes[0][0] 
        Diff = currentMid - self.lastPositionMid
        self.lastPositionMid = currentMid
        return Diff


    def phaseOne(self)->None:
        "Finds the pool depth"
        self.phase_depth_start.wait()


    def phaseTwo(self) -> None:
        """Spin until you find the frame"""
        self.phase_finding_start.wait()  ## IF Phase one is active

        while self.phase_finding_start.is_set():
            if len(self.recent_boxes[0]) != 0: ##self.recent_boxes has a value
                send_pwm(yaw=self.proportionalYawValue+self.integralYawValue+self.derivativeYawValue)
                if 178 < self.recent_boxes[0] < 238: ##Close to center 60 pixel leniency
                    self.phase_finding_start.clear()
                    self.phase_locationtest_start.set()
            else:
                send_pwm(yaw=400)

        self.phaseTwo()
    
    def phaseThree(self) -> None:
        """Determines the turn direction"""
        self.phase_locationtest_start.wait() ## IF Phase two is active
        
        while self.phase_locationtest_start.is_set():
            startTime = time.time()
            startRatio = self.recent_boxes[0][2]/self.recent_boxes[0][3]

            while time.time()<startTime+6:
                send_pwm(y=500*self.turn, yaw=self.proportionalYawValue) ##c-clockwise
            endRatio = self.recent_boxes[0][2]/self.recent_boxes[0][3]
            if endRatio>startRatio:
                self.turn = 1 ## c-clockwise turn around the frame
            else:
                self.turn = -1 ## clockwise turn
            self.phase_locationtest_start.clear()
            self.phase_alignment_start.set()

        self.phaseThree()
            

    def phaseFour(self)-> None:
        self.phase_alignment_start.wait()  ## IF Phase three is active
        while self.phase_alignment_start.is_set():
            if self.recent_boxes[0][2]/self.recent_boxes[0][3]>1.3 and self.recent_boxes[0][0]<238 and self.recent_boxes[0][0]>178: ##at most 60 pixel fail
                self.phase_alignment_start.clear()
                self.phase_end_start.set()
            else:
                send_pwm(y= 400*self.turn, yaw=self.proportionalYawValue)
          

        self.phaseFour()

    def phaseFive(self) -> None:
        self.phase_end_start.wait()   ## IF Phase four is active
        
        while self.phase_end_start.is_set():
            #send_pwm(x=400) #If yaw makes it worse somehow
            send_pwm(x=400, yaw=self.proportionalYawValue/3) ## Yaw tries to correct mistakes
           
            
            

        self.phaseFive()

# print(f'Mid X value{self.recent_boxes[0][0]}')
# print(f'Derivative {self.derivativeYawValue}')
# print(f'Integral {self.integralYawValue}')
# print(f'Proportional {self.proportionalYawValue}')
#addition = 100
# print(f'added {addition}')

