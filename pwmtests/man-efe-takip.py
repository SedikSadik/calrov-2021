from typing_extensions import runtime
from pymavlink import mavutil
from time import sleep
from time import time as TM
from datetime import datetime as dt
import sys

master = mavutil.mavlink_connection("udpin:192.168.2.1:14550")
master.wait_heartbeat()

def send_pwm(x =0, y=0 , z = 500, roll=0 , buttons=0):
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
    master.mav.manual_control_send(master.target_system, x,y,z,roll,buttons)
t_start = int(TM())
run_time = 10
while TM() < t_start+run_time:
    '''Do nothing'''
    send_pwm()
    print(f"Nothing Done!, ")