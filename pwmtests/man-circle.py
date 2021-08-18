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

master.arducopter_arm()
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')
t_start = int(TM())
run_time = 5


while TM() < t_start+run_time and master.motors_armed():
    '''circle'''
    send_pwm(x=600, roll=300)
master.arducopter_disarm()
master.motors_disarmed_wait()