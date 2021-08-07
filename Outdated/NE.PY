from pymavlink import mavutil
from time import sleep
from time import time as TM
from datetime import datetime as dt


pixhawk = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
pixhawk.wait_heartbeat() 
print("heartbeat found!")

def pwm_gonder(channel_id, pwm=1500):
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    pixhawk.mav.rc_channels_override_send(
        pixhawk.target_system,                
        pixhawk.target_component,            
        *rc_channel_values)

pixhawk.arducopter_arm()
