from pymavlink import mavutil
from time import sleep
from datetime import datetime as dt


pixhawk = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
pixhawk.wait_heartbeat() 
#sleep(3)


def pwm_gonder(channel_id, pwm=1500):
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    pixhawk.mav.rc_channels_override_send(
        pixhawk.target_system,                
        pixhawk.target_component,            
        *rc_channel_values)


def on_saniye(): 
    zaman = int(dt.now().timestamp()) 
    while(True): 
        if(zaman+10 != int(dt.now().timestamp())):
            pwm_gonder(5) 
        else: 
            break 

on_saniye()

