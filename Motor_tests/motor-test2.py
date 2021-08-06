from pymavlink import mavutil
from time import sleep
from time import time as TM
from datetime import datetime as dt
import sys
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
mode = 'MANUAL'

# Check if mode is available
if mode not in pixhawk.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(pixhawk.mode_mapping().keys()))
    sys.exit(1)

# Get mode ID
# # Set new mode
# master.mav.command_long_send(
#    master.target_system, master.target_component,
#    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#    0, mode_id, 0, 0, 0, 0, 0) or:
# master.set_mode(mode_id) or:
mode_id = pixhawk.mode_mapping()[mode]

pixhawk.mav.set_mode_send(
    pixhawk.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)
pixhawk.arducopter_arm()
def on_saniye(): # on_saniye diye bir fonksiyon oluştur
    """
    zaman = int(dt.now().timestamp()) #anlık zamanı a # robotu arm et
    dakika = dt.now().minute
    pwm=1100
    optime = int(dt.now().timestamp())
    """
    endtime = TM()+5
    while TM() < endtime:
        pwm_gonder(3,1900)
    """   
    endtime = TM()+5
    while TM() < endtime:
        pwm_gonder(3,1100)
    endtime = TM()+5
    print("Test11")
    while TM() < endtime:
        pwm_gonder(3,1200)
    endtime = TM()+5
    print("Test12")
    while TM() < endtime:
        pwm_gonder(3,1300)
    endtime = TM()+5
    print("Test13")
    while TM() < endtime:
        pwm_gonder(3,1400)
    endtime = TM()+5
    print("Test14")
    #while TM() < endtime:
    #    pwm_gonder(3,1500)
    endtime = TM()+5
    print("Test15")
    while TM() < endtime:
        pwm_gonder(3,1600)
    endtime = TM()+5
    print("Test16")
    while TM() < endtime:
        pwm_gonder(3,1700)
    endtime = TM()+5
    print("Test17")
    while TM() < endtime:
        pwm_gonder(3,1800)
    endtime = TM()+5
    print("Test18")
    while TM() < endtime:
        pwm_gonder(3,1900)
    endtime = TM()+5
    """
  
    
on_saniye()

pixhawk.arducopter_disarm()
sleep(0.3)
#pixhawk.arducopter_disarm()