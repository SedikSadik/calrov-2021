from pymavlink import mavutil
from time import sleep, time
from time import time as TM
import sys


master = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
master.wait_heartbeat() 
print("heartbeat found!")


mode = 'MANUAL'

if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)
mode_id = master.mode_mapping()[mode]

master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)
sleep(0.1)


### DEFINITIONS
def pwm_gonder(channel_id, pwm=1500):
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                
        master.target_component,            
        *rc_channel_values)


def req_msg_intvl(message_id: int, frequency_hz: float):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # MAVLINK mesaj tanimi
        1e6 / frequency_hz, # Istenen frekans
        0, # Hedef, (0=arac)
        0, 0, 0, 0)


def Motor_pwm(pwm, time=5):
    endtime = TM()+time
    while TM() < endtime:
        pwm_gonder(3,pwm)

# AHRS2 ayarının 5 Hertz olarak ayarlanması
req_msg_intvl(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 5)

# ATTITUDE mesajının 15 hertz olarak ayarlanması
req_msg_intvl(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 15)
req_msg_intvl(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1)

dtime = TM()+5
while TM() < dtime:
    try:
        packet = master.recv_match().to_dict()
        if packet['mavpackettype']=='ATTITUDE':
            print(packet['roll'])
    except KeyboardInterrupt:
        sys.exit()
    except:
        pass


master.arducopter_arm()
Motor_pwm(1500, time=5)
Motor_pwm(1400, time=8)
Motor_pwm(1200, time=4)
master.arducopter_disarm()