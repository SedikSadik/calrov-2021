import threading
from time import time, sleep
from pymavlink import mavutil
import sys
##Vehicle Connection
master = mavutil.mavlink_connection('192.168.2.1:14450')
master.wait_heartbeat()
print("Conection Successful")
#DEFINITION
def veri_al():
    while True:
        try:
            packet = master.recv_match().to_dict()
            if packet['mavpackettype']=='ATTITUDE':
                print(f"Roll: {packet['roll']}\n  Pitch: {packet['pitch']}\n Yaw: {packet['yaw']}")
        except:
            pass
def request_message_interval(message_id: int, frequency_hz: float):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # MAVLINK mesaj tanimi
        1e6 / frequency_hz, # Istenen frekans
        0, # Hedef, (0=arac)
        0, 0, 0, 0)
def pwm_gonder(channel_id, pwm=1500):
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                
        master.target_component,            
        *rc_channel_values)

def Master_pwm(startpwm, stoppwm,step=1):
    if startpwm<stoppwm:
        pwm_start = startpwm
        while pwm_start < stoppwm:
            pwm_gonder(3, pwm_start)
            pwm_start += step
            sleep(0.02)
            print(f'Pwm: {pwm_start}')
    elif startpwm>stoppwm:
        pwm_start = startpwm
        while pwm_start < stoppwm:
            pwm_gonder(3, pwm_start)
            pwm_start -= step
            sleep(0.02)
            print(f'Pwm: {pwm_start}')
##Vehicle Conection


# AHRS2 ayarının 5 Hertz olarak ayarlanması
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 5)

# ATTITUDE mesajının 15 hertz olarak ayarlanması
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 15)


##Threadings

data_thread = threading.Thread(target=veri_al)
pwm_thread = threading.Thread(target=Master_pwm, args=[1100,1500,4])
data_thread.start()
data_thread.join()