import threading
from time import time, sleep
from pymavlink import mavutil
import sys
##Vehicle Connection
master = mavutil.mavlink_connection('192.168.2.1:14450')
master.wait_heartbeat()

starttime = time()
operationtime = time() - starttime
last_updatedtime = time()
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

def Master_pwm(channel_id=3, step=1):
    global operationtime
    #currentpwm = startpwm
    sleep(5)
    teststarttime = time()
    sleep(0.1)
    while True:
        if time() > teststarttime      and  time()<teststarttime+5:
            pwm_gonder(channel_id,1900)
        if time() > teststarttime+5    and  time()<teststarttime+10:
            pwm_gonder(channel_id,1800)
        if time() > teststarttime+10   and  time()<teststarttime+15:
            pwm_gonder(channel_id,1700)
        if time() > teststarttime+15   and  time()<teststarttime+20:
            pwm_gonder(channel_id,1600)
        if time() > teststarttime+20   and  time()<teststarttime+25:
            pwm_gonder(channel_id,1500)
        if time() > teststarttime+25   and  time()<teststarttime+30:
            pwm_gonder(channel_id,1400)
        if time() > teststarttime+30   and  time()<teststarttime+35:
            pwm_gonder(channel_id,1300)
        if time() > teststarttime+35   and  time()<teststarttime+40:
            pwm_gonder(channel_id,1200)
        if time() > teststarttime+40   and  time()<teststarttime+45:
            pwm_gonder(channel_id,1100)
        
        
    
    



def updatetime():
    global starttime
    global operationtime
    global last_updatedtime
    while True:
        operationtime = time()-starttime
        last_updatedtime = time()
##Vehicle Conection


# AHRS2 ayarının 5 Hertz olarak ayarlanması
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 5)

# ATTITUDE mesajının 15 hertz olarak ayarlanması
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 15)


##Threadings

data_thread = threading.Thread(target=veri_al)
pwm_thread = threading.Thread(target=Master_pwm, args=[1100,1500,4])
time_thread = threading.Thread(target=updatetime)


data_thread.start()
time_thread.start()
pwm_thread.start()

data_thread.join()
time_thread.join()
pwm_thread.join()
