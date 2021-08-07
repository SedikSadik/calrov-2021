import time
from pymavlink import mavutil
import sys
master = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
master.wait_heartbeat()

def request_message_interval(message_id: int, frequency_hz: float):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # MAVLINK mesaj tanimi
        1e6 / frequency_hz, # Istenen frekans
        0, # Hedef, (0=arac)
        0, 0, 0, 0)

# AHRS2 ayarının 5 Hertz olarak ayarlanması
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 5)

# ATTITUDE mesajının 15 hertz olarak ayarlanması
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 15)

# Veri alma 
while True:
    try:
        packet = master.recv_match().to_dict()
        if packet['mavpackettype']=='ATTITUDE':
            print(packet['roll'])
    except KeyboardInterrupt:
        sys.exit()
    except:
        pass

    