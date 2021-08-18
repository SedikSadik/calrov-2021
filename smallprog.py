import pymavlink
from pymavlink import mavutil
master = mavutil.mavlink_connection("udpin:192.168.2.1:14550")
master.wait_heartbeat()
print(master.mode_mapping())