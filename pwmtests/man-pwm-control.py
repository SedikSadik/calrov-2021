from pymavlink import mavutil
from time import sleep
from time import time as TM
from datetime import datetime as dt
import sys

#udpin:192.168.2.1:14550

master = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
master.wait_heartbeat()
mode = 'STABILIZE'

# Check if mode is available
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)

mode_id = master.mode_mapping()[mode]
# Set new mode
# master.mav.command_long_send(
#    master.target_system, master.target_component,
#    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#    0, mode_id, 0, 0, 0, 0, 0) or:
# master.set_mode(mode_id) or:
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

while True:
    # Wait for ACK command
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    # Check if command in the same in `set_mode`
    if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
        continue

    # Print the ACK result !
    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break

master.arducopter_arm()
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')


def run_motors(run_time, x=0, y=0, z=0, yaw = 0, buttons = 0):
    """run_time = for how long is pwm sent"""
    t_start = int(TM())
    print(f"running for {run_time} seconds, with pwm x:{x},y={y}, z={z} \n yaw:{yaw}, buttons:{buttons}")
    while TM() < t_start+run_time:
        master.mav.manual_control_send(master.target_system, x,y,z,yaw,buttons)

run_motors(12, y=400)

master.arducopter_disarm()
master.motors_disarmed_wait()