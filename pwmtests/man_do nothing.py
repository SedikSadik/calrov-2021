from pymavlink import mavutil
from time import sleep
from time import time as TM
from datetime import datetime as dt
import sys

master = mavutil.mavlink_connection("udpin:192.168.2.1:14550")
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
def send_pwm(x =0, y=0 , z = 500, yaw=0 , buttons=0):
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
    master.mav.manual_control_send(master.target_system, x,y,z,yaw,buttons)


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

t_start = int(TM())
run_time = 6

t_start = TM()
while TM() < t_start+5:
    '''Do nothing'''
    send_pwm(x=500)

t_start = TM()
while TM() < t_start+8:
    '''Do nothing'''
    send_pwm(z=100)
mode = "ALT_HOLD"

mode_id = master.mode_mapping()[mode]

master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

t_start=TM()
while TM() < t_start+5:
    '''Do nothing'''
    send_pwm(x=500)
    
t_start=TM()
while TM() < t_start+3:
    '''Do nothing'''
    send_pwm(z=700)

t_start=TM()
while TM() < t_start+4:
    '''Do nothing'''
    send_pwm(yaw=800, z=350)
t_start=TM()

while TM() < t_start+4:
    '''Do nothing'''
    send_pwm(yaw=-800, z=350)


master.arducopter_disarm()
master.motors_disarmed_wait()