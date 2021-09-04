from pymavlink import mavutil
import time


master = mavutil.mavlink_connection("udpin:192.168.2.1:14550")
master.wait_heartbeat()

boot_time = time.time()
print("Successful Connection")


def set_target_depth(depth):

    master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=0xdfb, 
        lat_int=0, lon_int=0, alt=depth,
        vx=0, vy=0, vz=0, 
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
    
    )


def send_pwm(x =0, y=0 , z = 500, yaw=0 , buttons=0):
    """Send manual pwm to the axis of a joystick. 
    Relative to the vehicle
    x for right-left motion
    y for forward-backwards motion
    z for up-down motion, z -> {0-1000}
    r for the yaw axis
        clockwise is -1000
        counterclockwise is 1000
    buttons is an integer with 
    """
    print(f"running for  seconds, with pwm x:{x},y={y}, z={z} \n yaw:{yaw}, buttons:{buttons}")

    master.mav.manual_control_send(master.target_system, x,y,z,yaw,buttons)


##Arm Vehicle
master.arducopter_arm()
master.motors_armed_wait()

#Flight Mode Setup
FLIGHT_MODE = 'ALT_HOLD'
FLIGHT_MODE_ID = master.mode_mapping()[FLIGHT_MODE]

while not master.wait_heartbeat().custom_mode == FLIGHT_MODE_ID:
    master.set_mode(FLIGHT_MODE)


##Run motors for 20 seconds
tstart = time.time()
set_target_depth(-1.5)
while time.time()<tstart+20:
    
    send_pwm(yaw = 600)

master.arducopter_disarm()
master.motors_disarmed_wait()

def setTargetAttitude(roll=0, pitch=0, yaw=0):
    
    bitmask = 1<<6

    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        bitmask,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
    )

def run_motors(run_time, x=0, y=0, z=500, yaw = 0, buttons = 0):
    """run_time = for how long is pwm sent"""
    t_start = time.time()
    print(f"running for {run_time} seconds, with pwm x:{x},y={y}, z={z} \n yaw:{yaw}, buttons:{buttons}")
    while time.time() < t_start+run_time:
        master.mav.manual_control_send(master.target_system, x,y,z,yaw,buttons)
