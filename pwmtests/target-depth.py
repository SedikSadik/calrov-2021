from pymavlink import mavutil
import time



master = mavutil.mavlink_connection("udpin:192.168.2.1:14550")
master.wait_heartbeat()
boot_time = time.time()
print("Successful Connection")

def set_target_depth(depth):
    """ Sets the target depth while in depth-hold mode.

    Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

    'depth' is technically an altitude, so set as negative meters below the surface
        -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

    """
    master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=0xdfe, 
        #type_mask = 0b110111111110 ,  # ignore everything except z position
        lat_int=0, lon_int=0, alt=depth, # (x, y WGS84 frame pos - not used), z [m]
        vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )
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


def run_motors(run_time, x=0, y=0, z=500, yaw = 0, buttons = 0):
    """run_time = for how long is pwm sent"""
    t_start = time()
    print(f"running for {run_time} seconds, with pwm x:{x},y={y}, z={z} \n yaw:{yaw}, buttons:{buttons}")
    while time() < t_start+run_time:
        master.mav.manual_control_send(master.target_system, x,y,z,yaw,buttons)


master.arducopter_arm()
master.motors_armed_wait()

DEPTH_HOLD = 'ALT_HOLD'
DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]

while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
    master.set_mode(DEPTH_HOLD)

set_target_depth(-1.0)

run_motors(yaw=400, run_time=10)


master.arducopter_disarm()
master.motors_disarmed_wait()

