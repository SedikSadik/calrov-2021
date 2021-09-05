import tkinter
import numpy as np
import cv2
import sys
import math
from numpy.lib.arraysetops import isin
from pymavlink.quaternion import QuaternionBase
import threading
import time

from pymavlink import mavutil

# Vehicle Class

class OtonomVehicle():
    def __init__(self, connection ):
        self.connection = connection
        #self.connection.wait_heartbeat()
        self.boot_time = time.time()

        self.kp = 1.0
        self.ki = 0.002
        self.kd = 0.2
        self.preTotal = 0
        self.lastDetectedMidX = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.current_depth = 0

        self.turn = 1 ##Aligment phase turn direction

        # recentBoxes should ALWAYS be a list with 4 coords
        self.currentlyDetected = threading.Event()
        self.currentlyDetected.clear()

        self.recent_boxes = [208, 208, 1, 1]

        # self.PIDUpToDate = threading.Event()  # is pid values up to date?
        # self.PIDUpToDate.clear()  # init as not up to date
        self.boxLock = threading.Lock()

        self.updater_thread = threading.Thread(target=self.PIDUpdater)
        #self.updater_thread.start()

        self.status_update_event = threading.Event()
        self.status_update_event.clear()
        self.status_update_thread = threading.Thread()
        
        self.video_on_event = threading.Event()
        self.video_on_event.clear()
        
        #self.video_thread :threading.Thread
        print("vehicle init success")
    
    def PIDUpdater(self):
        self.currentlyDetected.wait()
        while self.currentlyDetected.is_set():
            # Safely get a tmp recent boxes
            # self.boxLock.acquire()
            pass
            #tmpBoxes = self.recent_boxes
            # self.boxLock.release()
        self.PIDUpdater()
    
    @property
    def distanceToMid(self):
        return self.recent_boxes[0][0]-208

    @property
    def proportionalYawValue(self):
        return self.kp * self.distanceToMid

    @property
    def integralYawValue(self):
        self.preTotal += self.ki * self.distanceToMid
        if self.preTotal > 50:
            self.preTotal = 50

        return self.preTotal

    @property
    def derivativeYawValue(self):
        currentMid = self.recent_boxes[0][0]
        Diff = currentMid - self.lastDetectedMidX
        self.lastDetectedMidX = currentMid
        return Diff*self.kd

    def setTargetDepth(self, depth, vehicle_target_depth_property = None, target_depth_label = None):
        
        self.connection.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - self.boot_time)), # ms since boot
            self.connection.target_system, self.connection.target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask=( # ignore everything except z position
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            ), lat_int=0, lon_int=0, alt=depth, # (x, y WGS84 frame pos - not used), z [m]
            vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
            afx=0, afy=0, afz=0, yaw=0, yaw_rate=0)
        if vehicle_target_depth_property is not None:
            vehicle_target_depth_property = depth
        if target_depth_label is not None and isinstance(target_depth_label,tkinter.Label):
            target_depth_label.config(text=f"Target Attitude Set to {depth}")

    def setTargetAttitude(self, roll=0, pitch=0, yaw=0, target_attitude_label = None):
        """ Sets the target attitude while in depth-hold mode.

        'roll', 'pitch', and 'yaw' are angles in degrees.

        """
        # https://mavlink.io/en/messages/common.html#ATTITUDE_TARGET_TYPEMASK
        # 1<<6 = THROTTLE_IGNORE -> allow throttle to be controlled by depth_hold mode
        bitmask = 1 << 6

        self.connection.mav.set_attitude_target_send(
            int(1e3 * (time.time() - self.boot_time)),  # ms since boot
            self.connection.target_system, self.connection.target_component,
            bitmask,
            # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
            QuaternionBase([math.radians(angle)
                           for angle in (roll, pitch, yaw)]),
            0, 0, 0, 0  # roll rate, pitch rate, yaw rate, thrust
        )
        if isinstance(target_attitude_label, tkinter.Label):
            target_attitude_label.config(text=f"Target Attitude: Roll={roll}, Pitch={yaw},Yaw={yaw}")

    def sendPwm(self, x=0, y=0, z=500, yaw=0, buttons=0, pwm_label = None):
        print("sent pwm")
        
        #self.connection.mav.manual_control_send(self.connection.target_system, x,y,z,yaw,buttons)
        if isinstance(pwm_label, tkinter.Label):
            pwm_label.config(text=f"Pwm: x={x}, y={y}, z={z}, {yaw}")
    
    def requestMessageInterval(self, message_id: int, frequency_hz: float):
        """
        Request MAVLink message in a desired frequency,
        documentation for SET_MESSAGE_INTERVAL:
            https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

        Args:
            message_id (int): MAVLink message ID
            frequency_hz (float): Desired frequency in Hz
        """
        self.connection.mav.command_long_send(
            self.connection.target_system, self.connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id,  # MAVLINK mesaj tanimi
            1e6 / frequency_hz,  # Istenen frekans
            0,  # Hedef, (0=arac)
            0, 0, 0, 0)

    def cameraGimbalSet(self, tilt, roll=0, pan=0, camera_gimbal_label=None):
        """
        Gimbal istenen pozisyona taşınır.
        Args:
            tilt (float): santiderece cinsinden tilt (0 nötr)
            roll (float, isteğe bağlı): santiderece cinsinden roll (0 nötr)
            pan  (float, isteğe bağlı): santiderece cinsinden pan (0 nötr)
        """
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            1,
            tilt,
            roll,
            pan,
            0, 0, 0,
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)

    def flightModeSet(self, mode_name, flight_mode_label=None):

        if mode_name not in self.connection.mode_mapping():
            print('Unknown mode : {}'.format(mode_name))
            print('Try:', list(self.connection.mode_mapping().keys()))
            sys.exit(1)
        
        mode_id = self.connection.mode_mapping()[mode_name]
        self.connection.mav.set_mode_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        if flight_mode_label is not None and isinstance(flight_mode_label, tkinter.Label):
            flight_mode_label.config(text=f"Flight Mode: {mode_name}")

    def toggleArm(self, arm_status_label = None):
        if self.connection.motors_armed():
            self.connection.arducopter_disarm()
            self.connection.motors_disarmed_wait()
            if arm_status_label is not None and isinstance(arm_status_label, tkinter.Label):
                arm_status_label.config(text="Vehicle Status: Disarmed")
        else:
            self.connection.arducopter_arm()
            self.connection.motors_armed_wait()
            if arm_status_label is not None and isinstance(arm_status_label, tkinter.Label):
                arm_status_label.config(text="Vehicle Status: Armed")
    
    def arm(self, arm_status_label = None):
        if arm_status_label is not None and isinstance(arm_status_label, tkinter.Label):
                arm_status_label.config(text="Vehicle Status: Armed")
        self.connection.arducopter_arm()
        self.connection.motors_armed_wait()
    
    def disarm(self, arm_status_label = None):
        if arm_status_label is not None and isinstance(arm_status_label, tkinter.Label):
                arm_status_label.config(text="Vehicle Status: Disarmed")
        self.connection.arducopter_disarm()
        self.connection.motors_disarmed_wait()

    def statusUpdate(self, status_update_event:threading.Event, roll_label:tkinter.Label, pitch_label:tkinter.Label, yaw_label:tkinter.Label, depth_label:tkinter.Label):
        status_update_event.wait()

        while status_update_event.is_set():

            try:
                rcvpacket = self.connection.recv_match(type=['ATTITUDE', 'AHRS2']).to_dict()
                
                roll_value= int(100* rcvpacket['roll']/math.pi)
                pitch_value = int(100* rcvpacket['pitch']/math.pi)
                yaw_value = int(100* rcvpacket['yaw']/math.pi)
                depth_value = float(100* rcvpacket['altitude'])

                roll_label.config(text=f'Roll: {roll_value} %')
                pitch_label.config(text=f'Pitch: {pitch_value} %')
                yaw_label.config(text=f'Yaw: {yaw_value}%')
                depth_label.config(text=f"Depth: {depth_value} cm")

                
            except:

                rcvpacket = None
            
        
            
        self.statusUpdate(status_update_event, roll_label,pitch_label, yaw_label, depth_label)  









# Driver -------------------------------------


class Driver():
    def __init__(self, vehicle: OtonomVehicle) -> None:
        # Vehicle Init
        self.vehicle: OtonomVehicle = vehicle

        # Phase Events
        self.phase_depth_start = threading.Event()
        self.phase_find_frame_event = threading.Event()
        self.phase_locationtest_start = threading.Event()
        self.phase_alignment_start = threading.Event()
        self.phase_end_start = threading.Event()

        # Disable All Phases
        self.phase_depth_start.clear()
        self.phase_find_frame_event.clear()
        self.phase_locationtest_start.clear()
        self.phase_alignment_start.clear()
        self.phase_end_start.clear()

        # DIFINE PHASE THREADS AND START ALL THREADS
        self.phase_depth_thread = threading.Thread(
            target=self.phaseOne, args=(self.vehicle,))
        self.phase_depth_thread.start()

        self.phase_find_frame_thread = threading.Thread(
            target=self.phaseTwo, args=(self.vehicle,))
        self.phase_find_frame_thread.start()

        self.phase_locationtest_thread = threading.Thread(
            target=self.phaseThree, args=(self.vehicle,))
        self.phase_locationtest_thread.start()

        self.phase_alignment_thread = threading.Thread(target=self.phaseFour,
                                                       args=(self.vehicle,))
        self.phase_alignment_thread.start()

        self.phase_end_thread = threading.Thread(target=self.phaseFive,
                                                 args=(self.vehicle,))
        self.phase_end_thread.start()

    def phaseOne(self, vehicle: OtonomVehicle) -> None:
        "Finds the pool depth"
        self.phase_depth_start.wait()
        print("Phase one has begun")

    def phaseTwo(self, vehicle: OtonomVehicle) -> None:
        """Spin until you find the frame"""
        self.phase_find_frame_event.wait()  # IF Phase one is active

        while self.phase_find_frame_event.is_set():
            if len(self.recent_boxes[0]) != 0:  # self.recent_boxes has a value
                self.vehicle.sendPwm(yaw=self.proportionalYawValue +
                                     self.integralYawValue+self.derivativeYawValue)
                # Close to center 60 pixel leniency
                if 178 < self.recent_boxes[0][0] < 238:
                    self.phase_find_frame_event.clear()
                    self.phase_locationtest_start.set()
            else:
                self.vehicle.sendPwm(yaw=400)

        self.phaseTwo()

    def phaseThree(self, vehicle: OtonomVehicle) -> None:
        """Determines the turn direction"""
        self.phase_locationtest_start.wait()  # IF Phase two is active

        while self.phase_locationtest_start.is_set():
            startTime = time.time()
            startRatio = self.recent_boxes[0][2]/self.recent_boxes[0][3]

            while time.time() < startTime+6:
                # c-clockwise
                vehicle.sendPwm(y=500*self.turn, yaw=self.proportionalYawValue)
            endRatio = self.recent_boxes[0][2]/self.recent_boxes[0][3]
            if endRatio > startRatio:
                self.turn = 1  # c-clockwise turn around the frame
            else:
                self.turn = -1  # clockwise turn
            self.phase_locationtest_start.clear()
            self.phase_alignment_start.set()

        self.phaseThree()

    def phaseFour(self, vehicle: OtonomVehicle) -> None:
        self.phase_alignment_start.wait()  # IF Phase three is active
        while self.phase_alignment_start.is_set():
            # at most 60 pixel fail
            if self.recent_boxes[0][2]/self.recent_boxes[0][3] > 1.3 and self.recent_boxes[0][0] < 238 and self.recent_boxes[0][0] > 178:
                self.phase_alignment_start.clear()
                self.phase_end_start.set()
            else:
                vehicle.sendPwm(y=400*self.turn, yaw=self.proportionalYawValue)

        self.phaseFour()

    def phaseFive(self, vehicle) -> None:
        self.phase_end_start.wait()  # IF Phase four is active

        while self.phase_end_start.is_set():
            # vehicle.sendPwm(x=400) #If yaw makes it worse somehow
            # Yaw tries to correct mistakes
            vehicle.sendPwm(x=400, yaw=self.proportionalYawValue/3)

        self.phaseFive()

if __name__=="__main__":

    talay = OtonomVehicle(200)
    driver = Driver(talay)
