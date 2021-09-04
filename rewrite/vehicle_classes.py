import numpy as np
import cv2
import sys
import math
from pymavlink.quaternion import QuaternionBase
import threading
import time

from pymavlink import mavutil

# Video Class ----------------------------


# Vehicle Class


class OtonomVehicle():
    def __init__(self, connection):
        self.connection = connection
        self.boot_time = time.time()

        self.kp = 1.0
        self.ki = 0.002
        self.kd = 0.2
        self.preTotal = 0
        self.lastDetectedMidX = 0

        self.turn = 1

        self.currentlyDetected = False
        # recentBoxes should ALWAYS be a list with 4 coords
        self.recent_boxes = [208, 208, 1, 1]

        self.PIDUpToDate = threading.Event()  # is pid values up to date?
        self.PIDUpToDate.clear()  # init as not up to date
        self.boxLock = threading.Lock()

        self.updater_thread = threading.Thread(target=self.PIDUpdater)
        self.updater_thread.start()

        self.status_update_event = threading.Event()

    def PIDUpdater(self):
        while self.currentlyDetected:
            # Safely get a tmp recent boxes
            # self.boxLock.acquire()
            pass
            #tmpBoxes = self.recent_boxes
            # self.boxLock.release()

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

    def setTargetDepth(self, depth):

        self.connection.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - self.boot_time)),  # ms since boot
            self.connection.target_system, self.connection.target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask=0xdfb,  # ignore everything except z position
            # (x, y WGS84 frame pos - not used), z [m]
            lat_int=0, lon_int=0, alt=depth,
            vx=0, vy=0, vz=0,  # velocities in NED frame [m/s] (not used)
            afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
            # accelerations in NED frame [N], yaw, yaw_rate
            #  (all not supported yet, ignored in GCS Mavlink)
        )

    def setTargetAttitude(self, roll=0, pitch=0, yaw=0):
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

    def statusUpdate(self):
        self.status_update_event.wait()

    def sendPwm(self, x=0, y=0, z=500, yaw=0, buttons=0):
        print("sent pwm")
        #self.connection.mav.manual_control_send(self.connection.target_system, x,y,z,yaw,buttons)

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
        flight_mode_label.config(text=f"Flight Mode: {mode_name}")
        mode_id = self.connection.mode_mapping()[mode_name]
        self.connection.mav.set_mode_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
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


talay = OtonomVehicle(200)
driver = Driver(talay)
