from .vehicle_classes import OtonomVehicle
from tkinter import *
from PIL import Image, ImageTk
import os





class CALROV_GUI():
    def __init__(self, startAllThreads,toggleVideo, toggleArm, toggleOnOff, vehicle: OtonomVehicle,  title : str="CALROV", icon_path: str = None) -> None:
        self.root = Tk()
        self.root.title(title)
        
        if icon_path is not None:
            _tmp = Image.open(os.path.abspath(icon_path))
            self.icon = ImageTk.PhotoImage(_tmp)
            self.root.tk.call("wm", "iconphoto", self.root._w, self.icon)

        self.title_label = Label(self.root, text = title)
        self.title_label.config(font =("Courier", 20))
        self.title_label.grid(row=0, column=0, columnspan=4)

        self.video_app = Frame(self.root, bg="white")
        self.video_app.grid(row=1,column=0,columnspan=2)

        self.yolo_video = Label(self.video_app)
        self.yolo_video.grid(row=1, column=0)

        self.yolo_fps_label = Label(self.video_app, text="Yolo Fps: 0")
        self.yolo_fps_label.grid(row=2, column=0)

        

        ##Buttons
        self.button_frame = Frame(self.root, bg="white")
        self.button_frame.config(width=416,height=416)
        self.button_frame.grid(row=3, column=0)

        self.start_threads_button = Button(self.button_frame, text="Start all threads", 
                                command=startAllThreads,)
        self.start_threads_button.grid()

        self.start_video_button = Button(self.button_frame, text='Toggle Video',
                                command=toggleVideo,args=)
        self.start_video_button.grid()

        self.toggle_arm_disarm_button = Button(self.button_frame, text='Toggle Arm/Disarm',
                                command=toggleArm)
        self.toggle_arm_disarm_button.grid()

        self.toggle_button = Button(self.button_frame, text='Toggle All Activity', command=toggleOnOff)
        self.toggle_button.grid()


        ##VEHICLE STATUS
        self.status_frame = Frame(self.root)
        self.status_frame.config( bg='gray')
        self.status_frame.grid(row=3, column=1)

        self.arm_status_label = Label(self.status_frame, text="Vehile Status:")
        self.arm_status_label.grid()
        
        self.flight_mode_label = Label(self.status_frame, text="Flight Mode: ")
        self.flight_mode_label.grid()


        self.roll_label = Label(self.status_frame, text="Roll: N/A")
        self.pitch_label = Label(self.status_frame, text="Pitch: N/A")
        self.yaw_label = Label(self.status_frame, text="Yaw: N/A")
        self.depth_label = Label(self.status_frame, text="Depth: N/A")


        self.roll_label.grid()
        self.pitch_label.grid()
        self.yaw_label.grid()
        self.depth_label.grid()


        self.manipulator_servo_label = Label(self.status_frame, text="Servo: N/A")
        self.recognition_label = Label(self.status_frame, text="Object Recognition: N/A")
        self.current_activity_label = Label(self.status_frame, text="Current Activity: ")

        self.manipulator_servo_label.grid()
        self.recognition_label.grid()
        self.current_activity_label.grid()

        self.last_heartbeat_label = Label(self.status_frame, text="Last heartbeat sent at ")
        self.last_heartbeat_label.grid()

        self.target_depth_label = Label(self.status_frame, text="Target Depth: ")
        self.target_depth_label.grid()

        self.target_attitude_label = Label(self.status_frame, text="Target Attitude")
        self.target_attitude_label.grid()
        
        self.current_pwm_label = Label(self.status_frame, text="Current pwm sent = N/A")
        self.current_pwm_label.grid()

if __name__ =="__main__":
    gui = CALROV_GUI( None, None, None, None)
    gui.root.mainloop()