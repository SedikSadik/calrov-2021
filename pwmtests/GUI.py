from tkinter import *
import os
from PIL import Image, ImageTk
import threading
from otonom_funcs import *

def video_main():
    global recent_boxes
    global detected_image
    if video_on and video.frame_available():
        
        frame = video.frame()
    
        """cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) #BGR RGB dönüşümü
        height, width = (int(cv2image.shape[1]*img_scale),int(cv2image.shape[0]*img_scale))

        scaled_img = cv2.resize(cv2image,(height, width))
        """
        frame = cv2.resize(frame, (416,416))
        detected_image, recent_boxes = yolo_detection(frame)
        detected_image = cv2.cvtColor(detected_image,cv2.COLOR_BGR2RGB)
        img = Image.fromarray(detected_image)
        
        #img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        imgtk = ImageTk.PhotoImage(image=img)
        yolo_video.imgtk = imgtk
        yolo_video.configure(image=imgtk)
        
        #pwm_decide_once(detected_image, recent_boxes)
    yolo_video.after(1,video_main)


### TINTER INITIALIZING
root=Tk()
root.title("CALROV GUI")
##Icon
icon = Image.open(os.path.abspath('./GUI/gui_images/calrov_logo.jpg'))
icon = ImageTk.PhotoImage(icon)
root.tk.call('wm','iconphoto',root._w, icon)

## TITLE TEXT
Title_label = Label(root, text = "CALROV TALAY")
Title_label.config(font =("Courier", 20))
Title_label.grid(row=0, column=0, columnspan=4)

##Live Video Display Box
video_app = Frame(root, bg="white")
video_app.grid(row=1,column=0,columnspan=2)


yolo_video = Label(video_app)
yolo_video.grid(row=1, column=0)
opencv_video = Label(video_app)
opencv_video.grid(row=1, column=1)



yolo_fps_label = Label(video_app, text="Yolo Fps: 0")
yolo_fps_label.grid(row=2, column=0)

opencv_fps_label = Label(video_app, text="Opencv Fps: 0")
opencv_fps_label.grid(row=2 , column=1)



button_frame = Frame(root, bg="white")
button_frame.config(width=416,height=416)
toggle_stop_start = Button(button_frame, text='Toggle Stop/Start')
toggle_arm_disarm = Button(button_frame, text='Toggle Arm/Disarm')

servo_open = Button(button_frame, text='Open Servo')
servo_close = Button(button_frame, text='Close Servo')

yolo_video_button = Button(button_frame, command=threading.Thread(target=video_main).start, text='Toggle YOLO Display')
opencv_video_button = Button(button_frame, text='Toggle Opencv Display')

toggle_attitude = Button(button_frame, text='Toggle Attitude Display')
button6 = Button(button_frame, text='button1')

## BUtton Display
button_frame.grid(row=3, column=0)



toggle_stop_start.grid()
toggle_arm_disarm.grid()

servo_open.grid()
servo_close.grid()

yolo_video_button.grid()
opencv_video_button.grid()

button6.grid()

### VEHICLE STATUS DISPLAY
status_frame = Frame(root)
status_frame.config( bg='gray')
status_frame.grid(row=3, column=1)

roll_label = Label(status_frame, text="Roll: N/A")
pitch_label = Label(status_frame, text="Pitch: N/A")
yaw_label = Label(status_frame, text="Yaw: N/A")
depth_label = Label(status_frame, text="Depth: N/A")

servo_label = Label(status_frame, text="Servo: N/A")
recognition_label = Label(status_frame, text="Object Recognition: N/A")



## STATUS DISPLAY
roll_label.grid()
pitch_label.grid()
yaw_label.grid()
depth_label.grid()


servo_label.grid()
recognition_label.grid()


