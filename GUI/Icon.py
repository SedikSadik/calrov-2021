from tkinter import *       
from PIL import Image, ImageTk
import cv2


root=Tk()
root.title("Hey")

#Icon
icontmp = Image.open('/home/violetcheese/Desktop/CALROV/GUI/gui_images/calrov_logo.jpg')
icon = ImageTk.PhotoImage(icontmp)
root.tk.call('wm','iconphoto',root._w, icon)
# TEXT
l = Label(root, text = "Fact of the Day")
l.config(font =("Courier", 14))
l.grid()

root.mainloop()
