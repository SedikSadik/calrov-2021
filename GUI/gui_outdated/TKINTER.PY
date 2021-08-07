
from tkinter import *
import cv2
from PIL import Image, ImageTk
root = Tk()
root.title('CALROV ARAYÜZ AŞAMA 1')
root.geometry("640x480")
#root.iconphoto(False, PhotoImage(file="/home/violetcheese/Desktop/CALROV/GUI/calrov_icon2.ico"))
#root.iconbitmap("/home/violetcheese/Desktop/CALROV/GUI/calrov_icon2.ico",)
app = Frame(root, bg="white")
app.grid(row=0, column=0, columnspan=4)
lmain = Label(app)
lmain.grid()

cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    conlost = cv2.imread("/home/violetcheese/Desktop/CALROV/GUI/gui_images/todo.webp")
    cv2image = cv2.cvtColor(conlost, cv2.COLOR_BGR2RGB) #BGR RGB dönüşümü

    img = Image.fromarray(cv2image)

    imgtk = ImageTk.PhotoImage(image=img)
    lmain.imgtk = imgtk
    lmain.configure(image=imgtk)
#BUtton
exitbutton = Button(root, text='Exit', command=root.quit)
exitbutton.grid(row=2,column=0, ipadx=20,ipady=10, )
root.mainloop()
