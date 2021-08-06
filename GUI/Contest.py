from tkinter import *
import tkinter as tk

root = Tk()
root.title('CALROV ARAYÜZ AŞAMA 1')
#root.geometry("640x480")
#root.iconphoto(False, PhotoImage(file="/home/violetcheese/Desktop/CALROV/GUI/calrov_icon2.ico"))
#root.iconbitmap("/home/violetcheese/Desktop/CALROV/GUI/calrov_icon2.ico")
app = Frame(root, bg="white")
#app.grid()
lmain = Label(app)
#lmain.grid()

latest_attitude = {}
## TEXT
T = Text(root, height = 5, width = 52)
l = Label(root, text = "Fact of the Day")
l.config(font =("Courier", 14))
l.grid()
b1 = Button(root, text = "Next", )
  
# Create an Exit button.
b2 = Button(root, text = "Exit",
            command = root.destroy)
l.pack()
T.pack()
b1.pack()
b2.pack()
Fact = 'HEHE'
# Insert The Fact.
T.insert(tk.END, Fact)
  
tk.mainloop()