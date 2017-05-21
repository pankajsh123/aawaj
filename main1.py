import Tkinter
import numpy as np
import pyttsx
import subprocess
from sklearn.externals import joblib
from sklearn.svm import SVC
import serial
import time
import pickle

ser = serial.Serial('/dev/ttyUSB4', 115200)

#applying svm algorithm to calculate mapping function
clf = joblib.load('svm.pkl')

#funtion to read serial cable data
def read_arduino():
	while 1: 
		state = ser.readline()
		if state == '0\n':
			time.sleep(0.700)
			state = ser.readline()
			with open('Z.txt', 'w') as file:
				file.write(state)
			break

class Reminder(object):
    def __init__(self, show_interval=3, hide_interval=6):
        self.hide_int = hide_interval  # In seconds
        self.show_int = show_interval  # In seconds
        self.root = Tkinter.Tk()
        Tkinter.Frame(self.root, width=250, height=100).pack()
        Tkinter.Label(self.root, text=p).place(x=30, y=30)
        self.root.after_idle(self.show)  # Schedules self.show() to be called when the mainloop starts

    def hide(self):
        self.root.destroy()  # Hide the window
        main()

    def show(self):
        self.root.deiconify() # Show the window
        self.root.after(1000 * self.show_int, self.hide)  # Schedule self.hide in show_int seconds

    def start(self):
        self.root.mainloop()

def main():
	global p
	p = ""
	read_arduino()
	Y = np.loadtxt("Z.txt",dtype = 'float')
	p = clf.predict([Y])
	engine = pyttsx.init()
	engine.say(p)
	engine.runAndWait()
	r = Reminder()
	r.start()

main()
    
