import numpy as np
from sklearn.externals import joblib
from sklearn.svm import SVC
import serial
from Tkinter import *
import sys
import tkMessageBox
import os
import time

# Dependencies are automatically detected, but it might need fine tuning.
##connection with arduino over serial cable
ser = serial.Serial('/dev/ttyUSB4', 115200)

clf = SVC()

str = ""

#funtion to read serial cable data
def read_arduino(str):
	f1 = open("training_temp.txt","w+")
	f2 = open("label_temp.txt","w+")
	for i in range(10):
		while 1: 
			state = ser.readline()
			print(state)
			if state == "0\n":
				time.sleep(0.7)
				state = ser.readline()
				f1.write(state + '\n')
				f2.writelines(str + '\n')
				break
			else:
				pass
	w = Label(root, text= "10 gesture received")
	w.pack()
	f1.close()
	f2.close()

def printtext():
	global e
	string = e.get() 
	if len(string) == 0:
		tkMessageBox.showinfo("Warning", "Enter Proper String")
		return
	read_arduino(string)

##loading training data set and labels and save the hyphothesis funtion 
def _train():
	try:
		f = open("training_temp.txt","r")
		contents = f.read()
		fp = open("label_temp.txt","r")
		content = fp.read()
		with open('training.txt', 'a') as file:
			file.write(contents)
		with open('label.txt', 'a') as file:
			file.write(content)
	except:
		tkMessageBox.showinfo("Warning", "Enter String than train")
		return
	f.close()
	fp.close()
	try:
		X = np.loadtxt("training.txt",dtype = 'float')  
		Y = np.genfromtxt("label.txt",dtype='str',delimiter = "\n")
		clf.fit(X,Y)
		joblib.dump(clf, 'svm.pkl') 
		tkMessageBox.showinfo("Success", "Machine trained")
	except:
		tkMessageBox.showinfo("Failure", "Machine not trained")
		return

root = Tk()
try:
	os.remove("label_temp.txt")
	os.remove("training_temp.txt")
except:
	pass
w = Label(root, text="Step1..Enter string in box\nStep2..Click on 'enter string' pass 10 gesture values \nStep3..Click on 'Train'\nStep4..Click on 'Quit'")
w.pack()
e = Entry(root)
e.pack()
e.focus_set()
b = Button(root,text='Quit',command=sys.exit)
b.pack(side='bottom')
b = Button(root,text='Train',command=_train)
b.pack(side='bottom')
b = Button(root,text='Enter String',command=printtext)
b.pack(side='bottom')

root.mainloop()
