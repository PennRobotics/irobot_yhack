#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 12 November 2016

###########################################################################
# This code has been modified for the YHack competition.
#
# Original Notice
# Copyright (c) 2015 iRobot Corporation
# http://www.irobot.com/
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
#   Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
#
#   Neither the name of iRobot Corporation nor the names
#   of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written
#   permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###########################################################################

import numpy as np
import cv2
import Tkinter as tk
import tkMessageBox
import tkSimpleDialog
from PIL import Image, ImageTk
import video
import struct
import sys, glob # for listing serial ports

try:
    import serial
except ImportError:
    tkMessageBox.showerror('Import error', 'Please install pyserial.')
    raise

upper = np.array([255, 155, 155])
lower = np.array([200, 0, 0])

# static variables for keyboard callback -- I know, this is icky
callbackKeyUp = False
callbackKeyDown = False
callbackKeyLeft = False
callbackKeyRight = False
callbackKeyLastDriveCommand = ''

connection = None

TEXTWIDTH = 40 # window width, in characters
TEXTHEIGHT = 16 # window height, in lines

VELOCITYCHANGE = 200
ROTATIONCHANGE = 300

helpText = """\
Supported Keys:
P\tPassive
S\tSafe
F\tFull
C\tClean
D\tDock
R\tReset
Space\tBeep
Arrows\tMotion

If nothing happens after you connect, try pressing 'P' and then 'S' to get into safe mode.
"""
cap = cv2.VideoCapture(0)

def driveFeedback(motorR, motorL):
    cmd = struct.pack(">Bhh", 145, motorR, motorL)
    sendCommandRaw(cmd)
    pass

def driveForward():
    cmd = struct.pack(">Bhh", 145, 200, 200)
    sendCommandRaw(cmd)
    pass

def driveForwardLeft():
    cmd = struct.pack(">Bhh", 145, 350, 50)
    sendCommandRaw(cmd)
    pass

def driveForwardRight():
    cmd = struct.pack(">Bhh", 145, 50, 350)
    sendCommandRaw(cmd)
    pass

def driveLeft():
    cmd = struct.pack(">Bhh", 145, 60, -60)
    sendCommandRaw(cmd)
    pass

def driveRight():
    cmd = struct.pack(">Bhh", 145, -60, 60)
    sendCommandRaw(cmd)
    pass

def driveLost():
    cmd = struct.pack(">Bhh", 145, -30, 30)
    sendCommandRaw(cmd)
    pass

def driveStop():
    cmd = struct.pack(">Bhh", 145, 0, 0)
    sendCommandRaw(cmd)
    pass

def show_frame():
    # Poll webcam
    _, frame = cap.read()
    frame = cv2.flip(frame, 1)

    # Isolate largest blue region
    mask = cv2.inRange(frame, lower, upper)
    # Find contours
    (_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if cnts:
        c = max(cnts, key=cv2.contourArea)
        xMax = [x for x, y in np.amax(c, axis=0)]
        yMax = [y for x, y in np.amax(c, axis=0)]
        xMin = [x for x, y in np.amin(c, axis=0)]
        yMin = [y for x, y in np.amin(c, axis=0)]
        xAvg = (xMin[0] + xMax[0]) / 2
        yAvg = (yMin[0] + yMax[0]) / 2
        spotArea = cv2.contourArea(c)

        if 0:
            # Discrete, unfiltered motion
            if spotArea > 500 and spotArea < 2500:
                if xAvg < 100:
                    print('<<<<             ')
                    driveRight()
                elif xAvg < 180:
                    print('<<<< FORWARD     ')
                    driveForwardRight()
                elif xAvg < 640 - 180:
                    print('     FORWARD     ')
                    driveForward()
                elif xAvg < 640 - 100:
                    print('     FORWARD >>>>')
                    driveForwardLeft()
                else:
                    driveLeft()
                    print('             >>>>')
                # print(xAvg, yAvg, spotArea)
            elif spotArea >= 2500:
                if xAvg < 220:
                    print('<<<<             ')
                    driveRight()
                elif xAvg < 640 - 220:
                    print('     S T O P     ')
                    driveStop()
                else:
                    driveLeft()
                    print('             >>>>')
            else:
                print('? ? ? ? ? ? ? ? ?')
                driveLost()
        elif spotArea > 500:
            # Motors proportional to error
            motorFwd = max(0, min(300, 200000 / spotArea - 80))
            if xAvg < 180:
                motorTurn = -72 + 2 * xAvg / 5  # Positive is LEFT (ccw)
            elif xAvg > 640 - 180:
                motorTurn = 2 * (xAvg - 460) / 5
            else:
                motorTurn = 0
            motorRight = motorFwd + motorTurn
            motorLeft = motorFwd - motorTurn
            driveFeedback(motorRight, motorLeft)

        # Approximate contour
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.05 * peri, True)

        # Draw a red bounding box surrounding the blue region
        cv2.drawContours(frame, [approx], -1, (0, 0, 255), 4)

    # Display image
    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
    img = Image.fromarray(cv2image)
    imgtk = ImageTk.PhotoImage(image=img)
    display.imgtk = imgtk
    display.configure(image=imgtk)
    display.after(10, show_frame)


# sendCommandASCII takes a string of whitespace-separated, ASCII-encoded base 10 values to send
def sendCommandASCII(command):
    cmd = ""
    for v in command.split():
        cmd += chr(int(v))

    sendCommandRaw(cmd)

# sendCommandRaw takes a string interpreted as a byte array
def sendCommandRaw(command):
    global connection

    try:
        if connection is not None:
            connection.write(command)
        else:
            tkMessageBox.showerror('Not connected!', 'Not connected to a robot!')
            print "Not connected."
    except serial.SerialException:
        print "Lost connection"
        tkMessageBox.showinfo('Uh-oh', "Lost connection to the robot!")
        connection = None

    print ' '.join([ str(ord(c)) for c in command ])
    text.insert(tk.END, ' '.join([ str(ord(c)) for c in command ]))
    text.insert(tk.END, '\n')
    text.see(tk.END)

# getDecodedBytes returns a n-byte value decoded using a format string.
# Whether it blocks is based on how the connection was set up.
def getDecodedBytes(n, fmt):
    global connection

    try:
        return struct.unpack(fmt, connection.read(n))[0]
    except serial.SerialException:
        print "Lost connection"
        tkMessageBox.showinfo('Uh-oh', "Lost connection to the robot!")
        connection = None
        return None
    except struct.error:
        print "Got unexpected data from serial port."
        return None

# get8Unsigned returns an 8-bit unsigned value.
def get8Unsigned():
    return getDecodedBytes(1, "B")

# get8Signed returns an 8-bit signed value.
def get8Signed():
    return getDecodedBytes(1, "b")

# get16Unsigned returns a 16-bit unsigned value.
def get16Unsigned():
    return getDecodedBytes(2, ">H")

# get16Signed returns a 16-bit signed value.
def get16Signed():
    return getDecodedBytes(2, ">h")

# A handler for keyboard events. Feel free to add more!
def callbackKey(event):
    global callbackKeyUp
    global callbackKeyDown
    global callbackKeyLeft
    global callbackKeyRight
    global callbackKeyLastDriveCommand

    k = event.keysym.upper()
    motionChange = False

    if event.type == '2': # KeyPress; need to figure out how to get constant
        if k == 'P':   # Passive
            sendCommandASCII('128')
        elif k == 'S': # Safe
            sendCommandASCII('131')
        elif k == 'F': # Full
            sendCommandASCII('132')
        elif k == 'C': # Clean
            sendCommandASCII('135')
        elif k == 'D': # Dock
            sendCommandASCII('143')
        elif k == 'SPACE': # Beep
            sendCommandASCII('140 3 1 88 16 141 3')
        elif k == '1': # Beep
            sendCommandASCII('140 3 8 83 12 88 12 1 12 83 12 88 12 1 12 83 12 88 12 141 3')
        elif k == '2': # Beep
            sendCommandASCII('140 3 8 83 12 88 12 90 12 88 12 87 12 1 12 88 12 90 12 141 3')
        elif k == '3': # Beep
            sendCommandASCII('140 3 8 83 12 87 12 1 12 83 12 87 12 1 12 83 12 87 12 141 3')
        elif k == '4': # Beep
            sendCommandASCII('140 3 8 83 12 87 12 88 12 87 12 85 12 1 12 87 12 88 12 141 3')
        elif k == '5': # Beep
            sendCommandASCII('140 3 16 95 8 1 4 95 8 1 4 95 12 92 8 1 4 92 8 1 4 92 12 88 8 1 4 88 8 1 4 88 12 83 24 141 3')
        elif k == '6': # Beep
            sendCommandASCII('140 3 10 80 12 81 12 83 12 85 12 87 12 88 12 90 12 92 12 93 12 90 24 141 3')
        elif k == '7': # Beep
            sendCommandASCII('140 3 16 93 8 1 4 93 8 1 4 93 8 1 4 93 12 90 8 1 4 90 8 1 4 90 12 87 8 1 4 87 12 83 24 141 3')
        elif k == '8': # Beep
            sendCommandASCII('140 3 9 95 8 1 4 95 12 97 12 95 12 93 12 92 12 90 12 88 24 141 3')
        elif k == 'R': # Reset
            sendCommandASCII('7')
        elif k == 'UP':
            callbackKeyUp = True
            motionChange = True
        elif k == 'DOWN':
            callbackKeyDown = True
            motionChange = True
        elif k == 'LEFT':
            callbackKeyLeft = True
            motionChange = True
        elif k == 'RIGHT':
            callbackKeyRight = True
            motionChange = True
        else:
            print repr(k), "not handled"
    elif event.type == '3': # KeyRelease; need to figure out how to get constant
        if k == 'UP':
            callbackKeyUp = False
            motionChange = True
        elif k == 'DOWN':
            callbackKeyDown = False
            motionChange = True
        elif k == 'LEFT':
            callbackKeyLeft = False
            motionChange = True
        elif k == 'RIGHT':
            callbackKeyRight = False
            motionChange = True

    if motionChange == True:
        velocity = 0
        velocity += VELOCITYCHANGE if callbackKeyUp is True else 0
        velocity -= VELOCITYCHANGE if callbackKeyDown is True else 0
        rotation = 0
        rotation += ROTATIONCHANGE if callbackKeyLeft is True else 0
        rotation -= ROTATIONCHANGE if callbackKeyRight is True else 0

        # compute left and right wheel velocities
        vr = velocity + (rotation/2)
        vl = velocity - (rotation/2)

        # create drive command
        cmd = struct.pack(">Bhh", 145, vr, vl)
        if cmd != callbackKeyLastDriveCommand:
            sendCommandRaw(cmd)
            callbackKeyLastDriveCommand = cmd

def onConnect():
    global connection

    if connection is not None:
        tkMessageBox.showinfo('Oops', "You're already connected!")
        return

    try:
        ports = getSerialPorts()
        #port = tkSimpleDialog.askstring('Port?', 'Enter COM port to open.\nAvailable options:\n' + '\n'.join(ports))
        port = '/dev/ttyUSB2'
    except EnvironmentError:
        port = tkSimpleDialog.askstring('Port?', 'Enter COM port to open.')

    if port is not None:
        print "Trying " + str(port) + "... "
        try:
            connection = serial.Serial(port, baudrate=115200, timeout=1)
            print "Connected!"
            # tkMessageBox.showinfo('Connected', "Connection succeeded!")
        except:
            print "Failed."
            tkMessageBox.showinfo('Failed', "Sorry, couldn't connect to " + str(port))


def onHelp():
    tkMessageBox.showinfo('Help', helpText)

def onQuit():
    if tkMessageBox.askyesno('Really?', 'Are you sure you want to quit?'):
        destroy()

def getSerialPorts():
    """Lists serial ports
    From http://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python

    :raises EnvironmentError:
        On unsupported or unknown platforms
    :returns:
        A list of available serial ports
    """
    if sys.platform.startswith('win'):
        ports = ['COM' + str(i + 1) for i in range(256)]

    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this is to exclude your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')

    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')

    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        print(port)
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


app = tk.Tk()

app.title("iRobot Create 2 Tethered Drive")
app.option_add('*tearOff', tk.FALSE)

menubar = tk.Menu()
app.configure(menu=menubar)

createMenu = tk.Menu(menubar, tearoff=False)
menubar.add_cascade(label="Create", menu=createMenu)

createMenu.add_command(label="Connect", command=onConnect)
createMenu.add_command(label="Help", command=onHelp)
createMenu.add_command(label="Quit", command=onQuit)

text = tk.Text(app, height =TEXTHEIGHT, width = TEXTWIDTH, wrap = tk.WORD)
scroll = tk.Scrollbar(app, command=text.yview)
text.configure(yscrollcommand=scroll.set)
imageFrame = tk.Frame(app, width=600, height=500)
imageFrame.grid(row=0, column=0, padx=10, pady=2)
imageFrame.pack()  # TODO(brwr))
text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
scroll.pack(side=tk.RIGHT, fill=tk.Y)

text.insert(tk.END, helpText)

display = tk.Label(imageFrame)
display.grid(row=1, column=0, padx=10, pady=2)

show_frame()

onConnect()

sendCommandASCII('128 131')  # TODO(brwr)
sendCommandASCII('128 132')  # TODO(brwr)

app.bind("<Key>", callbackKey)
app.bind("<KeyRelease>", callbackKey)

app.mainloop()
