#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor,ColorSensor,
                                 InfraredSensor, GyroSensor)
from pybricks.nxtdevices import UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from serialtalk.auto import SerialTalk

import threading


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
st = SerialTalk(Port.S1)
bw = Motor(Port.B) #foward and back
fw = Motor(Port.C) #left and right
sensor = ColorSensor(Port.S3) #color sensor
# Lultra = UltrasonicSensor(Port.S2) #left ultrasonic sensor
# Rultra = UltrasonicSensor(Port.S4) #right ultrasonic sensor
# Infra = InfraredSensor(Port.S4)

ev3.speaker.beep()

direction = 0
max_turnR = 500   #1274
max_turnL = -500 # -2795
fw.reset_angle(0)
bw.reset_angle(0)
fwspeed = 0
bwspeed = 0
prev_error = 0
error = 0
prev_error_FB = 0
run = 0
LeftWins = True
sensedBlock = 1
turn = 0
prevBW = 0
lap_count = 0
blockColor = 0
blockColorStr = ""
firstSense = True

prev_distance = 0
prev_Bpik = 0
offTheWall = 1
colorPriority = 0
crossedBlue = True
MGpass = 0

# variables from camera
balance = 0 
BRB = 0 
distance =  0
MGbalance = 0 
pik = 0 
Bpik = 0  
blockX = 0
blueLineDist = 9999
IRDist = 0

left = 0
right = 0

maxPower = 600
minPower = 500


# Write your program here.
ev3.speaker.beep()

def rgb_to_color(rgb):
    r, g, b = rgb
    if r <= 20:
        # ev3.speaker.beep()
        return -1    # blue

    elif b <= 10:
        # ev3.speaker.beep()
        return 1     # orange

    else:
        return 0     # white

def senseCam():
    status, data = st.call('cam')
    globals()['balance'], globals()['BRB'], globals()['distance'], globals()['blueLineDist'], globals()['MGbalance'], globals()['IRDist'] = data

def PID(kp, kd, balance):

    global prev_error, error
    

    # Calculate the error
    error = balance

    # Calculate the derivative of the error
    d_error = error - prev_error

    # Calculate the control signal
    control_signal = kp * error + kd * d_error

    # Update previous error
    prev_error = error

    # Set motor speeds based on control signal
    fwspeed = control_signal
    # bwspeed = 100 - control_signal

    # Limit motor speeds to max values
    fwspeed = max(-1000, min(1000, fwspeed))
    # bwspeed = max(-1000, min(1000, bwspeed))

    if fwspeed > 0:
        fwspeed = 1000
    elif fwspeed < 0:
        fwspeed = -1000
    else:
        fwspeed = 1
    
    # if control_signal == 0:
    #     control_signal = 1

    # print("Error:", left-right, "control_signal:", control_signal, " fwspeed:", fwspeed)

    # Set motor speeds
    control_signal = int(control_signal)

    if control_signal > max_turnR:
        control_signal = max_turnR
    elif control_signal < max_turnL:
        control_signal = max_turnL
    
    fw.run_target(fwspeed, control_signal, wait=False)

    if control_signal > maxPower or control_signal < -maxPower:
        control_signal = maxPower
    
    bwspeed = maxPower - abs(control_signal)
    
    if bwspeed < minPower:
        bwspeed = minPower
    
    bw.run(bwspeed)
    print(" bwspeed:", bwspeed ,"control_signal:", control_signal)


def mainThread():

    global color, direction

    while True:
        b = brick.buttons()
        if Button.RIGHT in b:
            ev3.speaker.beep()
            break
        
    while run <= 11:

        senseCam()
        color = rgb_to_color(sensor.rgb())

        if color == 1 and (direction == 0 or direction == 1):
            direction = 1
            # run += 1
            ev3.speaker.beep()
            if fw.angle() < 700:
                ev3.speaker.beep(100, 100)
                bw.stop()
                fw.run_target(1100, 700, wait=True)

        elif color == -1 and (direction == 0 or direction == -1):
            direction = -1
            # run += 1
            ev3.speaker.beep()
            if fw.angle() > -700:
                ev3.speaker.beep(100, 100)
                bw.stop()
                fw.run_target(-1100, -700, wait=True)
        
        # PID(0.35, 0.075, balance)
        PID(0.15, 2.5, balance)


    else:
        bw.reset_angle(0)

        while(bw.angle() < 1800):
            senseCam()
            PID(0.15, 2.5, balance)
        
        bw.stop()
        fw.stop()

def intersectionCount():
    global run
    while run <= 11:
        # print("Run:", run)
        color = rgb_to_color(sensor.rgb())
        if color == 1:
            run += 1
            wait(2000)




threading.Thread(target=intersectionCount).start()
mainThread()