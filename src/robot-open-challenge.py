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
st = SerialTalk(Port.S4)
bw = Motor(Port.B) #foward and back
fw = Motor(Port.C) #left and right
sensor = ColorSensor(Port.S2) #color sensor
gyro = GyroSensor(Port.S3) #gyro sensor
# Lultra = UltrasonicSensor(Port.S2) #left ultrasonic sensor
# Rultra = UltrasonicSensor(Port.S4) #right ultrasonic sensor
# Infra = InfraredSensor(Port.S4)

ev3.speaker.beep()

direction = 0
max_turnR = 30   #500
max_turnL = -30 # -500
fw.reset_angle(0)
bw.reset_angle(0)
gyro.reset_angle(0)

fwspeed = 0
bwspeed = 0
prev_error = 0
error = 0
run = 0

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

# maxPower = 600 
# minPower = 500
#old pid sychronous bw speed

maxPower = 750
minPower = 600

# Write your program here.
ev3.speaker.beep()

def rgb_to_color(rgb):
    r, g, b = rgb

    # print("R:", r, "G:", g, "B:", b)

    if r < 40:
        # ev3.speaker.beep()
        return -1    # blue

    elif b < 70 and r > 40:
        # ev3.speaker.beep()
        return 1     # orange

    else:
        return 0     # white

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
    control_signal = -control_signal
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
    
    bw.run(-bwspeed)

    print(" fwspeed:", fwspeed ," bwspeed:", bwspeed ,"control_signal:", control_signal)


def mainThread():

    global color, direction

    while True:
        b = brick.buttons()
        if Button.RIGHT in b:
            ev3.speaker.beep()
            break
    

    while gyro.angle() <= 1060:
        color = rgb_to_color(sensor.rgb())
        print("gyro", gyro.angle())

        if color == 1 and (direction == 0 or direction == 1):
            direction = 1
            # run += 1
            ev3.speaker.beep()
            if fw.angle() > -30:
                bw.stop()
                fw.run_target(-1100, -30, wait=True)

        elif color == -1 and (direction == 0 or direction == -1):
            direction = -1
            # run += 1
            ev3.speaker.beep()
            if fw.angle() < 30:
                bw.stop()
                fw.run_target(1100, 30, wait=True)

        PID(0.01, 1.5, balance)
        # PID(0.005, 0.00085, balance)


    else:
        bw.reset_angle(0)
        while(bw.angle() > -400):
            # senseCam()
            # PID(0.01, 0.005, balance)
            PID(0.0125, 0.005, balance)

        
        bw.stop()
        fw.stop()


def intersectionCount():
    global run, direction
    while run <= 11:
        # print("Run:", run)
        color = rgb_to_color(sensor.rgb())
        if color == 1 and (direction == 0 or direction == 1):
            direction = 1
            run += 1
            # ev3.speaker.beep(100, 100)
            wait(500)
        elif color == -1 and (direction == 0 or direction == -1):
            direction = -1
            run += 1
            # ev3.speaker.beep(100, 100)
            wait(500)



def senseCam():
    start_yaw = None
    while True:
        status, data = st.call('cam')
        globals()['balance'], globals()['BRB'], globals()['distance'], globals()['IRDist'], bad_yaw, globals()['blockX'] = data
        
        # if start_yaw==None:
        #     start_yaw=bad_yaw
        # if bad_yaw-start_yaw > 0:
        #     final_yaw=bad_yaw - start_yaw
        # else:
        #     final_yaw= 360 + (bad_yaw - start_yaw)

        # if round(final_yaw)==360:
        #     final_yaw=0
        # globals()['yaw']=round(final_yaw)



threading.Thread(target=senseCam).start()
threading.Thread(target=intersectionCount).start()
mainThread()