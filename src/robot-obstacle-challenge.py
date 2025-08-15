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
gyro = GyroSensor(Port.S4) #gyro sensor
# Lultra = UltrasonicSensor(Port.S2) #left ultrasonic sensor
# Rultra = UltrasonicSensor(Port.S4) #right ultrasonic sensor
# Infra = InfraredSensor(Port.S4)

ev3.speaker.beep()

direction = 0

fw.reset_angle(0)
bw.reset_angle(0)
gyro.reset_angle(0)

gyro_angle = 0

prev_error = 0
error = 0
run = 0

# sensedBlock = 1
turn = 0
prevBW = 0

blockColor = 0
blockColorStr = ""

# variables from camera
balance = 0 
BRB = 0 
distance =  0
MGbalance = 0   
blockX = 0
blueLineDist = 9999
IRDist = 9999

left = 0
right = 0
firstSense = True

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
    globals()['balance'], globals()['BRB'], globals()['distance'], globals()['blueLineDist'], globals()['MGbalance'], globals()['IRDist'], globals()['blockX'] = data

def PID(kp, kd, left, right):

    global prev_error, error
    

    # Calculate the error
    error = left - right

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

def PID(kp, kd, balance, minbwPower=250, maxbwPower=600, max_turnR=900, max_turnL=-900):

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
        
    # print("Error:", left-right, "control_signal:", control_signal, " fwspeed:", fwspeed)

    # Set motor speeds
    control_signal = int(control_signal)

    if control_signal > max_turnR:
        control_signal = max_turnR
    elif control_signal < max_turnL:
        control_signal = max_turnL
    
    fw.run_target(fwspeed, control_signal, wait=False)
    
    if control_signal > maxbwPower or control_signal < -maxbwPower:
        control_signal = maxbwPower
    
    bwspeed = maxbwPower - abs(control_signal)
    
    if bwspeed < minbwPower:
        bwspeed = minbwPower
        
    bw.run(bwspeed)

def PIDBlocks(kp, kd, BRB, minbwPower=250, maxbwPower=600, max_turnR=900, max_turnL=-900):

    global prev_error, error    

    # Calculate the error
    error = BRB

    # Calculate the derivative of the error
    d_error = error - prev_error

    # Calculate the control signal
    control_signal = kp * error + kd * d_error

    # Update previous error
    prev_error = error

    # Set motor speeds based on control signal
    fwspeed = control_signal
    
    # Limit motor speeds to max values
    fwspeed = max(-1000, min(1000, fwspeed))
    # bwspeed = max(-1000, min(1000, bwspeed))

    if fwspeed > 0:
        fwspeed = 1000
    elif fwspeed < 0:
        fwspeed = -1000
    else:
        fwspeed = 1
    
    
    # print("BRB:", BRB, "distance:", distance)

    # Set motor speeds
    control_signal = int(control_signal)
    # print("distance", distance, "control_signal", control_signal, "BRB", BRB)

    if control_signal > max_turnR:
        control_signal = max_turnR
    elif control_signal < max_turnL:
        control_signal = max_turnL

    fw.run_target(fwspeed, control_signal, wait=False)
    
    if control_signal > maxbwPower or control_signal < -maxbwPower:
        control_signal = maxbwPower
    
    bwspeed = maxbwPower - abs(control_signal)
    
    if bwspeed < minbwPower:
        bwspeed = minbwPower
        
    bw.run(bwspeed)

def backToZeroSteering(bwPower, fwPower, steering=0, senseColor=True):
    bw.run(bwPower)
    colorPriority = 0
    while fw.angle() >= steering:  #steering facing to the right
        # ev3.speaker.beep()
        color = rgb_to_color(sensor.rgb())
        if color != 0 and senseColor:
            bw.stop()
            # ev3.speaker.beep(100, 100)
            if color > 0:
                colorPriority = 1
                orangeCornerMovements()
            else:
                colorPriority = -1
                # ev3.speaker.beep(100, 100)
                crossedBlue = True
            
            break
        fw.run(-fwPower)
    else:
        while fw.angle() <= steering:   #steering facing to the left
            # ev3.speaker.beep()
            color = rgb_to_color(sensor.rgb())
            if color != 0 and senseColor:
                bw.stop()
                if color > 0: # orange
                    colorPriority = 1
                    # orangeCornerMovements()
                else: # blue
                    colorPriority = -1                    
                    ev3.speaker.beep(100, 100)
                    crossedBlue = True
                
                
                break
            fw.run(fwPower)

    fw.stop()
    # bw.run(300)
    ev3.speaker.beep()  

    return colorPriority

def orangeCornerMovements(): 
    global direction, BRB
    
    if direction == 0 or direction == 1:  # no direction or clockwise and no block detected
        ev3.speaker.beep() 
        backToZeroSteering(0, 1100, steering=1100, senseColor=False)
        wait(50)

        bw.run_angle(500, 250)  # move forward a bit to avoid the corner
            
        bw.reset_angle(0)
        while distance > 500 and bw.angle() <= 450:
            ev3.speaker.beep() 
            senseCam()                
            bw.run(1100)
        else:
            bw.stop()

        # bw.run_angle(1100, 600)
        # ev3.speaker.beep()
        backToZeroSteering(0, 1100, steering=-1100, senseColor=False)  
        # wait(50)

        while distance > 500:  # find the block
            ev3.speaker.beep()
            senseCam()                
            bw.run(-500) 
        else:
            while blockX >= 30:  # different for blue corner movement
                ev3.speaker.beep()
                senseCam()
                bw.run(-300)
            # while BRB > 20 or BRB <-20 :
            #     ev3.speaker.beep()
            #     senseCam()
            #     bw.run(-300)
             
               
        backToZeroSteering(0, 1100, steering=0, senseColor=False)
        bw.run_angle(500, 200)

        # bw.run_angle(-400, 500)
        senseCam()
        if distance <= 125:            
            bw.run_angle(-500, 300)
        

def blueCornerMovements(): 
    global direction, BRB
    if direction == 0 or direction == -1:  # no direction or clockwise and no block detected
        ev3.speaker.beep() 
        backToZeroSteering(0, 1100, steering=-1100, senseColor=False)
        wait(50)

        bw.run_angle(500, 250)  # move forward a bit to avoid the corner
            
        bw.reset_angle(0)
        while distance > 500 and bw.angle() <= 450:
            senseCam()                
            bw.run(1100)
        else:
            bw.stop()

        # bw.run_angle(1100, 600)
        # ev3.speaker.beep()
        # wait(50)
        backToZeroSteering(0, 1100, steering=1000, senseColor=False)  
        
        while distance > 500:  # find the block
            ev3.speaker.beep()
            senseCam()                
            bw.run(-700) 
        else:
            while blockX >= 40:  # different for blue corner movement
                ev3.speaker.beep()
                senseCam()
                bw.run(-300)
            
               
        backToZeroSteering(0, 1100, steering=0, senseColor=False)

        # bw.run_angle(-400, 500)
        senseCam()
        if distance <= 100:            
            bw.run_angle(-500, 400)

def mainThread():

    global color, direction


    while True:
        b = brick.buttons() 
        if Button.RIGHT in b:
            ev3.speaker.beep()
            break
    
    # while True:
    
    #     print("gyro_angle", gyro_angle)
    #     gyro_angle = gyro.angle()

    # while True:
    #     senseCam()
    #     print("blockX", blockX)

    senseCam()
        
    if balance > 0.0:  # clockwise movement (turn right from parking)
        backToZeroSteering(0, 1100, steering=950, senseColor=True)
        bw.run_angle(400, 150)
        backToZeroSteering(0, 1100, steering=-950, senseColor=True)
        bw.run_angle(400, -120)
        backToZeroSteering(0, 1100, steering=950, senseColor=True)
        bw.run_angle(400, 150)    
        backToZeroSteering(0, 1100, steering=0, senseColor=True)
        bw.run_angle(400, 100)
    elif balance < 0.0:  # counter-clockwise movement (turn left from parking)
        backToZeroSteering(0, 1100, steering=-950, senseColor=True)
        bw.run_angle(400, 150)
        backToZeroSteering(0, 1100, steering=950, senseColor=True)
        bw.run_angle(400, -120)
        backToZeroSteering(0, 1100, steering=-1000, senseColor=True)
        bw.run_angle(500, 600)    
        backToZeroSteering(0, 1100, steering=0, senseColor=True)
        bw.run_angle(500, 600)
        backToZeroSteering(0, 1100, steering=-950, senseColor=True)
        bw.run_angle(500, -600)
        backToZeroSteering(0, 1100, steering=-0, senseColor=True)



    wait(5000)

    
    while run <= 11:

        global blueLineDist, firstSense, corner
            
        senseCam()
        color = rgb_to_color(sensor.rgb())
        # gyro = gyro.angle()

        print("direction:", direction)
        print("run", run)
        # print("IRDist", IRDist)


        # MODE: ==================================================== BLOCK AVOIDANCE
        # elif BRB != 0.0 and distance <= 155 and (blueLineDist > distance or blueLineDist <= 50): 
        # elif distance <= 155 and (blueLineDist > distance or blueLineDist <= 50):
        if distance <= 155: 

            if firstSense:
                firstSense = False
                # print("blockX:", blockX) 

                backToZeroSteering(0, 1100, steering=0.0, senseColor=True)                

                # if blockX > 0 and distance > 110:
                #     senseCam()                    
                #     backToZeroSteering(100, 1100, steering=500, senseColor=True)
                # elif blockX < 0 and distance > 110:
                #     senseCam()                    
                #     backToZeroSteering(100, 1100, steering=-500, senseColor=True)

                bw.reset_angle(0)
                while (blockX < 80 or blockX >= -80) and distance < 40 and bw.angle() >= -500 :
                    wait(50)
                    senseCam()                    
                    bw.run(-500)

                # while blockX >= 30 :  # steer to the left
                #     ev3.speaker.beep(100,100)
                #     senseCam()                    
                #     bw.run(400)

                # while blockX <= -30 :  # steer to the right
                #     ev3.speaker.beep(100,100)
                #     senseCam()                    
                #     bw.run(400)

                
            
            # ev3.speaker.beep(100, 100) kp 9.5 kd 4 (kp 7.5 kd 3.5 smooth)
            PIDBlocks(7.5, 3.5, BRB, minbwPower=350, maxbwPower=500, max_turnR=900, max_turnL=-900)

            # if IRDist <= 100:
            #     backToZeroSteering(0, 1100, steering=0, senseColor=True)

            #     bw.reset_angle(0)
            #     while bw.angle() >= -400:
            #         senseCam()
            #         bw.run(-400)
            #     # ev3.speaker.beep()
            #     bw.stop()
            #     wait(50)
            


        # MODE: ==================================================== ORANGE INTERSECTION (CLOCKWISE)
        elif color == 1 and (direction == 0 or direction == 1) and blockX >= 120:
            firstSense = True
            orangeCornerMovements()
            print("distance:", distance)

        # MODE: ==================================================== BLUE INTERSECTION (COUNTER-CLOCKWISE)
        elif color == -1 and (direction == 0 or direction == -1) and blockX <= -120:
            firstSense = True
            blueCornerMovements()
            
    

        # MODE: ==================================================== TOO CLOSE TO A BLOCK
        elif IRDist <= 100:
            firstSense = True
            backToZeroSteering(0, 1100, steering=0, senseColor=False)   
            
            bw.reset_angle(0)
            while bw.angle() >= -500:
                senseCam()
                bw.run(-400)
            # ev3.speaker.beep()
            bw.stop()
            wait(50)
            
  
            # print("IRDist", IRDist)


        # MODE: ==================================================== MAGENTA AVOIDANCE COUNTER-CLOCKWISE
        elif MGbalance >= 450:
            bw.stop()
            # wait(5000)
            if direction == 1:
                fw_speed = 1100
            else:
                fw_speed = -1100  

            while True:
                senseCam()

                if MGbalance == 0:
                    break

                fw.run(fw_speed)
                bw.run(100)   

            fw.stop()
            backToZeroSteering(0, 1100, steering=0, senseColor=False)
          

        # MODE: ==================================================== PID WHITE BALANCE
        elif distance > 155 and rgb_to_color(sensor.rgb()) == 0 :
            firstSense = True
            
            # ev3.speaker.beep() #kp 0.3 kd 0.1
            PID(0.25, 0.2, balance, minbwPower=400, maxbwPower=500, max_turnR=700, max_turnL=-700)

            # PID(0.35, 0.1, balance, minbwPower=250, maxbwPower=600, max_turnR=700, max_turnL=-700)

    
    # else:  # parking
            
    else:
        bw.reset_angle(0)

        while(bw.angle() < 1300):
            senseCam()

            if distance <= 155: 
                if firstSense:
                    firstSense = False
                    backToZeroSteering(0, 1100, steering=0.0, senseColor=True)                

                    bw.reset_angle(0)
                    while (blockX < 80 or blockX >= -80) and distance < 40 and bw.angle() >= -500 :
                        wait(50)
                        senseCam()                    
                        bw.run(-500)

                PIDBlocks(7.5, 3.5, BRB, minbwPower=350, maxbwPower=500, max_turnR=900, max_turnL=-900)

            elif distance > 155 and rgb_to_color(sensor.rgb()) == 0 :
                firstSense = True
                
                # ev3.speaker.beep() #kp 0.3 kd 0.1
                PID(0.28, 0.2, balance, minbwPower=400, maxbwPower=500, max_turnR=700, max_turnL=-700)

            
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
            wait(2000)
        elif color == -1 and (direction == 0 or direction == -1):
            direction = -1
            run += 1
            wait(2000)




threading.Thread(target=intersectionCount).start()
mainThread()