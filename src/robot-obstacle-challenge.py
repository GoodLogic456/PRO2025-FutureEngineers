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

#####left power + right power -
#####gyro left - right +

# Create your objects here.
ev3 = EV3Brick()
st = SerialTalk(Port.S4)
bw = Motor(Port.B) #foward and back
fw = Motor(Port.C) #left and right
sensor = ColorSensor(Port.S2) #color sensor
# gyro = GyroSensor(Port.S3) #gyro sensor
timer = StopWatch()
ev3.speaker.beep()

direction = 0

fw.reset_angle(0)
bw.reset_angle(0)
# gyro.reset_angle(0)

gyro_angle = 0

prev_error = 0
error = 0
sum_error = 0
run = 0
final_error = 0
Ferror = 0

cornergreen = False
predistance = 999
greenmove = False
color = 0
target = 0
section = 0
angle = 0

arc1 = 0
arc2 = 0
x1 = 0
x2 = 0
target_yaw = 0



# variables from camera
balance = 0 
BRB = 0 
distance =  999
yaw = True
blockX = 0
IRDist = 9999

lane = 1
yaw = 0

firstBlockPerSection = [0, 0, 0, 0]
firstBlock = False


# Write your program here.
# ev3.speaker.beep()

def rgb_to_color(rgb):
    r, g, b = rgb

    # print("R:", r, "G:", g, "B:", b)

    if r < 40:
        # ev3.speaker.beep()
        return -1    # blue

    elif b < 70 and r > 55:
        # ev3.speaker.beep()
        return 1     # orange

    else:
        return 0     # white

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

def steerToAngle(bwPower, fwPower, time=1500, steering=0):
    bw.run(bwPower)
    timer.reset()

    if fw.angle() < steering:
        while fw.angle() <= steering:  #steering facing to the left 
            # print("fw_angle()", fw.angle())       
            fw.run(fwPower)
    else:
        while fw.angle() >= steering :   #steering facing to the right
            # print("fw_angle()", fw.angle())
            fw.run(-fwPower)

    fw.stop()
    fw.brake()
    ev3.speaker.beep()  

def shiftlane():
    steerToAngle(0, 300, steering=-26)
    while gyro.angle() <= 65:
        bwspeed = (65 - gyro.angle())/65 * 800 
        if bwspeed < 200:
            bwspeed = 200
        print(gyro.angle())
        bw.run(-bwspeed)

    
    bw.stop()    
    # steerToAngle(0, 300, steering=0)
    
    # bw.run_angle(-600, 125)
    steerToAngle(0, 300, steering=40)

    while gyro.angle() >= 0:
        if gyro.angle() > 0:
            bwspeed = (gyro.angle() - 0)/65 * 800

        if bwspeed < 200:
            bwspeed = 200
        print(gyro.angle())
        bw.run(-bwspeed)
    
    steerToAngle(0, 300, steering=0)
    
    bw.stop()

def calc_turn_error(yaw,target_yaw):
    global final_error

    #gyro to yaw conversion
    if target_yaw < 0:
        target_yaw = target_yaw + 360
    elif target_yaw > 359:
        target_yaw = target_yaw - 360

    arc1=abs(yaw - target_yaw)
    arc2 = 360 - arc1
    if arc1 < arc2 :
        error = arc1
    else:
        error = arc2
    x1= yaw + error
    x2= yaw - error

    if x1 > 359:
        x1 = x1 - 360
    elif x1 < 0:
        x1= x1 + 360
    if x2 > 359:
        x2 = x2 - 360
    elif x2 < 0:
        x2= x2 + 360
    if x1 == target_yaw:
        final_error = (error)
    elif x2 == target_yaw:
        final_error= -error 
    return final_error

def sign(error):
    return (error > 0) - (error < 0)

def gyroPID(kp, kd, target_yaw, ki=0, minPower=200, maxPower=700, resetKI=True):
    
    global prev_error, error, yaw, section, sum_error

    if resetKI:
        sum_error = 0

    if minPower > 0 or maxPower > 0:
        error = calc_turn_error(yaw, target_yaw)
    else:
        error = -(calc_turn_error(yaw, target_yaw)) 

    # Calculate the error
    # if minPower > 0 or maxPower > 0:
    #     error = yaw - target
    # else:
    #     error = target -yaw

    # Calculate the derivative of the error
    d_error = error - prev_error

    sum_error += error

    # Calculate the control signal
    control_signal = kp * error + kd * d_error + ki * sum_error

    # control_signal = control_signal

    # Update previous error
    prev_error = error

    if control_signal > 0:
        fwspeed = -abs((maxPower - (abs(control_signal) * 20)))
    elif control_signal < 0:
        fwspeed = abs((maxPower - (abs(control_signal) * 20)))
    else:
        fwspeed = 1
    
    if int(fwspeed) == 0:
        fwspeed = 1

            # Set motor speeds

    control_signal = int(-(control_signal))
    fwspeed = int(fwspeed)
    fw.run_target(fwspeed, control_signal, wait=False)


    if minPower > 0 or maxPower > 0:
        if control_signal > maxPower or control_signal < -maxPower:
            control_signal = maxPower

    else:
        if control_signal < maxPower or control_signal > -maxPower:
            control_signal = maxPower

    if minPower > 0 or maxPower > 0:
        bwspeed = maxPower - (abs(control_signal) * 20) 

        if bwspeed < minPower:
            bwspeed = minPower
    else:
        bwspeed = maxPower + (abs(control_signal) * 20) 

        if bwspeed > minPower:
            bwspeed = minPower

    if bwspeed > 0:
        bw.run(-700)
    else:
        bw.run(700)
    
    # bw.run(-bwspeed)

    # print("yaw", yaw, "error:", error, "control_signal:", control_signal, "fwspeed:", fwspeed,"bwspeed:", bwspeed)

def gyroDistance(minPower,maxPower,target_yaw,distance):
    bw.reset_angle(0)

    while abs(bw.angle()) < abs(distance):
        gyroPID(1.75, 0.0001, target_yaw=target_yaw, ki=0.0035, minPower=minPower, maxPower=maxPower, resetKI=False)

    bw.brake()

def gyroTime(minPower,maxPower,target_yaw,time):

    timer.reset()
    while timer.time() < time:
        gyroPID(1.75, 0.0025, target_yaw=target_yaw, minPower=minPower, maxPower=maxPower)

    bw.stop()

def gyroIRDist(minPower,maxPower,target_yaw,uldist):
    global IRDist

    while IRDist > uldist:
        gyroPID(1.75, 0.0025, target_yaw=target_yaw, minPower=minPower, maxPower=maxPower)
    bw.brake()

def gyroTurn(minpower, maxpower, steer, angle): # Ferror is the first value is not changing 

    global yaw
    begin_angle = yaw
    Ferror = calc_turn_error(begin_angle, (target_yaw + angle))
    error = calc_turn_error(yaw, (target_yaw + angle))
    steerToAngle(0, 300, steering = steer)
    
    while sign(error) == sign(Ferror) and sign(Ferror)!=0:
        error = calc_turn_error(yaw, (target_yaw + angle))

        # print("error",error,"sign",sign(error))
        bwspeed = int(((maxpower - minpower ) * (abs(error)/abs(Ferror)) + minpower))
        bw.run(-bwspeed)

    bw.stop()
    steerToAngle(0, 300, steering = 0)

def corner():  # default laneOut is right lane
    global lane, target_yaw, section, cornergreen, sum_error
    
    sum_error = 0
    
    bw.hold()

    steerToAngle(0, 300, steering = 0)


    if lane == -1:  # left lane

        gyroDistance(-200,-700,target_yaw,1500)
        steerToAngle(0, 300, steering = 0)
        gyroTurn(50, 900, -40, 90)
        steerToAngle(0, 300, steering = 0)
        # gyroTime(-200,-600,(target + 90),1200)
        

        lane = 1


    elif lane == 1:  # right lane 
        
        if cornergreen == False:
            gyroDistance(-200,-700,target_yaw,1450)
            steerToAngle(0, 300, steering = 0)
            gyroTurn(50, 900, -40, 85)
            steerToAngle(0, 300, steering = 0)

            lane = 1

        elif cornergreen == True:
            gyroDistance(-200,-700,target_yaw,450)
            steerToAngle(0, 300, steering = 0)
            gyroTurn(50, 900, -40, 85)
            steerToAngle(0, 300, steering = 0)

            lane = -1

    section = section + 1
    cornergreen = False

    gyroPID(1.95, 0.0025, target_yaw=target_yaw, minPower=-200, maxPower=-700)        
   

def corner(laneOut=1):  # default laneOut is right lane
    global lane, target_yaw, section, cornergreen, sum_error
    
    sum_error = 0
    
    bw.hold()

    steerToAngle(0, 300, steering = 0)


    if lane == -1:  # left lane

        if laneOut == 1:
            gyroDistance(-200,-700,target_yaw,1200)
            lane = 1
        elif laneOut == -1:
            gyroDistance(-200,-700,target_yaw,300)
            lane = -1
        
        steerToAngle(0, 300, steering = 0)
        gyroTurn(150, 900, -40, 90)
        steerToAngle(0, 300, steering = 0)
        # gyroTime(-200,-600,(target + 90),1200)
        
        # lane = 1


    elif lane == 1:  # right lane 
        
        if cornergreen == False:

            if laneOut == 1:
                gyroDistance(-200,-700,target_yaw,1200)
                lane = 1
            elif laneOut == -1:
                gyroDistance(-200,-700,target_yaw,300)
                lane = -1
            
            steerToAngle(0, 300, steering = 0)
            gyroTurn(150, 900, -40, 85)
            steerToAngle(0, 300, steering = 0)

            # lane = 1

        elif cornergreen == True:
            if laneOut == 1:
                gyroDistance(-200,-700,target_yaw,1200)
                lane = 1
            elif laneOut == -1:
                gyroDistance(-200,-700,target_yaw,300)
                lane = -1
            
            steerToAngle(0, 300, steering = 0)
            gyroTurn(150, 900, -40, 85)
            steerToAngle(0, 300, steering = 0)

            lane = -1

    section = section + 1
    cornergreen = False

    # gyroPID(1.95, 0.0025, target_yaw=target_yaw, minPower=-200, maxPower=-700)


def shiftlane2():
    ###########For color is red 50
    global lane, greenmove

    ev3.speaker.beep(100)

    if BRB == 1 :  #red
        if lane != 1 :
            # print("target",target)
            gyroTurn(150, 900, -40, angle = 85)

            gyroDistance(200,800,target_yaw=(target_yaw + 85),distance=150)

            gyroTurn(150, 700, 35, angle = 0)

            steerToAngle(0, 300, steering = 0)
            lane = 1
        else:
            gyroPID(1.75, 0.0025, target_yaw=target_yaw, minPower=200, maxPower=700)   

        storeFirstBlockPerSection(1)     


    elif BRB == -1: 
        if lane != -1:
            steerToAngle(0, 300, steering = 0)

            greenmove = True
            gyroTurn(150, 900, 35, angle= (-85))#(-(target + 90)))

            steerToAngle(0, 300, steering = 0)
            # bw.run_time(-600, 1500,wait=True)
            gyroIRDist(150, 500, target_yaw=(target_yaw -85), uldist=200)
   
            steerToAngle(0, 300, steering = 0)

            gyroDistance(-200,-800,target_yaw=(target_yaw - 85),distance=355)


            # bw.run_angle(600, 600, wait=True)

            gyroTurn(150, 800, -40, 0)

            steerToAngle(0, 300, steering = 0)
            greenmove = False
            lane = -1
    

        storeFirstBlockPerSection(-1) 
        gyroPID(1.75, 0.025, target_yaw=target_yaw, minPower=200, maxPower=700)       



def storeFirstBlockPerSection(color):

    global firstBlock, firstBlockPerSection, section

    if firstBlock and firstBlockPerSection[abs(section) % 4] == 0:
        # print("abs(section) % 4", abs(section) % 4)
        firstBlockPerSection[abs(section) % 4] = color  # store first block
        firstBlock = False


def mainThread():
# while True:
    #     global color, direction, lane , predistance , cornergreen, target, greenmove, section

    #     color = rgb_to_color(sensor.rgb())

    #     while calc_turn_error() <= 90:

    #         bw.run(-200)
    #         print(calc_turn_error())

    #     steerToAngle(0, 250, steering = 0)

    #     bw.stop()
    #     wait(2000)



        

        # print("yaw",yaw,"target_yaw",target_yaw)

        # if distance <= 205: 
        #     if lane == 1 and blockX < 0:  #red
        #         shiftlane2()
        #     elif lane == -1:  #blue
        #         shiftlane2()

        #     elif lane == 1 and blockX > 0 and BRB == -1:
        #         cornergreen = True
        #         gyroPID(1.68, 0.018, target=target, minPower=200, maxPower=700)

        # else:
        #     gyroPID(1.68, 20, target=target, minPower=200, maxPower=700)        





    # while True:
    #     steerToAngle(0, 250, steering = 32)
    #     print("yaw",yaw,"target_yaw",target_yaw)
    #     while yaw > 270:
    #         print("yaw",yaw,"target_yaw",target_yaw)
    #         bw.run(-200)
        
    #     bw.stop()
    #     wait(10000)

        
    # while True:
    #     global color, direction, lane , predistance , cornergreen, target, greenmove, section

    #     color = rgb_to_color(sensor.rgb())

    #     print("target_yaw",target_yaw,"yaw",yaw)

    #     make_target()
    #     if section < 4:
    #         if direction == 0:
    #             target = (section % 4) * 90 * -1
    #         else:
    #             target = (section % 4) * 90 * direction
    #     else:
    #         section = 0
    #         if direction == 0:
    #             target = (section % 4) * 90 * 1
    #         else:
    #             target = (section % 4) * 90 * direction

    #     if color == -1:
    #         section = section + 1
    #         steerToAngle(0, 250, steering = -32)
    #         print(target_yaw)

    #         if section != 4:
    #             while yaw < (target_yaw - 90):
    #                 bw.run(-200)
    #         else:
    #             while yaw < 358 and yaw > 5:
    #                 bw.run(-200)

    #         bw.stop()

    #     else:
    #         gyroPID(1.68, 20, target=target, minPower=200, maxPower=700)      
  
    #  while True:
    #     global color, direction, lane , predistance , cornergreen, target_yaw, greenmove, section

    #     color = rgb_to_color(sensor.rgb())
        
    #     #color 1 sec + 1 color -1 sec -1
    #     if color == 1:
    #         section = section + 1
    #         gyroTurn(300, 900, -32, angle= 90)
    #         ev3.speaker.beep()
    #     else:
    #         gyroPID(1.75, 0.0025, target_yaw=target_yaw, minPower=200, maxPower=700)


    #     if direction == 0:
    #         target_yaw = (section % 4) * 90 * 1
    #     else:
    #         target_yaw = (section % 4) * 90 * direction

        


        # print(target_yaw)
###


    while True:

        global color, direction, lane , predistance , cornergreen, target_yaw, greenmove, section, firstBlock

        color = rgb_to_color(sensor.rgb())

        if direction == 0:
            target_yaw = (section % 4) * 90 * 1
        else:
            target_yaw = (section % 4) * 90 * direction

        # print("cornergreen:", cornergreen, "IRDist:", IRDist,"greenmove:", greenmove,"target",target,"gyro:",gyro.angle())

        # senseCam()
        if IRDist < 150 and greenmove == False:  # corner
            firstBlock = True
            # corner()  
            nextSection = 0
            if (abs(section) % 4) + 1 == 4:
                nextSection = 0
            else:
                nextSection = (abs(section) % 4) + 1

            if firstBlockPerSection[nextSection] != 0: # first block of next section is already saved

                if firstBlockPerSection[nextSection] == -1:  # first block of next section is green
                    corner(laneOut=-1)   
                elif firstBlockPerSection[nextSection] == 1:  # first block of next section is red
                    corner(laneOut=1)

            else:  # no stored color yet
                corner(laneOut=1)


        elif distance == 999 and predistance < 999:
            passBlock = True
            predistance = 999

        elif distance < 160 and distance != 999: # block detected is too near
            if BRB == 1 and lane != 1 :  #red                

                while distance <= 190 and distance != 999:
                    gyroPID(1.75, 0.0025, target_yaw=target_yaw, minPower=-200, maxPower=-700)        
                    
                bw.stop()     

                storeFirstBlockPerSection(1) 

            elif BRB == -1 and lane != -1 and blockX < 0:  #green               

                while distance <= 190 and distance != 999:
                    gyroPID(1.75, 0.0025, target_yaw=target_yaw, minPower=-200, maxPower=-700)        
                    
                bw.stop()

                storeFirstBlockPerSection(-1)

            else:
                gyroPID(1.75, 0.0025, target_yaw=target_yaw, minPower=200, maxPower=700)        
        
        elif distance <= 205:  # block detected
            predistance = distance
            if lane == 1 and blockX < 0:  #red                
                
                shiftlane2()

                # storeFirstBlockPerSection(1)

            elif lane == -1:  #green               

                shiftlane2()
                # storeFirstBlockPerSection(-1)

            elif lane == 1 and blockX > 0 and BRB == -1:  # green              

                storeFirstBlockPerSection(-1)

                cornergreen = True
                gyroPID(1.75, 0.0025, target_yaw=target_yaw, minPower=200, maxPower=700)

        else:
            # ev3.speaker.beep()
            gyroPID(1.75, 0.055, target_yaw=target_yaw, ki=0.000001, minPower=200, maxPower=700, resetKI=False) 


        print("section: ", section)
        print("firstBlock:", firstBlock)
        print(firstBlockPerSection)       


def intersectionCount():
    global run, direction, color
    while True:
        color = rgb_to_color(sensor.rgb())
        # print("color", color)

    # while run <= 11:
    #     print("Run:", run)
    #     color = rgb_to_color(sensor.rgb())
    #     if color == 1 and (direction == 0 or direction == 1):
    #         direction = 1
    #         run += 1
    #         wait(500)
    #     elif color == -1 and (direction == 0 or direction == -1):
    #         direction = -1
    #         run += 1
    #         wait(500)

# def senseCam():
#     while True:
#         global 
#         status, data = st.call('cam')
#         globals()['balance'], globals()['BRB'], globals()['distance'], globals()['IRDist'], globals()['yaw'], globals()['blockX'] = data
 
#         # if globals()['yaw'] != 0 and yaw == True:


#         #globals()['IRDist'],

def senseCam():
    start_yaw = None
    while True:
        status, data = st.call('cam')
        globals()['balance'], globals()['BRB'], globals()['distance'], globals()['IRDist'], bad_yaw, globals()['blockX'] = data
        
        if start_yaw==None:
            start_yaw=bad_yaw
        if bad_yaw-start_yaw > 0:
            final_yaw=bad_yaw - start_yaw
        else:
            final_yaw= 360 + (bad_yaw - start_yaw)

        if round(final_yaw)==360:
            final_yaw=0
        globals()['yaw']=round(final_yaw)



threading.Thread(target=senseCam).start()
# threading.Thread(target=intersectionCount).start()
mainThread()

# while True:
#     print(fw.angle())