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

#####left power + right power -
#####gyro left - right +

# Create your objects here.
ev3 = EV3Brick()
st = SerialTalk(Port.S4)
bw = Motor(Port.A) #foward and back
fw = Motor(Port.C) #left and right
sensor = ColorSensor(Port.S2) #color sensor
# gyro = GyroSensor(Port.S3) #gyro sensor
timer = StopWatch()
ev3.speaker.beep()

fw.reset_angle(0)
bw.reset_angle(0)
# gyro.reset_angle(0)

direction = 0
lane = 1

prev_error = 0
error = 0
sum_error = 0
final_error = 0
Ferror = 0

passedABlock=True
greenmove = False
color = 0
target = 0
section = 0
angle = 0

target_yaw = 0

saveCD = 0

kp = 1.5
kd = 2.5
ki = 0.000001

# variables from camera
balance = 0 
BRB = 0 
distance =  999
IRDist = 9999
yaw = 0
blockX = 0



firstBlockPerSection = [0, 0, 0, 0]
firstBlock = False


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

# def steerToAngle(bwPower, fwPower, time=1250, steering=0):
#     global timer
    
#     bw.run(bwPower)
#     timer.reset()

#     if fw.angle() < steering:
#         while fw.angle() <= steering and timer.time() < time:  #steering facing to the left 
#             # print("fw_angle()", fw.angle())       
#             fw.run(fwPower)
#     else:
#         while fw.angle() >= steering and timer.time() < time:   #steering facing to the right
#             # print("fw_angle()", fw.angle())
#             fw.run(-fwPower)

#     fw.stop()
#     fw.brake()

def steerToAngle(bwPower, fwPower, time=750, steering=0):

    global timer
    
    bw.run(bwPower)
    timer.reset()

    kp = 12.5
    kd = 5.0

    prev_error = 0

    if fw.angle() < steering:
        error = steering - fw.angle() 
        while abs(error) > 0 and timer.time() < time:  #steering facing to the left 
            print("error", error)
            error = steering - fw.angle()
            fwPower = kp * error + kd * (error - prev_error) 

            if fwPower > 0 and fwPower < 75:
                fwPower = 75
            elif fwPower < 0 and fwPower > -75:
                fwPower = -75

            fw.run(fwPower)
            prev_error = error
    else:
        error = fw.angle() - steering
        while abs(error) > 0 and timer.time() < time:  #steering facing to the left 
            print("error", error)
            error = fw.angle() - steering
            fwPower = kp * error + kd * (error - prev_error)  

            if fwPower > 0 and fwPower < 75:
                fwPower = 75
            elif fwPower < 0 and fwPower > -75:
                fwPower = -75


            fw.run(-fwPower)
            prev_error = error

    fw.stop()
    fw.brake()

def calc_turn_error(yaw,target_yaw):
    global final_error
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

def sign(error): #positive =1 negative-1
    return (error > 0) - (error < 0)

def gyroPID(kp, kd, target_yaw, ki=0, minPower=200, maxPower=700, resetKI=True):
    
    global prev_error, error, yaw, section, sum_error

    if resetKI:
        sum_error = 0

    if minPower > 0 or maxPower > 0:
        error = calc_turn_error(yaw, target_yaw)
    else:
        error = -(calc_turn_error(yaw, target_yaw)) 

    # Calculate the derivative of the error
    d_error = error - prev_error

    sum_error += error

    # Calculate the control signal
    control_signal = kp * error + kd * d_error + ki * sum_error

    # Update previous error
    prev_error = error

    if control_signal > 0:
        fwspeed = -abs((maxPower - (abs(control_signal) * 50)))
    elif control_signal < 0:
        fwspeed = abs((maxPower - (abs(control_signal) * 50)))
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

    # if bwspeed > 0:
    #     bw.run(-700)
    # else:
    #     bw.run(700)
    
    bw.run(-maxPower)

    # print("yaw", yaw, "error:", error, "control_signal:", control_signal, "fwspeed:", fwspeed,"bwspeed:", bwspeed)

def gyroDistance(minPower,maxPower,target_yaw,distance):
    bw.reset_angle(0)

    global kp, kd, ki

    while abs(bw.angle()) < abs(distance):
        gyroPID(kp, kd, target_yaw=target_yaw, ki=ki, minPower=minPower, maxPower=maxPower, resetKI=True)

    bw.brake()

def gyroIRDist(minPower,maxPower,target_yaw,uldist):
    global IRDist, kp, kd, ki

    while IRDist > uldist:
        gyroPID(kp, kd, target_yaw=target_yaw, ki=ki, minPower=minPower, maxPower=maxPower, resetKI=True)
    bw.brake()

def gyroTurn(minpower, maxpower, steer, angle): # Ferror is the first value is not changing 

    global yaw, target_yaw
    begin_angle = yaw
    Ferror = calc_turn_error(begin_angle, (target_yaw + angle))
    error = calc_turn_error(yaw, (target_yaw + angle))

    if steer == 1:
        # steerToAngle(0, 300, steering = 45)
        steerToAngle(0, 300, steering = 65)
    elif steer == -1:
        # steerToAngle(0, 300, steering = -45)
        steerToAngle(0, 300, steering = -65)

    
    while sign(error) == sign(Ferror) and sign(Ferror)!=0:
        error = calc_turn_error(yaw, (target_yaw + angle))

        # print("error",error,"sign",sign(error))
        bwspeed = int(((maxpower - minpower ) * (abs(error)/abs(Ferror)) + minpower))
        bw.run(-bwspeed)

    bw.stop()

def corner(laneOut=1, saved=False):# default laneOut is inner lane
    global lane, target_yaw, section, sum_error,direction, timer, saveCD
    sum_error = 0
    bw.reset_angle(0)

    if laneOut == 1:
        gyroDistance(-200,-700,target_yaw,1500)
        lane = 1
    elif laneOut == -1:
        gyroDistance(-200,-700,target_yaw,500)
        lane = -1
    elif laneOut == 0:
        gyroDistance(-200,-700,target_yaw,750)                
        lane = -1
    wait(100)    
    gyroTurn(150, 900, (-1 * direction), (85 * direction) )
    steerToAngle(0, 300, steering = 0)
    # wait(2000)
    if saved==False:gyroDistance(-200,-700,(target_yaw+(90*direction)),600)
    saveCD = timer.time() + 1500
    section = section + direction

def shiftlane2():
    global lane, greenmove, section , direction, BRB, kp, kd, ki
    if (BRB * direction) == 1:#clockwise=red,counterclockwise=green
        storeFirstBlockPerSection(abs(section),1)
        if lane == -1 :
            gyroTurn(150, 900, (-1 * direction) , angle = (55 * direction))

            if abs(section) % 4 == 0:
                gyroDistance(200,500,target_yaw=(target_yaw + (55 * direction)),distance=100)
            else:
                gyroDistance(200,500,target_yaw=(target_yaw + (55 * direction)),distance=400)


            gyroTurn(150, 700, (1 * direction), angle = 0)
            steerToAngle(0, 300, steering = 0)
            if abs(section)<5:
                gyroDistance(-200,-800,target_yaw=target_yaw,distance=150)
            lane = 1
        else:
            gyroPID(kp, kd, target_yaw=target_yaw, ki=ki, minPower=200, maxPower=700, resetKI=True)  
    elif (BRB * direction) == -1:#clockwise=green,counterclockwise=red
        storeFirstBlockPerSection(abs(section),-1)
        if lane == 1:
            greenmove = True
            steerToAngle(0, 150, steering = 0)
            gyroTurn(150, 900, (1 * direction), angle=(-85 * direction))
            steerToAngle(0, 150, steering = 0)
            gyroIRDist(150, 500, target_yaw=(target_yaw -(85 * direction)), uldist=200)
            steerToAngle(0, 150, steering = 0)
            if abs(section) % 4 == 0:
                gyroDistance(-200,-800,target_yaw=(target_yaw - (85 * direction)),distance=700)            
            else:
                gyroDistance(-200,-800,target_yaw=(target_yaw - (85 * direction)),distance=400)
            gyroTurn(150, 800, (-1 * direction),angle=0)
            steerToAngle(0, 300, steering = 0)
            greenmove = False
            lane = -1
        else:
            gyroPID(kp, kd, target_yaw=target_yaw, ki=ki, minPower=200, maxPower=700, resetKI=True)    

def storeFirstBlockPerSection(sectionToApply, movement):
    global firstBlock, firstBlockPerSection
    if firstBlock and firstBlockPerSection[abs(sectionToApply) % 4] == 0:
        firstBlockPerSection[abs(sectionToApply) % 4] = movement  # store movement block
        firstBlock = False

def mainThread():
    global direction, lane, target_yaw, section, BRB, timer, kp, kd, ki
    print(firstBlockPerSection)
    timer.reset()
 
    #get out of parking
    if balance > 0.0:  # clockwise movement (turn right from parking)
        direction = 1
        gyroTurn(100, 700, -1, angle = 45)
        steerToAngle(0, 300, steering = 0)
        if BRB == -1:  # green
            steerToAngle(0, 300, steering = 0)
            gyroDistance(200,500,target_yaw = 45 , distance=80)            
            gyroTurn(100, 700, 1, angle = -10)
            steerToAngle(0, 300, steering = 0)
            lane = -1
        elif BRB == 1  or BRB == 0:  # red
            gyroTurn(100, 500, -1, angle = 90)
            steerToAngle(0, 300, steering = 0)
            gyroDistance(200,500,target_yaw = 90 , distance=270)
            gyroTurn(200, 700, 1, angle = 0)
            steerToAngle(0, 300, steering = 0)
            lane = 1
    elif balance < 0.0:  # counter-clockwise movement (turn left from parking)
        direction = -1
        gyroTurn(100, 800, 1, angle = -62)
        wait(100)        
        if BRB == -1:  # green
            # gyroDistance(-200,-500,target_yaw = -62 , distance=270)
            # wait(100)
            gyroTurn(100, 500, 1, angle = -90)
            wait(100)
            steerToAngle(0, 400, steering = 0)
            wait(100)
            gyroDistance(200,500,target_yaw = -90 , distance=310)
            wait(100)
            gyroTurn(200, 700, -1, angle = 0)
            wait(100)
            steerToAngle(0, 200, steering = 0)
            wait(100)
            gyroDistance(-200,-500,target_yaw = 0 , distance=450)
            lane = 1
        elif BRB == 1  or BRB == 0:  # red
            steerToAngle(0, 150, steering = 0) 
            wait(100)
            gyroTurn(100, 700, -1, angle = 0)
            steerToAngle(0, 300, steering = 0)
            lane = -1

    while abs(section) < 12:
        global color, direction, lane , target_yaw, greenmove, section, firstBlock, distance, passedABlock, timer , saveCD
        color = rgb_to_color(sensor.rgb())
        target_yaw = (section % 4) * 90 
        nextSection = 0
        if (abs(section)%4)+1 != 4:nextSection=(abs(section)%4)+1
        print("target_yaw",target_yaw,"Memory:",firstBlockPerSection)
        # if saveCD<timer.time() and (saveCD-1000)<timer.time():
            # ev3.speaker.beep(100, 100)

        if lane == 1 and sign(blockX) == direction and distance <= 180 and saveCD<timer.time():
            gyroPID(kp, kd, target_yaw=target_yaw, ki=ki, minPower=200, maxPower=700, resetKI=False)
            # ev3.speaker.beep(100, 100)
            firstBlock = True
            storeFirstBlockPerSection((abs(section)%4)+1,BRB*direction)



        #corner movement
        elif IRDist < 150 and greenmove == False:
            firstBlock = True
            print("section: ",section,"Memory: ",firstBlockPerSection)
            if firstBlockPerSection[nextSection] != 0: # first movement of next section is already saved
                if nextSection == 0:
                    if firstBlockPerSection[nextSection] == 1:corner(laneOut=1,saved=True)
                    #elif firstBlockPerSection[nextSection] == -1:corner(laneOut=-1,saved=True)
                    else:corner(laneOut=0,saved=True)
                else:
                    if firstBlockPerSection[nextSection] == 1:corner(laneOut=1,saved=True)
                    else:corner(laneOut=-1,saved=True)
            else:# no stored block yet
                if nextSection % 4 == 0:
                    corner(laneOut=0,saved=False)
                else:
                    if lane==1:
                        corner(laneOut=1,saved=True)
                    elif lane==-1:
                        corner(laneOut=1,saved=False)
        # block detected is too near
        elif distance < 170 and distance != 999:
            back=False

            #clockwise=red,counterclockwise=green
            if lane == -1 and BRB == (1 * direction):back=True
    
            #clockwise=red,counterclockwise=green
            #sign(blockX) != direction means not ignored block
            elif lane == 1 and BRB == (-1 * direction) and sign(blockX) != direction:back=True
            if back:
                steerToAngle(0, 300, steering = 0)
                while distance <= 190 and distance != 999:
                    # ev3.speaker.beep(100, 100)
                    gyroPID(kp, kd, target_yaw=target_yaw, ki=ki, minPower=-200, maxPower=-700, resetKI=True)
                bw.stop()
            else:
                gyroPID(kp, kd, target_yaw=target_yaw, ki=ki, minPower=200, maxPower=700, resetKI=True)   


        # block avoidance
        elif distance <= 205:
            #sign(blockX) != direction means not ignored block
            if lane == -1 and sign(blockX) != direction:shiftlane2()
            elif lane == 1 and sign(blockX) != direction:shiftlane2()
            else:gyroPID(kp, kd, target_yaw=target_yaw, ki=ki, minPower=200, maxPower=700, resetKI=True)
        # no block seen
        else:
            gyroPID(kp, kd, target_yaw=target_yaw, ki=ki, minPower=200, maxPower=700, resetKI=False)    


    # else: for ending
  





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

        # print("BRB", BRB, "yaw", yaw)



threading.Thread(target=senseCam).start()
# threading.Thread(target=intersectionCount).start()
wait(250)
mainThread()




# def intersectionCount():
#     global run, direction, color
#     while True:
#         color = rgb_to_color(sensor.rgb())
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