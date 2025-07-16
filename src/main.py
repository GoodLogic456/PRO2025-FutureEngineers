#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from serialtalk.auto import SerialTalk


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

ev3 = EV3Brick()
st = SerialTalk(Port.S1) #camera
bw = Motor(Port.B) #foward and back
fw = Motor(Port.C) #left and right
sensor = ColorSensor(Port.S3) #color sensor
Lultra = UltrasonicSensor(Port.S2) #left ultrasonic sensor
Rultra = UltrasonicSensor(Port.S4) #right ultrasonic sensor

# Write your program here.

adjust = 0
balance  = 0
balanceRB = 0
direction = 0
max_turnR = 900   #1274
max_turnL = -900 # -2795
fw.reset_angle(0)
fwspeed = 0
bwspeed = 0
prev_error = 0
error = 0
run = 0
LeftWins = True

def turnAngle2(kp, kd, balance):  #### use the white balance(get more information on white balance from the camera code) for  proportional pd controller to control the speed and angle for the robot to turn away from the wall
    p = 0
    d = 0
    rot = 0
    prev_error = 0

    mRot = fw.angle()   # current motor rotation
    
    if balance > 0:
        rot = (max_turnR/1100) * balance
    else:
        rot = (max_turnL/-1100) * balance * 1.25    # *1.35
        # *1.35
    
    print("mRot:" + str(mRot))
    print("rot:" + str(rot))
    print("balance:" + str(balance))
    
    error = mRot - rot

    p = -1.0 * kp * error
    d = -1.0 * kd * (error - prev_error)  
    fwspeed = int(p + d )
    prev_error = error
    
    print("fwspeed:" + str(fwspeed))
    print(" ")
    
    fw.run(fwspeed)


def PIDultraL(kp, kpf, kdf, min, max, target):      ######### use the distance of Ultrasonic Sensor for proportional pd controller to go away from the wall (Left side)
    p = 0
    d = 0
    rot = 0
    mRot = fw.angle()   # current motor rotation

    global fwspeed, bwspeed, prev_error, error, direction

    ultraDistL = Lultra.distance()

    error = target - ultraDistL
    rot = kp * error

    
    if error > 0:
        if mRot < rot:
            p = kpf * abs(rot)
            d = kdf * (error - prev_error)
            prev_error = error
            fwspeed = int(p + d)
        elif mRot > rot:
            p = kpf * abs(rot)
            d = kdf * (error - prev_error)
            prev_error = error
            fwspeed = int(-(p + d))
    
        fw.run(fwspeed)
    
    else:
        fw.stop()

    # if abs(rot) > 0:
    #     bwspeed = (abs((min - max) / (1100 * abs(rot))) + max)
    #     bwspeed = int(bwspeed)
    #     bw.run(bwspeed)    #     bw.run(300)
    # else:
    #     bw.run(min)



    print("error:" + str(error))


def PIDultraR(kp, kpf, kdf, min, max, target):      ######### use the distance of Ultrasonic Sensor for proportional pd controller to go away from the wall (Right side)

    p = 0
    d = 0
    rot = 0
    mRot = fw.angle()   # current motor rotation

    global fwspeed, prev_error, error, direction

    ultraDistR = Rultra.distance()

    error = ultraDistR - target
    rot = kp * error

    if error < 0:
        if mRot > rot:
            p = kpf * rot
            d = kdf * (error - prev_error)
            prev_error = error
            fwspeed = int(p + d)
        elif mRot < rot:
            p = kpf * rot 
            d = kdf * (error - prev_error)
            prev_error = error
            fwspeed = int(abs((p + d)))

        fw.run(fwspeed)

    else:
        fw.stop()
    

  
    # if abs(rot) > 0:
    #     bwspeed = (abs((min - max) / 1100 * abs(rot)) + max)
    #     bwspeed = int(bwspeed)
    #     bw.run(bwspeed)    #     bw.run(300)
    # else:
    #     bw.run(min)


    print("error:" + str(error))
    

def setBWSpeed(min, max, balance):  # min and max must be positive  #####speed depends on how far the robot from the wall
    bwSpeed = (min - max) / 1100 * abs(balance) + max
    bwSpeed = int(bwSpeed)
    bw.run(bwSpeed)


def rgb_to_color(rgb): ######### use ev3 Color Sensor to detect the color
    r, g, b = rgb
    if r <= 20:
        # ev3.speaker.beep()
        return -1    # blue

    elif b <= 10:
        # ev3.speaker.beep()
        return 1     # orange

    else:
        return 0     # white
#####################

# ###############################openchallenge
# while True:
#     b = brick.buttons()
#     if Button.RIGHT in b:
while run <= 11:

    status, data = st.call('cam')
    balance, balanceRB, distance = data

    color = rgb_to_color(sensor.rgb())
    print("Direction: " + str(direction))

    Lultra_distance = Lultra.distance()
    Rultra_distance = Rultra.distance()

    if Lultra_distance < Rultra_distance:
        LeftWins = True
    else:
        LeftWins = False


    if (Lultra_distance + Rultra_distance) < 600: ######### if the distance between the wall is too near will only use white balance as error
    # if (Lultra.distance() + Rultra.distance()) < 600 and balance == 0.0:
        setBWSpeed(150, 400, balance)
        turnAngle2(130.0, 2.25, balance)

    elif balance <= -1000 or balance >= 1000:  # severe case #########the robot is too close from the wall
        setBWSpeed(150, 150, balance)
        turnAngle2(100.0, 2.25, balance)

    elif fw.angle() < 200 and fw.angle() > -200: # if fairly straight of the front wheel and not close to the wall the robot back wheel can run max speed
        setBWSpeed(150, 600, balance)
        turnAngle2(120.0, 2.25, balance)

    else: # normal camera mode
        setBWSpeed(150, 400, balance)
        turnAngle2(120.0, 2.25, balance)


    if rgb_to_color(sensor.rgb()) == 1: # orange

        if direction == 0 or direction == 1: # no direction or clockwise
            # ev3.speaker.beep()
            while rgb_to_color(sensor.rgb()) > -1:  # not yet blue (from orange)
                if fw.angle() <= 100 and Rultra_distance > 150:  ########If the front wheel angle is less than 100 and not close to the wall, the front wheel will move at a 800 speed until the front wheel is greater than 100 degrees or the robot is too close to the wall.
                    bw.run(400)
                    fw.run(800)     
            else:
                while fw.angle() >= 0:
                    bw.run(100)
                    # ev3.speaker.beep()
                    # bw.stop()
                    fw.run(-1100)

            run = run + 1
            direction = 1
        


    if rgb_to_color(sensor.rgb()) == -1: # blue

        if direction == 0 or direction == -1: # no direction or counter-clockwise
            # ev3.speaker.beep()

            while rgb_to_color(sensor.rgb()) < 1:  # not yet orange (from blue)
                if fw.angle() >= -100 and Lultra_distance > 150: ########If the front wheel angle is greater than -100 and not close to the wall, the front wheel will move at a -800 speed until the front wheel is less than -100 degrees or the robot is too close to the wall.
                    bw.run(400)
                    fw.run(-800)        

            else:
                while fw.angle() <= 0:
                    bw.run(100)
                    ev3.speaker.beep()
                    # bw.stop()
                    fw.run(1100)

            run = run + 1

            direction = -1



    if direction == -1: ####### If it's counter-clockwise, the robot will only activate the Left side of the Ultrasonic Sensor for it to go away from the inner wall
        if Lultra_distance < 250 :
            ev3.speaker.beep()
            PIDultraL(10, 100, 0.001, 200, 300, 250)
            bw.run(400)  

    if direction == 1: ####### If it's clockwise, the robot will only activate the Right side of the  Ultrasonic Sensor for it to go away from the inner wall
        if Rultra_distance < 250 :        
            ev3.speaker.beep()
            PIDultraR(10, 100, 0.001, 200, 300, 250)
            bw.run(400) 

    if direction == 0: ####### From the very start, we don't know which direction the robot will drive, so activate both sides of the Ultrasonic Sensor for it to go away from the inner wall
        if Lultra_distance < 250 :
            ev3.speaker.beep()
            PIDultraL(10, 100, 0.001, 200, 300, 250)
            bw.run(400)  

        if Rultra_distance < 250 :        
            ev3.speaker.beep()
            PIDultraR(10, 100, 0.001, 200, 300, 250)
            bw.run(400)  

        run = 0

if run > 11: ############ After three rounds of the track, the robot should stop on its original region 
    
    bw.reset_angle(0)
    while bw.angle() < 1000:
        status, data = st.call('cam')
        balance, turnangle, distance = datas
        
        setBWSpeed(150, 1100, balance)
        turnAngle2(100.0, 2.25, balance)
    
    run = 20

if run ==20:
    fw.stop()
    bw.stop()