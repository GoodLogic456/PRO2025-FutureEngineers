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
st = SerialTalk(Port.S1)
bw = Motor(Port.B) #foward and back
fw = Motor(Port.C) #left and right
sensor = ColorSensor(Port.S3) #color sensor
Lultra = UltrasonicSensor(Port.S2) #left ultrasonic sensor
Rultra = UltrasonicSensor(Port.S4) #right ultrasonic sensor

# db = DriveBase(lm,rm,62, 19*8)

# Write your program here.
ev3.speaker.beep()

# status, data = st.call('cam')
# turnangle,balance,distance  = [0,0,0]
distance = 0
balance  = 0
BRB = 0
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
sensedBlock = 1
turn = 0


# def turnAngle(kp, kd, BRB):
#     p = 0
#     d = 0
#     rot = 0
#     prev_error = 0

#     mRot = fw.angle()   # current motor rotation
    
#     if BRB > 0:
#         rot = (max_turnR/1100) * BRB 
#     else:
#         rot = (max_turnL/-1100) * BRB  # *1.35

#     # print("mRot:" + str(mRot))
#     # print("rot:" + str(rot))
#     print("BRB:" + str(BRB))
    
#     error = mRot - rot

#     p = -1.0 * kp * error
#     d = -1.0 * kd * (error - prev_error)  
#     fwspeed = int(p + d )
#     prev_error = error
    
#     # print("fwspeed:" + str(fwspeed))
#     # print(" ")

#     if fwspeed > 1100:
#         fwspeed = 1100
#     elif fwspeed < -1100:
#         fwspeed = -1100
    
#     fw.run(fwspeed)


def turnAngle2(kp, kd, balance):
    
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
    
    
    
    # p = 0
    # d = 0
    # rot = 0
    # prev_error = 0

    # mRot = fw.angle()   # current motor rotation
    
    # if balance > 0:
    #     rot = (max_turnR/1100) * balance
    # else:
    #     rot = (max_turnL/-1100) * balance * 1.25    # *1.35
    #     # *1.35
    
    # # print("mRot:" + str(mRot))
    # # print("rot:" + str(rot))
    # # print("balance:" + str(balance))
    
    # error = mRot - rot

    # p = -1.0 * kp * error
    # d = -1.0 * kd * (error - prev_error)  
    # fwspeed = int(p + d )
    # prev_error = error
    
    # # print("fwspeed:" + str(fwspeed))
    # # print(" ")
    
    # # if fwspeed > 1100:
    # #     fwspeed = 1100
    # # elif fwspeed < -1100:
    # #     fwspeed = -1100

    # fw.run(fwspeed)


def PIDultraL(kp, kpf, kdf, min, max, target):
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



    # print("error:" + str(error))


def PIDultraR(kp, kpf, kdf, min, max, target):

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
    


def setBWSpeed(min, max, balance):  # min and max must be positive
    bwSpeed = (min - max) / 1100 * abs(balance) + max
    bwSpeed = int(bwSpeed)
    bw.run(bwSpeed)

def setBWSpeedRB(min, max, BRB):  # min and max must be positive
    bwSpeed = (min - max) / 1100 * abs(BRB) + max
    bwSpeed = int(bwSpeed)
    bw.run(bwSpeed)

def turnAngle3(kpf, kdf, balance):

    p = 0
    d = 0
    rot = 0
    mRot = fw.angle()   # current motor rotation

    global fwspeed, bwspeed, prev_error, error, direction

    if balance > 0:
        rot = (max_turnR/1100) * balance
    else:
        rot = (max_turnL/-1100) * balance * 1.35    # *1.35

    error = mRot - rot

    print("balance: " + str(balance))

    
    if mRot < rot:
        p = kpf * error
        d = kdf * (prev_error - error)
        prev_error = error
        fwspeed = int(-(p + d))
        fw.run(fwspeed)
    else:
        p = kpf * error
        d = kdf * (prev_error - error)
        prev_error = error
        fwspeed = int(-(p + d)) 
        fw.run(fwspeed)

def turnAngle4(kpf, kdf, MGbalance):

    p = 0
    d = 0
    rot = 0
    mRot = fw.angle()   # current motor rotation

    global fwspeed, bwspeed, prev_error, error, direction

    if balance > 0:
        rot = (max_turnR/1100) * MGbalance
    else:
        rot = (max_turnL/-1100) * MGbalance * 1.35    # *1.35

    error = mRot - rot
    
    if mRot < rot:
        p = kpf * error
        d = kdf * (prev_error - error)
        prev_error = error
        fwspeed = int(-(p + d))
        fw.run(fwspeed)
    else:
        p = kpf * error
        d = kdf * (prev_error - error)
        prev_error = error
        fwspeed = int(-(p + d)) 
        fw.run(fwspeed)
  

def turnAngle(kpf, kdf, BRB):
  
    p = 0
    d = 0
    rot = 0
    mRot = fw.angle()   # current motor rotation

    global fwspeed, bwspeed, prev_error, error, direction

    if balance > 0:
        rot = (max_turnR/1100) * BRB
    else:
        rot = (max_turnL/-1100) * BRB * 1.35    # *1.35

    error = mRot - rot
    
    if mRot < rot:
        p = kpf * error
        d = kdf * (prev_error - error)
        prev_error = error
        fwspeed = int(-(p + d))
        fw.run(fwspeed)
    else:
        p = kpf * error
        d = kdf * (prev_error - error)
        prev_error = error
        fwspeed = int(-(p + d)) 
        fw.run(fwspeed)
  





######################3
def rgb_to_color(rgb):
    r, g, b = rgb
    if r <= 20:
        # ev3.speaker.beep()
        return -1    # blue

    elif b <= 10:
        ev3.speaker.beep()
        return 1     # orange

    else:
        return 0     # white
#####################



while run <= 8:

    status, data = st.call('cam')
    balance, BRB, distance, MGbalance = data
    BRB = BRB * 1.25
    # print("BRB: " + str(BRB))
    # print("distance: " + str(distance))

    color = rgb_to_color(sensor.rgb())
    print(color)
    # print("Direction: " + str(direction))
    # print("balance: " + str(balance))

    # print("Rotation: " + str(fw.angle()))

    Lultra_distance = Lultra.distance()
    Rultra_distance = Rultra.distance()

#############
    if BRB != 0:
        # ev3.speaker.beep()
        turnAngle(1000, 0, BRB)
        setBWSpeedRB(300, 400, BRB)
        sensedBlock = 1


    # elif balance != 0 and BRB == 0:
    #     # ev3.speaker.beep()
    #     setBWSpeed(100, 400, balance)
    #     turnAngle3(50, 10, balance)
    
    
    # elif rgb_to_color(sensor.rgb()) == 1: # orange
    #     if direction == 0 or direction == 1: # no direction or clockwise
    #         if BRB == 0:
    #             while fw.angle() <= 900 and BRB == 0:
    #                 bw.run(300)
    #                 # ev3.speaker.beep()
    #                 fw.run(1100)

    #             # if fw.angle() >= 0:
    #             #     bw.run(100)
    #             #     fw.run(-1100)
        
    #         run = run + 1
    #         direction = 1  
    #         fw.stop()  

        
    elif color == 1: # orange
        if direction == 0 or direction == 1: # no direction or clockwise
            while fw.angle() <= 500 :
                ev3.speaker.beep()
                bw.stop()
                fw.run(1100)

            # if fw.angle() >= 500:
            #     bw.run(-100)
          
            run = run + 1
            direction = 1  
            # fw.stop()  

    # elif rgb_to_color(sensor.rgb()) == -1: # blue
    #     if direction == 0 or direction == -1: # no direction or counter-clockwise
    #         if BRB == 0:
    #             while fw.angle() >= -800:
    #                 # ev3.speaker.beep()
    #                 bw.run(300)
    #                 fw.run(-1100)  

    #         run = run + 1
    #         direction = -1

    #         fw.stop()

    elif color == -1: # orange
        if direction == 0 or direction == -1: # no direction or clockwise
            while fw.angle() >= -500 :
                ev3.speaker.beep()
                bw.stop()
                fw.run(-1100)

            # if fw.angle() >= 500:
            #     bw.run(-100)
          
            run = run + 1
            direction = 1  
            # fw.stop()  
    
    else:
        # ev3.speaker.beep()
        setBWSpeed(100, 400, balance)
        turnAngle3(50, 10, balance)

    

    # if direction == -1: ####### If it's counter-clockwise, the robot will only activate the Left side of the Ultrasonic Sensor for it to go away from the inner wall
    #     if Lultra_distance < 300 and Lultra_distance > 30:
    #         ev3.speaker.beep()
    #         PIDultraL(10, 80, 0.001, 200, 300, 250)
    #         bw.run(400)  

    # if direction == 1: ####### If it's clockwise, the robot will only activate the Right side of the  Ultrasonic Sensor for it to go away from the inner wall
    #     if Rultra_distance < 300 and Rultra_distance > 30:        
    #         ev3.speaker.beep()
    #         PIDultraR(10, 80, 0.001, 200, 300, 250)
    #         bw.run(400) 

    # if direction == 0: ####### From the very start, we don't know which direction the robot will drive, so activate both sides of the Ultrasonic Sensor for it to go away from the inner wall
    #     if Lultra_distance < 250 and Lultra_distance > 30:
    #         ev3.speaker.beep()
    #         PIDultraL(10, 80, 0.001, 200, 300, 250)
    #         bw.run(400)  

    #     if Rultra_distance < 250 and Rultra_distance > 30:        
    #         ev3.speaker.beep()
    #         PIDultraR(10, 80, 0.001, 200, 300, 250)
    #         bw.run(400)  

    #     run = 0
    
##########################    
    # elif BRB == 0 and sensedBlock == 1:
    #     sensedBlock = 0

    # if balance >= 850 or balance <= -850:


    # fw.stop()
    # elif rgb_to_color(sensor.rgb()) == 0 and sensedBlock == 0: 

    #     if fw.angle() > 0:           
    #         ev3.speaker.beep()
    #         while fw.angle() > 0 and rgb_to_color(sensor.rgb()) != 0:
    #             fw.run(-500)
    #             bw.run(500)
            
    #         fw.stop()
    #         bw.reset_angle(0)
    #         bw.run_target(500, 360, wait=False)
    #         bw.run(200)
    #         # wait(5000)

    #     else:
    #         ev3.speaker.beep()
    #         while fw.angle() < 0 and rgb_to_color(sensor.rgb()) != 0:
    #             fw.run(500)
    #             bw.run(500)
        
    #         fw.stop()
    #         bw.stop()
    #         bw.reset_angle(0)
    #         bw.run_target(500, 360, wait=False)
    #         bw.run(200)
    #         # wait(5000)

        
        
    #     sensedBlock = 2

    # # elif fw.angle() !=0:
    # #     if BRB == 0 :
    # #         while fw.angle() > 50:
    # #             fw.run(-1100)
    # #             bw.run(200)

    # #         while fw.angle() < -50:
    # #             fw.run(1100)
    # #             bw.run(200)

    # # fw.stop()
#################################        

while run > 8 and run < 12:
    if turn == 0:
        fw.run_target(1100, 800)
        wait(100)
        bw.run_target(500, 300)
        wait(100)
        fw.run_target(1100, -1000)
        wait(100)
        bw.run_target(500, -300)
        wait(100)
        fw.run_target(1100, 1000)
        wait(100)
        bw.run_target(500, 200)
        wait(100)
        fw.run_target(1100, -500)
        wait(100)
        turn = 2
        direction = direction * -1

#############
    elif BRB != 0:
        # ev3.speaker.beep()
        turnAngle(1000, 0, BRB)
        setBWSpeedRB(300, 400, BRB)
        sensedBlock = 1


    # elif balance != 0 and BRB == 0:
    #     # ev3.speaker.beep()
    #     setBWSpeed(100, 400, balance)
    #     turnAngle3(50, 10, balance)
    
    
    # elif rgb_to_color(sensor.rgb()) == 1: # orange
    #     if direction == 0 or direction == 1: # no direction or clockwise
    #         if BRB == 0:
    #             while fw.angle() <= 900 and BRB == 0:
    #                 bw.run(300)
    #                 # ev3.speaker.beep()
    #                 fw.run(1100)

    #             # if fw.angle() >= 0:
    #             #     bw.run(100)
    #             #     fw.run(-1100)
        
    #         run = run + 1
    #         direction = 1  
    #         fw.stop()  

        
    elif color == 1: # orange
        if direction == 0 or direction == 1: # no direction or clockwise
            while fw.angle() <= 500 :
                ev3.speaker.beep()
                bw.stop()
                fw.run(1100)

            # if fw.angle() >= 500:
            #     bw.run(-100)
          
            run = run + 1
            direction = 1  
            # fw.stop()  

    # elif rgb_to_color(sensor.rgb()) == -1: # blue
    #     if direction == 0 or direction == -1: # no direction or counter-clockwise
    #         if BRB == 0:
    #             while fw.angle() >= -800:
    #                 # ev3.speaker.beep()
    #                 bw.run(300)
    #                 fw.run(-1100)  

    #         run = run + 1
    #         direction = -1

    #         fw.stop()

    elif color == -1: # orange
        if direction == 0 or direction == -1: # no direction or clockwise
            while fw.angle() >= -500 :
                ev3.speaker.beep()
                bw.stop()
                fw.run(-1100)

            # if fw.angle() >= 500:
            #     bw.run(-100)
          
            run = run + 1
            direction = 1  
            # fw.stop()  
    
    else:
        # ev3.speaker.beep()
        setBWSpeed(100, 400, balance)
        turnAngle3(50, 10, balance)

    

    # if direction == -1: ####### If it's counter-clockwise, the robot will only activate the Left side of the Ultrasonic Sensor for it to go away from the inner wall
    #     if Lultra_distance < 300 and Lultra_distance > 30:
    #         ev3.speaker.beep()
    #         PIDultraL(10, 80, 0.001, 200, 300, 250)
    #         bw.run(400)  

    # if direction == 1: ####### If it's clockwise, the robot will only activate the Right side of the  Ultrasonic Sensor for it to go away from the inner wall
    #     if Rultra_distance < 300 and Rultra_distance > 30:        
    #         ev3.speaker.beep()
    #         PIDultraR(10, 80, 0.001, 200, 300, 250)
    #         bw.run(400) 

    # if direction == 0: ####### From the very start, we don't know which direction the robot will drive, so activate both sides of the Ultrasonic Sensor for it to go away from the inner wall
    #     if Lultra_distance < 250 and Lultra_distance > 30:
    #         ev3.speaker.beep()
    #         PIDultraL(10, 80, 0.001, 200, 300, 250)
    #         bw.run(400)  

    #     if Rultra_distance < 250 and Rultra_distance > 30:        
    #         ev3.speaker.beep()
    #         PIDultraR(10, 80, 0.001, 200, 300, 250)
    #         bw.run(400)  

    #     run = 0









        

  
