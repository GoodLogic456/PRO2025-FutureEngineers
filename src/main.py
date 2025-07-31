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
# Infra = InfraredSensor(Port.S4)

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
bw.reset_angle(0)
fwspeed = 0
bwspeed = 0
prev_error = 0
error = 0
run = 0
LeftWins = True
sensedBlock = 1
turn = 0
prevBW = 0
lap_count = 0
timer = StopWatch()


curr_time = 0
prev_time = 0
time_elapsed = 0
ignore = False

prev_distance = 0
prev_Bpik = 0
offTheWall = 1
colorPriority = 0





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

def turnAngleMG(kpf, kdf, MGbalance):

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

  


def turnAngleMG2(kpf, kdf, MGbalance): ##direction = 1

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
        fwspeed = int(p + d)
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
  

######################
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
#####################

def backToZeroSteering(bwPower, fwPower, color, steering=0):
    bw.run(bwPower)
    colorPriority = 0
    while fw.angle() >= steering:  #steering facing to the right
        if color != 0:
            bw.stop()
            colorPriority = 1
            break
        fw.run(-fwPower)
    
    if colorPriority == 0:
        while fw.angle() < steering:   #steering facing to the left
            if color != 0:
                bw.stop()
                colorPriority = 1
                break
            fw.run(fwPower)

    fw.stop()

    # if colorPriority:
    #     ev3.speaker.beep()


    return colorPriority
    

while True:
# while run <= 11:

    status, data = st.call('cam')
    balance, BRB, distance, MGbalance, pik, Bpik, balance2, BRC = data
    BRB = BRB * 1.5
    
    color = rgb_to_color(sensor.rgb())
    Lultra_distance = Lultra.distance()
    Rultra_distance = Rultra.distance()
    # Infra_distance = Infra.distance()
    
    # print("color: " + str(color))
    # print("BRB: " + str(BRB))
    # print("distance: " + str(distance))
    # print("run: " + str(run))
    # print("distance: " + str(distance)) 
    # print("Infra_distance: " + str(Infra_distance))    
    # print("Bpik: " + str(Bpik))
    # print("BRB: " + str(BRB))
    # print("Lultra_distance: " + str(Lultra_distance))
    # print("Rultra_distance: " + str(Rultra_distance))

# #############

    # MODE: block avoidance
    if pik >= 600 and distance <= 240 and color == 0:
        # ev3.speaker.beep()
        # turnAngle(650, 2.25, BRB)

        if distance < 30:
            backToZeroSteering(0, 1000, 0, steering=0)

            while distance < 50:
                ev3.speaker.beep()
                status, data = st.call('cam')
                balance, BRB, distance, MGbalance, pik, Bpik,balance2,BRC = data
                bw.run(-400)
        else:
            turnAngle(650, 7.5, BRB)
            setBWSpeedRB(100, 500, BRB)
            sensedBlock = 1
        
    # elif (color == 1 and distance == 0) or colorPriority:  # orange detected and no distance
    elif color == 1 or colorPriority:  # orange detected and no distance
        ev3.speaker.beep()
        
        # print("time elapsed: " + str(timer.time()))
       
        # if curr_time == 0:
        #     prev_time = -9999
        # else:
        #     prev_time = curr_time
        
        # curr_time = timer.time()
        # time_elapsed = curr_time - prev_time

        # if time_elapsed < 10000:
        #     ignore = True
        # else:
        #     run += 1
        #     ignore = False


        if direction == 0 or direction == 1 and not ignore:  # no direction or clockwise
            colorPriority = 0
            direction = 1

            fw.stop()
            bw.run_angle(1100, 850)
            backToZeroSteering(0, 1000, 0, steering=0)  # zero parameter means we dont care about the color
            
            # while fw.angle() >= 50 or fw.angle() <= -50:
            #     if fw.angle() > 0:
            #         bw.stop()
            #         fw.run(-1100)
            #     elif fw.angle() < 0:
            #         bw.stop()
            #         fw.run(1100)

            # fw.stop()

            while distance == 0:
                status, data = st.call('cam')
                balance, BRB, distance, MGbalance, pik, Bpik,balance2,BRC = data
                if fw.angle() > max_turnL:
                    fw.run(-1100)
                    bw.run(-300)                    
                else:
                    fw.run(-1100)
            else:
                bw.reset_angle(0)
                bw.run_angle(-300, 200)

            # bw.stop()
            # fw.stop()
            
            # while fw.angle() >= 100 or fw.angle() <= -100:
            #     if fw.angle() > 0:
            #         bw.stop()
            #         fw.run(-1100)

            #     elif fw.angle() < 0:
            #         bw.stop()
            #         fw.run(1100)
                
            
            # fw.stop()

            # if BRC < 0:  # green = -1
            #     # Turn backward to -350 degrees
            #     while fw.angle() > -200:
            #         # ev3.speaker.beep()
            #         fw.run(-1100)
            #         bw.run(50)
            #     fw.stop()
            #     bw.stop()

            #     # backToZeroSteering(200, 1000, 0)  # zero parameter means we dont care about the color


            # else:  # red = 1
            #     # Turn forward to 350 degrees
            #     while fw.angle() < 200:
            #         # ev3.speaker.beep()
            #         fw.run(1100)
            #         bw.run(50)
            #     fw.stop()
            #     bw.stop()

            backToZeroSteering(0, 1000, 0, steering=0) 
            wait(500) 
            # bw.run(300)

        
    # elif (color == -1 and distance == 0) or colorPriority:  # blue detected and no distance
    elif color == -1 or colorPriority:  # blue detected
        # print("time elapsed: " + str(timer.time()))
        if direction == 0 or direction == -1:  # no direction or clockwise
            colorPriority = 0
            direction = -1
            bw.run_angle(1100, 550)

            while fw.angle() >= 50 or fw.angle() <= -50:
                if fw.angle() > 0:
                    bw.stop()
                    fw.run(-1100)
                elif fw.angle() < 0:
                    bw.stop()
                    fw.run(1100)

            # fw.stop()

            while distance == 0:
                status, data = st.call('cam')
                balance, BRB, distance, MGbalance, pik, Bpik, balance2, BRC = data
                if fw.angle() < max_turnR:
                    bw.run(-400)
                    fw.run(1100)
                else:
                    fw.run(1100)
            else:
                bw.reset_angle(0)
                bw.run_angle(-400, 150)

            # bw.stop()
            # fw.stop()
            
            while fw.angle() >= 100 or fw.angle() <= -100:
                if fw.angle() > 0:
                    bw.stop()
                    fw.run(-1100)

                elif fw.angle() < 0:
                    bw.stop()
                    fw.run(1100)
                
            bw.run(200)
            fw.stop()

            if BRC < 0:
                # Turn backward to -350 degrees
                while fw.angle() > -200:
                    # ev3.speaker.beep()
                    fw.run(-1100)
                    bw.run(50)
                fw.stop()
                bw.stop()
            else:
                # Turn forward to 350 degrees
                while fw.angle() < 200:
                    # ev3.speaker.beep()
                    fw.run(1100)
                    bw.run(50)
                fw.stop()
                bw.stop()

        # if timer.time() - pretimer < 50000:
        #     run = run + 1 

        # pretimer = timer.time()
     
    elif abs(fw.angle()) > 200 and ((distance == 0 and prev_distance == 0) or (prev_distance - distance < 0)) and sensedBlock == 1:
        # ev3.speaker.beep()

        colorPriority = backToZeroSteering(250, 1000, color, steering=0)
        sensedBlock = 0
    
    
    elif abs(fw.angle()) > 200 and Bpik - prev_Bpik < 0 and Bpik <= 20 and Bpik > 0 and offTheWall == 1:
        # ev3.speaker.beep()
        if balance > 0:
            colorPriority = backToZeroSteering(250, 1000, color, steering=100)
        elif balance < 0:
            colorPriority = backToZeroSteering(250, 1000, color, steering=-100)
        else:
            colorPriority = backToZeroSteering(250, 1000, color, steering=0)

        offTheWall = 0
    
    elif Lultra_distance <= 150 and BRB == 0.0 and direction == 1 and Bpik >= 20:
        ev3.speaker.beep()
        bw.stop()

        while fw.angle() >= 100:
            ev3.speaker.beep()
            bw.stop()
            fw.run(-1100)

        while fw.angle() <= -100:
            ev3.speaker.beep()
            bw.stop()
            fw.run(1100)

        while distance == 0:
            status, data = st.call('cam')
            balance, BRB, distance, MGbalance, pik,Bpik,balance2,BRC = data
            fw.run(-1100)
            bw.run(-300)

        while fw.angle() >= 100:
            ev3.speaker.beep()
            bw.stop()
            wait(100)
            fw.run(-1100)

        while fw.angle() <= -100:
            ev3.speaker.beep()
            bw.stop()
            wait(100)
            fw.run(1100)
        
    elif Rultra_distance <= 150 and BRB == 0.0 and direction == 1 and Bpik >= 20:
        ev3.speaker.beep()
        bw.stop()
        while fw.angle() >= 100:
            ev3.speaker.beep()
            bw.stop()
            wait(100)
            fw.run(-1100)

        while fw.angle() <= -100:
            ev3.speaker.beep()
            bw.stop()
            wait(100)
            fw.run(1100)

        while distance == 0:
            status, data = st.call('cam')
            balance, BRB, distance, MGbalance, pik,Bpik,balance2,BRC= data
            fw.run(-1100)
            bw.run(-300)

        while fw.angle() >= 100:
            ev3.speaker.beep()
            bw.stop()
            # wait(100)
            fw.run(-1100)

        while fw.angle() <= -100:
            ev3.speaker.beep()
            bw.stop()
            # wait(100)
            fw.run(1100)

        if BRB < 0:
            # Turn backward to -350 degrees
            while fw.angle() > -200:
                ev3.speaker.beep()
                fw.run(-1100)
                bw.run(50)
            fw.stop()
            bw.stop()
        else:
            # Turn forward to 350 degrees
            while fw.angle() < 200:
                ev3.speaker.beep()
                fw.run(1100)
                bw.run(50)
            fw.stop()
            bw.stop()            
        
    elif Lultra_distance <= 150 and BRB == 0.0 and direction == -1 and Bpik >= 20:
        ev3.speaker.beep()
        bw.stop()
        while fw.angle() >= 100:
            ev3.speaker.beep()
            bw.stop()
            wait(100)
            fw.run(-1100)

        while fw.angle() <= -100:
            ev3.speaker.beep()
            bw.stop()
            wait(100)
            fw.run(1100)

        while distance == 0:
            status, data = st.call('cam')
            balance, BRB, distance, MGbalance, pik,Bpik,balance2,BRC= data
            fw.run(1100)
            bw.run(-300)

        while fw.angle() >= 100:
            ev3.speaker.beep()
            bw.stop()
            wait(100)
            fw.run(-1100)

        while fw.angle() <= -100:
            ev3.speaker.beep()
            bw.stop()
            wait(100)
            fw.run(1100)

        if BRB < 0:
            # Turn backward to -350 degrees
            while fw.angle() > -200:
                ev3.speaker.beep()
                fw.run(-1100)
                bw.run(50)
            fw.stop()
            bw.stop()
        else:
            # Turn forward to 350 degrees
            while fw.angle() < 200:
                ev3.speaker.beep()
                fw.run(1100)
                bw.run(50)
            fw.stop()
            bw.stop()            
        
    elif Rultra_distance <= 150 and BRB == 0.0 and direction == -1 and Bpik >= 20:
        ev3.speaker.beep()
        bw.stop()
        while fw.angle() >= 100:
            ev3.speaker.beep()
            bw.stop()
            wait(100)
            fw.run(-1100)

        while fw.angle() <= -100:
            ev3.speaker.beep()
            bw.stop()
            wait(100)
            fw.run(1100)

        while distance == 0:
            status, data = st.call('cam')
            balance, BRB, distance, MGbalance, pik,Bpik,balance2,BRC = data
            fw.run(1100)
            bw.run(-300)

        while fw.angle() >= 100:
            ev3.speaker.beep()
            bw.stop()
            wait(100)
            fw.run(-1100)

        while fw.angle() <= -100:
            ev3.speaker.beep()
            bw.stop()
            wait(100)
            fw.run(1100)

        if BRB < 0:
            # Turn backward to -350 degrees
            while fw.angle() > -200:
                ev3.speaker.beep()
                fw.run(-1100)
                bw.run(50)
            fw.stop()
            bw.stop()
        else:
            # Turn forward to 350 degrees
            while fw.angle() < 200:
                ev3.speaker.beep()
                fw.run(1100)
                bw.run(50)
            fw.stop()
            bw.stop()

    elif BRB == 1100:
        bw.run_angle(-1100,200)

    elif BRB == -1100:
        bw.run_angle(-1100,200)
    
    else:
        setBWSpeed(100, 500, balance)
        turnAngle3(10, 2.25, balance) 
        offTheWall = 1
    
    
    
    prev_distance = distance
    prev_Bpik = Bpik





            

    



