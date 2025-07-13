#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
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


# Rultra.distance() = 0
# Lultra.distance() = 0
# balance = balance * 21.42857142857143
# turnangle,distance,balance = data

# def turnAngle(kp, kd, speed):
#     p = 0
#     d = 0
#     rot = 0
#     mRot = fw.angle()
#     print("mRot:" + str(mRot))
    
#     previous_balance = 0
    
#     p = kp * balance
#     d = kd * (balance - previous_balance)  
#     fwspeed = p + d
#     previous_balance = balance
    
#     if balance > 0:
#         rot = (max_turnR/1100) * balance 
#     else:
#         rot = (max_turnL/-1100) * balance 
    
#     print("rot:" + str(rot))
#     print("balance:" + str(balance))
    
#     if rot > 0:
#         if mRot > rot:  #switch steering direction
#             fwspeed = -1 * fwspeed - 200
#             ev3.speaker.beep()
#     # elif rot == 0:
#     #     if mRot > rot:
#     #         fwspeed = -1 * fwspeed - 200
#     #         ev3.speaker.beep()
#     #     elif mRot < rot:
#     #         fwspeed = fwspeed + 200

#     else:
#         if mRot < rot:
#             fwspeed = -1 * fwspeed - 200
#             ev3.speaker.beep()
#         # else:
#         #     fwspeed = fwspeed + 200

#     print("fwspeed:" + str(fwspeed))
#     print(" ")
#     #balance -> rot
#     #mrot going to rot
#     #rot 250, 240
#     # if distance >= 110:
#     #     speed = speed

#     # if distance < 110:
#     #     speed = 100
    
#     # if balance == 0.0 and fw.angle() == 0:   
#     #     fw.stop()
#     #     bw.run(speed)

#     fwspeed = int(fwspeed)

#     if balance != 0.0 :
#         fw.run(fwspeed)
#         # bw.run(speed - 80)

#     # if fw.angle() >= max_turnR:
#     #     fw.stop()
#     #     # bw.run(speed)

#     # if fw.angle() < max_turnL:   #negative
#     #     fw.stop()
#         # bw.run(speed)

# def PIDultra(kp, kd):
#     p = 0
#     d = 0

#     global fwspeed, prev_error, error

#     ultraDistL = Lultra.distance()
#     ultraDistR = Rultra.distance()

    
#     if ultraDistL < 150:
#         error = 150 - ultraDistL 
#         p = kp * error
#         d = kd * (error - prev_error)
#         fwspeed = int(p + d)
#         prev_error = error
        
        
#     elif ultraDistL > 150:
#         ev3.speaker.beep()
#         error = 150 - ultraDistL
#         p = kp * error
#         d = kd * (error - prev_error)
#         fwspeed = int(p + d)
#         prev_error = error
        
        
#     # elif ultraDistR < 150:  
#     #     error = ultraDistR - 150
#     #     p = kp * error
#     #     d = kd * (error - prev_error)
#     #     fwspeed = int(p + d)
#     #     prev_error = error
        
        
#     # elif ultraDistR > 150: 
#     #     # ev3.speaker.beep()
 
#     #     error = ultraDistR - 150
#     #     p = kp * error
#     #     d = kd * (error - prev_error)
#     #     fwspeed = int(p + d)
#     #     prev_error = error
        
        
#     # if fw.angle() >= max_turnR or fw.angle() <= max_turnL:
#     #     fw.stop()
#     # else:
#     #     fw.run(fwspeed)

#     fw.run(fwspeed)

#     print("error:" + str(error))
#     print("fwspeed:" + str(fwspeed))
#     print("ultraL:" + str(ultraDistL))
#     print("P:" + str(p))
#     print("D:" + str(d))
#     print(" ")



def turnAngle(kp, kd, balanceRB):
    p = 0
    d = 0
    rot = 0
    prev_error = 0

    mRot = fw.angle()   # current motor rotation
    
    if balance > 0:
        rot = (max_turnR/1100) * balanceRB
    else:
        rot = (max_turnL/-1100) * balanceRB * 1.25    # *1.35
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



    print("error:" + str(error))


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


######################3
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
   # while True:
    #     status, data = st.call('cam')
    #     balance, turnangle = data
    #     print(balance, turnangle)
    
    #     if balance != 0.0:
    #         turnAngle(0.5, 0.1, 500)
    #     else:
    #         fw.stop()
    #         bw.run(500)

# ###############################openchallenge
while run <= 11:
    
    status, data = st.call('cam')
    balance, balanceRB, distance = data
 
    color = rgb_to_color(sensor.rgb())
    print("Direction: " + str(direction))

    Lultra_distance = Lultra.distance()
    Rultra_distance = Rultra.distance()

    # print("color: " + str(color))
    # print("direction: " + str(direction))
    # print("balance: " + str(balance))
    # print("Lultra: " + str(Lultra_distance))
    # print("Rultra: " + str(Rultra_distance))
    # print("balance: " + str(balance))
    
# # # ###print color
    # rgb = sensor.rgb()
    # r, g, b = rgb
    # ev3.screen.clear()
    # ev3.screen.print("R:", r)
    # ev3.screen.print("G:", g)
    # ev3.screen.print("B:", b)

    # print("R: " + str(r))
    # print("G: " + str(g))
    # print("B: " + str(b))
    # print(" ")


    # print("R:", r, "G:", g, "B:", b)

    # wait(500)

    # if direction == 0:
    #     if color == -1: #blue
    #         direction = -1

    #     if color == 1: #orange
    #         direction = 1

###########    
    if Lultra_distance < Rultra_distance:
        LeftWins = True
    else:
        LeftWins = False


    if (Lultra_distance + Rultra_distance) < 600:
    # if (Lultra.distance() + Rultra.distance()) < 600 and balance == 0.0:
        setBWSpeed(150, 400, balance)
        turnAngle2(130.0, 2.25, balance)

    elif balance <= -1000 or balance >= 1000:  # severe case
        setBWSpeed(150, 150, balance)
        turnAngle2(100.0, 2.25, balance)

##########################
    # elif Lultra_distance < 250 and LeftWins:
    #     ev3.speaker.beep()
    #     PIDultraL(5, 2.5, 0.01, 200, 300, 180)
    #     bw.run(400)  # stop moving backwards

    # elif Rultra_distance < 250 and not LeftWins:        
    #     ev3.speaker.beep()
    #     PIDultraR(5, 2.5, 0.01, 200, 300, 180)
    #     bw.run(400)  # stop moving backward
#####################################

    elif fw.angle() < 200 and fw.angle() > -200: # fairly straight
        setBWSpeed(150, 600, balance)
        turnAngle2(120.0, 2.25, balance)
    
    else: # normal camera mode
        setBWSpeed(150, 400, balance)
        turnAngle2(120.0, 2.25, balance)

    
    if rgb_to_color(sensor.rgb()) == 1: # orange

        if direction == 0 or direction == 1: # no direction or clockwise
            # ev3.speaker.beep()
            while rgb_to_color(sensor.rgb()) > -1:  # not yet blue (from orange)
                if fw.angle() <= 100 and Rultra_distance > 150:
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
                if fw.angle() >= -100 and Lultra_distance > 150:
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



    if direction == -1:
        if Lultra_distance < 250 :
            ev3.speaker.beep()
            PIDultraL(10, 100, 0.001, 200, 300, 250)
            bw.run(400)  # stop moving backwards

    if direction == 1:
        if Rultra_distance < 250 :        
            ev3.speaker.beep()
            PIDultraR(10, 100, 0.001, 200, 300, 250)
            bw.run(400)  # stop moving backward

    if direction == 0:
        if Lultra_distance < 250 :
            ev3.speaker.beep()
            PIDultraL(10, 100, 0.001, 200, 300, 250)
            bw.run(400)  # stop moving backwards

        if Rultra_distance < 250 :        
            ev3.speaker.beep()
            PIDultraR(10, 100, 0.001, 200, 300, 250)
            bw.run(400)  # stop moving backward

        run = 0

    
    # if direction == 0 and rgb_to_color(sensor.rgb()) == 1:  # orange:  # no direction yet
    #     # if rgb_to_color(sensor.rgb()) == 1:  # orange
    #     ev3.speaker.beep()
    #     while rgb_to_color(sensor.rgb()) != -1:  # not yet blue (from orange)
    #         if fw.angle() <= 300 or Rultra_distance < 180:
    #             fw.run(500)     
    #             bw.run(400)    

    #     # while rgb_to_color(sensor.rgb()) == -1:
    #     else:
    #         while fw.angle() >= 0:
    #             bw.run(100)
    #             # ev3.speaker.beep()
    #             # bw.stop()
    #             fw.run(-900)

    #     run = run + 1

    #     direction = 1

    # if direction == 0 and rgb_to_color(sensor.rgb()) == -1: # blue  # no direction yet
    #     ev3.speaker.beep()
    #     while rgb_to_color(sensor.rgb()) != 1:  # not yet orange (from blue)
    #         if fw.angle() <= -400 or Lultra_distance < 180:
    #             bw.run(250)
    #             fw.run(-1100)        

    #     else:
    #         while fw.angle() <= 0:
    #             bw.run(100)
    #             ev3.speaker.beep()
    #             # bw.stop()
    #             fw.run(900)

    #     run = run + 1

    #     direction = -1


    
    # if direction == 1 and rgb_to_color(sensor.rgb()) == 1:  #clockwise (orange first)
    #     # if rgb_to_color(sensor.rgb()) == 1:
    #         ev3.speaker.beep()
    #         while rgb_to_color(sensor.rgb()) != -1:  # not yet blue (from orange)
    #             if fw.angle() <= 300 or Rultra_distance < 180:
    #                 fw.run(500)     
    #                 bw.run(400)    

    #         # while rgb_to_color(sensor.rgb()) == -1:
    #         else:
    #             while fw.angle() >= 0:
    #                 bw.run(100)
    #                 # ev3.speaker.beep()
    #                 # bw.stop()
    #                 fw.run(-900)

    #         run = run + 1


    # if direction == -1 and rgb_to_color(sensor.rgb()) == -1: # counter-clockwise (blue first)
    #     # if rgb_to_color(sensor.rgb()) == -1: # blue
    #         ev3.speaker.beep()
    #         while rgb_to_color(sensor.rgb()) != 1:  # not yet orange (from blue)
    #             if fw.angle() <= -400 or Lultra_distance < 180:
    #                 bw.run(250)
    #                 fw.run(-1100)        

    #         else:
    #             while fw.angle() <= 0:
    #                 bw.run(100)
    #                 # ev3.speaker.beep()
    #                 # bw.stop()
    #                 fw.run(900)

    #         run = run + 1

if run > 11:
    
    bw.reset_angle(0)
    while bw.angle() < 1200:
        status, data = st.call('cam')
        balance, turnangle, distance = data
        
        setBWSpeed(150, 1100, balance)
        turnAngle2(100.0, 2.25, balance)