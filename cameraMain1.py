import sensor,time,pyb,math
from serialtalk.auto import SerialTalk
st = SerialTalk()


#gnd,vcc
# , ,rx,tx
#import sensor,image,time
#threshHold(L min,L max,A min,A max,B min,B max)
threshHold_red    =(20,100,  15,127,  15,127)
threshHold_green  =( 0,100,-128,-34,-128,127)
threshHold_blue   =(25, 46,-128, 21,-128,-42)
threshHold_orange =(30, 80,  25,127,  25,127)
threshHold_magenta=(30,100,  50, 90, -15, 20)
#width,hight
robotSize=(100,50)
# z, x, y
blockSizes=[
    (10,10,10),#red and green 10cm in actual equal to screen 0.058cm
    (200,20,100)#magenta
]
#robot angle=use orange and blue lines as a guide for robot angle in terms of the board

#find blobs

#make rectangle of the blob
#distance=[(normal size of block)*(constant to regulate to mm or inches)]/(hight and width size of rectangle)
#print(distance)

#perfect track

#line

#curve

#speed=distance, offset from perfect track
#wheel angle when moving=color of block,robot angle,distance,speed
sumError=0
prevError=0
sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)  # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)  # Set frame size to QVGA (320x240)
sensor.set_vflip(True) # Flips the image vertically
sensor.set_hmirror(True) # Mirrors the image horizontally
sensor.skip_frames(time=2000)  # Wait for settings take effect.
clock = time.clock()  # Create a clock object to track the FPS.
def cal_hyp(adjacent,opposite):return math.sqrt(adjacent**2+opposite**2)
def cal_adj(opposite,hypotenuse):return math.sqrt(hypotenuse**2-opposite**2)


offsetSpeed=0
cam_data=[0,0,0,0,0]
def cam():return cam_data# global cam_data
st.add_command(cam, "repr")


while True:
    clock.tick()  # Update the FPS clock.
    img=sensor.snapshot()  # Take a picture and return the image.
    blobs=img.find_blobs([threshHold_red],area_threshold=20)
    # print(len(blobs))
    if len(blobs)>=1:
        blob=blobs[0]
        aveHW=(blob[2]+blob[3])/2   #average of the hight and width
        aveBlock=(blockSizes[0][1]+blockSizes[0][2])/2
        distance=(aveBlock*290)/aveHW #block aveHW is inversly proportional to distance
        #print(" distance: "+str(distance))
        #print(" aveHW: "+str(aveHW))
        img.draw_circle(int(img.width()/2),int(img.height()/2),1,fill=True)
        #x,y,w,h,cx,cy
        img.draw_rectangle(blob[0:4],color=(0,255,0))
        img.draw_cross(blob[5],blob[6],color=(0,255,0))
        #imaginary rectangle
        offsetA=(int(img.width()/2)-blob[5])*-1
        #distance=10#dummy distance
        if offsetA==0:offsetA=1
        color="red"
        if color=="red":
            img.draw_rectangle(blob[0]+blob[2],(blob[1]+blob[3])-robotSize[1],robotSize[0],robotSize[1],color=(0,255,0))
            img.draw_cross(blob[0]+blob[2]+int(robotSize[0]/2),(blob[1]+blob[3])-int(robotSize[1]/2),color=(0,255,0))
            offsetI=(int(img.width()/2)-(blob[0]+blob[2]+int(robotSize[0]/2)))
            offsetA=offsetA*-1
            botDirectionAngle=math.degrees(math.atan2(offsetA,distance))#relative to the block
            botDirectionLength=cal_hyp(distance,offsetA)#relative to the block
            #Atriangle=(distance,offsetA,botDirectionLength)a,o,h
            #Itriangle=(botDirectionLength,abs(offsetI),cal_hyp(botDirectionLength,abs(offsetI)))a,o,h
            if botDirectionAngle<0:botDirectionLength=(-abs(botDirectionLength))+(int(robotSize[0]/2)+(blob[2]/2))-6
            turnAngle=math.degrees(math.atan2(offsetI,botDirectionLength))/2.5
        if color=="green":
            img.draw_rectangle(blob[0]-robotSize[0],(blob[1]+blob[3])-robotSize[1],robotSize[0],robotSize[1],color=(0,255,0))
            img.draw_cross(blob[0]-int(robotSize[0]/2),(blob[1]+blob[3])-int(robotSize[1]/2),color=(0,255,0))
            offsetI=(int(img.width()/2)-(blob[0]-int(robotSize[0]/2)))
            botDirectionAngle=math.degrees(math.atan2(offsetA,distance))#relative to the block
            botDirectionLength=cal_hyp(distance,offsetA)#relative to the block
            #Atriangle=(distance,offsetA,botDirectionLength)a,o,h
            #Itriangle=(botDirectionLength,abs(offsetI),cal_hyp(botDirectionLength,abs(offsetI)))a,o,h
            if botDirectionAngle<0:botDirectionLength=(-abs(botDirectionLength))+int(robotSize[0]/2)+6
            turnAngle=math.degrees(math.atan2(offsetI,botDirectionLength))/2.5
        #print("offsetA: "+str(offsetA))
        #print("offsetI: "+str(offsetI))
        #print(botDirectionLength)
        # print(turnAngle)
        # if turnAngle<15 and turnAngle>-15:speed=100
        # else:
        #     if turnAngle<60 and turnAngle>-60:speed=75
        #     else:speed=50

        #error=(img.width()/2)-(blob[0]+blob[2]+(robotSize[0]/2))

        #interpret values to send to the robot
        kp=1*21.42857142857143
        ki=0
        kd=0
        speed=50
        error=turnAngle-0
        sumError+=error
        compensation=(kp*error)+(ki*sumError)+(kd*(error-prevError))
        prevError=error
        speedPID=speed+abs(compensation)
        print(speedPID)

        compensation=(1*error)+(0*sumError)+(0*(error-prevError))
        anglePID=-(turnAngle-(compensation))
        print(anglePID)






        # P=(1*error)#ratio of angle to gear rotation
        # I=(0*sumError)
        # D=(0*(error-prevError))
        #compensation=P+I+D

        #if turnAngle>30:offsetSpeed+=30
        #if turnAngle<-30:offsetSpeed-=30
        cam_data=[speedPID,anglePID,turnAngle,botDirectionAngle,distance]
        print(cam_data)
        st.process()
        pyb.delay(50)


























    #print(clock.fps())  # Note: OpenMV Cam runs about half as fast when connected
    # to the IDE. The FPS should increase once disconnected.
