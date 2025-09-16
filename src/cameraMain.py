# --- Import required libraries ---
import sensor, pyb
from serialtalk.auto import SerialTalk# For serial communication with EV3
from machine import I2C               # I2C bus for sensors
from vl53l0x import VL53L0X           # Time-of-Flight distance sensor
from bno055 import BNO055             # IMU sensor (yaw, pitch, roll)
# --- Initialize I2C and sensors ---
i2c=I2C(4)       # Create I2C bus on channel 4
print(i2c.scan())# Print detected I2C devices for debugging
imu=BNO055(i2c)  # Initialize IMU
tof=VL53L0X(i2c) # Initialize ToF sensor
tof.set_measurement_timing_budget(40000)
tof.set_Vcsel_pulse_period(tof.vcsel_period_type[0],12)
tof.set_Vcsel_pulse_period(tof.vcsel_period_type[1],8)
# --- Setup SerialTalk for communication ---
st=SerialTalk()
cam_data=[0,0,0,0,0,0]
def cam():return cam_data# global cam_data
st.add_command(cam, "repr")
# --- Setup camera sensor ---
sensor.reset()                     # Initialize camera
sensor.set_pixformat(sensor.RGB565)# Use RGB color mode
sensor.set_framesize(sensor.QVGA)  # Resolution = 320x240
sensor.set_auto_whitebal(False)    # Disable auto white balance (important for color tracking)
sensor.set_auto_gain(False)        # Disable auto gain (stable brightness)
# Blink LED while initializing
led = pyb.LED(1)# Red LED
led.on()
sensor.skip_frames(time=2000)# Allow settings to take effect
led.off()
img_center=(160, 120)  # Image center pixel (for alignment)
img_roi=(45,15,230,100)# ROI for color block detection
def white_balance(img,img_debug,ROI_Y,ROI_H):
    #image filter to make find_blobs more accurate
    img_contrast1=img.copy()
    img_contrast1.gamma_corr(gamma=1.3,contrast=2.2)
    BLACK_THRESH=(0, 83, -128, 7, -128, 127)
    #find black blobs
    roi_leftP=(0,ROI_Y,img.width()//4,ROI_H)
    roi_rightP=(3*(img.width()//4),ROI_Y,img.width()//4,ROI_H)
    Lblack_blobs=img_contrast1.find_blobs([BLACK_THRESH],roi=roi_leftP,pixels_threshold=200)#find left side black blobs
    Rblack_blobs=img_contrast1.find_blobs([BLACK_THRESH],roi=roi_rightP,pixels_threshold=200)#find right side black blobs
    Lblack_pixels=sum([b.pixels() for b in Lblack_blobs])#sum of black pixels that are located in the left ROI
    Rblack_pixels=sum([b.pixels() for b in Rblack_blobs])#sum of black pixels that are located in the right ROI
    if True: #drawings for debugging
        img_debug.draw_rectangle(roi_leftP,color=(255,0,0))#draw red box on left
        img_debug.draw_rectangle(roi_rightP,color=(0,0,255))#draw blue box on right
        for blob in Lblack_blobs:
            img_debug.draw_rectangle(blob.rect(),color=(0,255,255))
        for blob in Rblack_blobs:
            img_debug.draw_rectangle(blob.rect(),color=(0,255,255))
    black_balance=Lblack_pixels-Rblack_pixels
    return int(black_balance)
def find_block(img,img_debug,distance_cap):
    img_contrast=img.copy()
    img_contrast.gamma_corr(gamma=1.9,contrast=1.1,brightness=-0.1)
    color="None"
    blob=None
    nearestRed=None
    nearestGreen=None
    red_val=0
    green_val=0
    blocks=0
    #thresholds in LAB format for color filtering
    threshold_red=(17, 60, 22, 54, -2, 35)#(23, 61, 16, 54, 0, 22)#(0, 47, 38, 54, 22, 55)#(0, 45, 23, 127, 5, 31)#(0, 32, 10, 127, 9, 127)#(0, 31, 10, 127, 9, 127)#(20, 45, 25, 40, 10, 127)#(0, 45, 25, 49, 13, 127)#(0, 46, 18, 49, 13, 127)#(0, 43, 9, 127, 4, 127)#(0, 35, 10, 127, -10, 127)#(0, 43, 10, 127, -12, 127)
    threshold_green=(29, 47, -46, -21, 10, 36)#(25, 50, -65, -25, 21, 79)#(25, 50, -65, -25, 21, 79)#(20, 45, -40, -15, 4, 57)#(21, 43, -128, -19, 16, 127)#(0, 100, -56, -25, 17, 50)#(28, 46, -56, -25, 17, 50)#(20, 45, -40, -15, 4, 57)#(20, 54, -40, -17, 0, 57)#(0, 100, -98, -25, -10, 127)
    #detect blobs with minimum size; no merging to prevent combining distant blocks
    img_debug.draw_rectangle(img_roi,color=(0,0,255))
    red=img_contrast.find_blobs([threshold_red],area_threshold=150,roi=img_roi,merge=True)
    green=img_contrast.find_blobs([threshold_green],area_threshold=150,roi=img_roi,merge=True)
    if red:# evaluate all red blobs
        for b in red:
            img_debug.draw_rectangle(b.rect(),color=(255,0,0))
            val=b.y()+b.h()
            if val>distance_cap:blocks+=1
            if val>red_val:# update nearest red
                nearestRed=b
                red_val=val
    if green:# valuate all green blobs
        for b in green:
            img_debug.draw_rectangle(b.rect(),color=(0,255,0))
            val=b.y()+b.h()                            #(2*b.pixels())-b.area()
            if val>distance_cap:blocks+=1
            if val>green_val:# update nearest green
                nearestGreen=b
                green_val=val
    if red_val==0 and green_val==0:return {"color":"None","blob":None,"blocks":blocks}
    # choose the blob with the higher score
    if nearestRed and nearestGreen:
        if red_val>=green_val:
            color="red"
            blob=nearestRed
        else:
            color="green"
            blob=nearestGreen
    elif nearestRed:
        color="red"
        blob=nearestRed
    elif nearestGreen:
        color="green"
        blob=nearestGreen
    else:
        color="None"
        blob=None
    return {"color":color,"blob":blob,"blocks":blocks}#return blob info and color if found
while True:
    #img_debug is for humans to interpret and contains drawings
    img_debug=sensor.snapshot()#capture raw image
    #img_debug.gamma_corr(gamma=1.9, contrast=1.1,brightness=-0.1)#debug find_block()
    #img_debug.gamma_corr(gamma=1.3,contrast=2.2)#debug white_balance()
    #img is for the robot to interpret and has no drawings for clean processing
    img=img_debug.copy()#clean image
    img_debug.draw_circle(img_center[0],img_center[1],1,color=(0,0,255))#cursor
    error=0
    distance=999
    block_x=999
    color=0
    white_balance=white_balance(img,img_debug,45,155)
    block=find_block(img,img_debug,30)
    if block["color"]!="None":
        # draw dot at blob center
        img_debug.draw_circle(block["blob"].cx(),block["blob"].cy(),5,color=(0,0,255),fill=True)
        distance=240-(block["blob"][1]+block["blob"][3])#works even when there is occlusion
        block_x=block["blob"].cx()-170
    if block["color"]=="red":color=1
    if block["color"]=="green":color=-1
    yaw,roll,pitch=imu.euler()
    cam_data=[white_balance,color,distance,tof.ping(),round(yaw),block_x]
    print(cam_data)
    st.process()