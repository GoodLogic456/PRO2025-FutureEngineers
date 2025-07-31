import sensor,time,pyb,math
from serialtalk.auto import SerialTalk
st=SerialTalk()
cam_data=[0,0,0]
def cam():return cam_data# global cam_data
st.add_command(cam, "repr")

sensor.reset()#initialize sensor
sensor.set_pixformat(sensor.RGB565)#use color
sensor.set_framesize(sensor.QVGA)#320x240 resolution
sensor.set_vflip(True)#vertical flip to match world
sensor.set_hmirror(True)#horizontal mirror to align view
sensor.set_auto_whitebal(False)#disable auto white balance
sensor.set_auto_gain(False)#disable auto gain
sensor.skip_frames(time=2000)#wait for config to apply
clock=time.clock()
img_center=(160,120)
def check_color_balance(img,img_debug,ROI_Y,ROI_H):
    #image filter to make find_blobs more accurate
    img_contrast1=img.copy()
    img_contrast1.gamma_corr(gamma=1.3,contrast=2.2)
    img_contrast2=img.copy()
    img_contrast2.gamma_corr(gamma=1.9, contrast=1.1,brightness=-0.1)#(gamma=2.0,contrast=1.5,brightness=-0.15)
    MAGENTA_THRESH=(0, 100, 17, 64, -22, 6)#(46, 80, 25, 127, -30, 14)#(60, 100, 25, 127, -128, 10)#(0, 100, 19, 127, -34, 20)
    BLACK_THRESH=(0, 93, -128, 0, -7, 36)
    #find black blobs
    roi_left=(0,ROI_Y,img.width()//2,ROI_H)
    roi_right=(img.width()//2,ROI_Y,img.width()//2,ROI_H)
    Lblack_blobs=img_contrast1.find_blobs([BLACK_THRESH],roi=roi_left,pixels_threshold=200)#find left side black blobs
    Rblack_blobs=img_contrast1.find_blobs([BLACK_THRESH],roi=roi_right,pixels_threshold=200)#find right side black blobs
    Lblack_pixels=sum([b.pixels() for b in Lblack_blobs])#sum of black pixels that are located in the left ROI
    Rblack_pixels=sum([b.pixels() for b in Rblack_blobs])#sum of black pixels that are located in the right ROI
    Lmagenta_blobs=img_contrast2.find_blobs([MAGENTA_THRESH],roi=roi_left,pixels_threshold=400)#find left side magenta blobs
    Rmagenta_blobs=img_contrast2.find_blobs([MAGENTA_THRESH],roi=roi_right,pixels_threshold=400)#find right side magenta blobs
    Lmagenta_pixels=sum([b.pixels() for b in Lmagenta_blobs])#sum of magenta pixels that are located in the left ROI
    Rmagenta_pixels=sum([b.pixels() for b in Rmagenta_blobs])#sum of magenta pixels that are located in the right ROI
    blackH_balance=99999
    #wall_distance=99999
    LblackH_blobs=img_contrast1.find_blobs([BLACK_THRESH],roi=(0,40,img.width()//2,200),pixels_threshold=200,merge=True)#find left side black blobs
    RblackH_blobs=img_contrast1.find_blobs([BLACK_THRESH],roi=(img.width()//2,40,img.width()//2,200),pixels_threshold=200,merge=True)#find right side black blobs
    if len(LblackH_blobs)==1 and len(RblackH_blobs)==1:
        blackH_balance=(RblackH_blobs[0].y()+RblackH_blobs[0].h())-(LblackH_blobs[0].y()+LblackH_blobs[0].h())
        #if abs(blackH_balance)<15:
        #wall_distance=240-(((RblackH_blobs[0].y()+RblackH_blobs[0].h())+(LblackH_blobs[0].y()+LblackH_blobs[0].h()))/2)
    if True: #drawings for debugging
        img_debug.draw_rectangle(roi_left,color=(255,0,0))#draw red box on left
        img_debug.draw_rectangle(roi_right,color=(0,0,255))#draw blue box on right
        for blob in Lblack_blobs:
            img_debug.draw_rectangle(blob.rect(),color=(0,255,0))
        for blob in Rblack_blobs:
            img_debug.draw_rectangle(blob.rect(),color=(0,255,0))
        for blob in Lmagenta_blobs:
            img_debug.draw_rectangle(blob.rect(),color=(255,100,255))
        for blob in Rmagenta_blobs:
            img_debug.draw_rectangle(blob.rect(),color=(255,100,255))
    black_balance=max(-1100,min(1100,(Lblack_pixels-Rblack_pixels)/8))
    magenta_balance=max(-1100,min(1100,(Lmagenta_pixels-Rmagenta_pixels)/6))
    return int(black_balance),int(magenta_balance),int((Lblack_pixels+Rblack_pixels)/1000),int(blackH_balance)
def target_point(img_debug,block,color,k):
    offset_scaler=int(k*(block.y()+block.h()))
    if color=="red":offset_x=(block.x()+block.w())+offset_scaler#right side-offset
    if color=="green":offset_x=block.x()-offset_scaler#left side-offset
    draw_x=max(0,min(319,offset_x))
    img_debug.draw_circle(draw_x,block.cy(),3,color=(150,255,150),fill=True)#target for cursor
    img_debug.draw_arrow(img_center[0],img_center[1],draw_x,img_center[1],color=(0,0,255))
    error=offset_x-img_center[0]
    if color=="red" and error<0:error=0
    if color=="green" and error>0:error=0
    img_debug.draw_string(img_center[0]-30,img_center[1]-10,"Err: "+str(error),color=(255,255,255),scale=1)
    return error
def find_block(img,img_debug,distance_cap):
    img_contrast=img.copy()
    img_contrast.gamma_corr(gamma=1.9, contrast=1.1,brightness=-0.1)
    color="None"
    blob=None
    nearestRed=None
    nearestGreen=None
    red_val=0
    green_val=0
    blocks=0
    #thresholds in LAB format for color filtering
    threshold_red=(0, 45, 25, 49, 13, 127)#(0, 46, 18, 49, 13, 127)#(0, 43, 9, 127, 4, 127)#(0, 35, 10, 127, -10, 127)#(0, 43, 10, 127, -12, 127)
    threshold_green=(0, 100, -98, -25, -10, 127)
    #detect blobs with minimum size; no merging to prevent combining distant blocks
    red=img_contrast.find_blobs([threshold_red],pixels_threshold=50,merge=True)
    green=img_contrast.find_blobs([threshold_green],pixels_threshold=50,merge=True)
    if not red and not green:return {"color":"None","blob":None,"blocks":blocks}
    if red:# evaluate all red blobs
        for b in red:
            img_debug.draw_rectangle(b.rect(),color=(255,0,0))
            val=b.y()+b.h()
            if val>distance_cap:blocks+=1
            if val>red_val:# update nearest red
                nearestRed=b
                red_val=val
    if green:# evaluate all green blobs
        for b in green:
            img_debug.draw_rectangle(b.rect(),color=(0,255,0))
            val=b.y()+b.h()                            #(2*b.pixels())-b.area()
            if val>distance_cap:blocks+=1
            if val>green_val:# update nearest green
                nearestGreen=b
                green_val=val
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

    
snap_distance=None
peak_distance=0
while True:
    clock.tick()
    #img_debug is for humans to interpret and contains drawings
    img_debug=sensor.snapshot()#capture raw image
    img_debug=img_debug.lens_corr(strength=1.53,x_corr=-0.04)#correct lens distortion
    #img_debug.gamma_corr(gamma=1.3,contrast=2.2)#debug check_color_balance(black)
    #img_debug.gamma_corr(gamma=2.0, contrast=1.5,brightness=-0.15)#debug check_color_balance(magenta)
    #img_debug.gamma_corr(gamma=1.9, contrast=1.1,brightness=-0.1)#debug find_block()
    #img is for the robot to interpret and has no drawings for clean processing
    img=img_debug.copy()#clean image
    img_debug.draw_circle(img_center[0],img_center[1],1,color=(0,0,255))#cursor
    white_balance=0
    error=0
    distance=0
    block_pixels=0
    #sss=False
    white_balance,magenta_balance,black_pixels,blackH_balance=check_color_balance(img,img_debug,80,160)
    #white_balance,magenta_balance,black_pixels,blackH_balance=check_color_balance(img,img_debug,130,170)
    block=find_block(img,img_debug,30)
    if block["color"]!="None":
        # draw dot at blob center
        img_debug.draw_circle(block["blob"].cx(),block["blob"].cy(),5,color=(0,0,255),fill=True)
        distance=240-(block["blob"][1]+block["blob"][3])#works even when there is occlusion
        snap_distance=distance
        if distance>peak_distance:peak_distance=distance
        error=target_point(img_debug,block["blob"],block["color"],1.35)#1.35
        block_pixels=block["blob"].pixels()
    ORANGE_THRESH=(20, 100, 10, 66, 18, 127)
    orange_blobs=img.find_blobs([ORANGE_THRESH],pixels_threshold=300)
    orange_distance=0
    if len(orange_blobs)==1:
        #img_debug.draw_rectangle(orange_blobs[0].rect(),color=(255,255,0))
        orange_distance=240-orange_blobs[0].cy()
        #img_debug.draw_line(orange_blobs[0].x(),orange_blobs[0].y(),orange_blobs[0].x()+orange_blobs[0].w(),orange_blobs[0].y()+orange_blobs[0].h())
        #if orange_blobs[0].y()+orange_blobs[0].h()==240:
            #if orange_blobs[0].x()+orange_blobs[0].w()==320:
                #sss=True
    color=0
    if block["color"]=="red":color=1
    if block["color"]=="green":color=-1
    cam_data=[white_balance,error,distance,abs(magenta_balance),block_pixels,black_pixels,blackH_balance,color]
    print(cam_data)
    st.process()



