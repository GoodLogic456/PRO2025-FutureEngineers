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
    img_contrast=img.copy()
    img_contrast.gamma_corr(gamma=1.3,contrast=2.2)
    roi_left=(0,ROI_Y,img.width()//2,ROI_H)
    roi_right=(img.width()//2,ROI_Y,img.width()//2,ROI_H)
    BLACK_THRESH=(0, 93, -128, 0, -7, 36)
    Lblack_blobs=img_contrast.find_blobs([BLACK_THRESH],roi=roi_left,pixels_threshold=200)
    Rblack_blobs=img_contrast.find_blobs([BLACK_THRESH],roi=roi_right,pixels_threshold=200)
    Lblack_pixels=sum([b.pixels() for b in Lblack_blobs])
    Rblack_pixels=sum([b.pixels() for b in Rblack_blobs])
    if True: #drawings for debugging
        img_debug.draw_rectangle(roi_left,color=(255,0,0))#draw red box on left
        img_debug.draw_rectangle(roi_right,color=(0,0,255))#draw blue box on right
        for blob in Lblack_blobs:
            img_debug.draw_rectangle(blob.rect(),color=(0,255,0))
        for blob in Rblack_blobs:
            img_debug.draw_rectangle(blob.rect(),color=(0,255,0))
    black_balance=max(-1100,min(1100,(Lblack_pixels-Rblack_pixels)/8))
    return int(black_balance)
def target_point(img_debug,block,color,k):
    offset_scaler=int(k*(block.y()+block.h()))
    if color=="red":offset_x=(block.x()+block.w())+offset_scaler#right side-offset
    if color=="green":offset_x=block.x()-offset_scaler#left side-offset
    draw_x=max(0,min(319,offset_x))
    img_debug.draw_circle(draw_x,block.cy(),3,color=(0,255,255),fill=True)#target for cursor
    img_debug.draw_arrow(img_center[0],img_center[1],draw_x,img_center[1],color=(0,0,255))
    error=offset_x-img_center[0]
    img_debug.draw_string(img_center[0]-30,img_center[1]-10,"Err: "+str(error),color=(255,255,255),scale=1)
    return error
def find_block(img,img_debug):
    img_contrast=img.copy()
    img_contrast.gamma_corr(gamma=1.9, contrast=1.1,brightness=-0.1)
    color="None"
    blob=None
    nearestRed=None
    nearestGreen=None
    red_val=0
    green_val=0
    #thresholds in LAB format for color filtering
    threshold_red=(0, 40, 5, 127, -10, 55)
    threshold_green=(25, 51, -85, -21, -7, 73)
    #detect blobs with minimum size; no merging to prevent combining distant blocks
    red=img_contrast.find_blobs([threshold_red],pixels_threshold=100,merge=False)
    green=img_contrast.find_blobs([threshold_green],pixels_threshold=100,merge=False)
    if not red and not green:return {"color":"None","blob":None}
    if red:# evaluate all red blobs
        for b in red:
            img_debug.draw_rectangle(b.rect(),color=(255,0,0))
            val=b.y()+b.h()
            if val>red_val:# update nearest red
                nearestRed=b
                red_val=val
    if green:# evaluate all green blobs
        for b in green:
            img_debug.draw_rectangle(b.rect(),color=(0,255,0))
            val=b.y()+b.h()                            #(2*b.pixels())-b.area()
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
    return {"color":color,"blob":blob}#return blob info and color if found
snap_distance=None
peak_distance=0
while True:
    clock.tick()
    #img_debug is for humans to interpret and contains drawings
    img_debug=sensor.snapshot()#capture raw image
    img_debug=img_debug.lens_corr(strength=1.53,x_corr=-0.04)#correct lens distortion
    #img_debug.gamma_corr(gamma=1.9, contrast=1.1,brightness=-0.1)#debug
    #img is for the robot to interpret and has no drawings for clean processing
    img=img_debug.copy()#clean image
    img_debug.draw_circle(img_center[0],img_center[1],1,color=(0,0,255))#cursor
    white_balance=check_color_balance(img,img_debug,80,160)
    block=find_block(img,img_debug)
    distance=None
    error=None
    if block["color"]!="None":
        # draw dot at blob center
        img_debug.draw_circle(block["blob"].cx(),block["blob"].cy(),5,color=(0,0,255),fill=True)
        distance=240-(block["blob"][1]+block["blob"][3])#works even when there is occlusion
        snap_distance=distance
        if distance>peak_distance:peak_distance=distance
        error=target_point(img_debug,block["blob"],block["color"],0.85)
    #print(time.time(),distance,snap_distance,peak_distance,error,white_balance)

    cam_data=[white_balance,error,distance]
    print(cam_data)
    st.process()



