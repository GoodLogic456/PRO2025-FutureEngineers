### Introduction
<img align="right" width="300" height="auto" src="https://github.com/user-attachments/assets/a881d5a4-3cb7-4174-ad33-f6f3b2d881e3">

This is the Engineering Documentation of **TEAM GRACE CHRISTIAN COLLEGE (GCC)** for **PRO Future Engineers 2025**

This repository contains engineering materials of a self-driven vehicle's model. It also contains relevant information such as [wiring diagrams](https://github.com/GoodLogic456/PRO2025-FutureEngineers/tree/main/schemes), robot [images](https://github.com/GoodLogic456/PRO2025-FutureEngineers/tree/main/v-photos) and [videos](https://github.com/GoodLogic456/PRO2025-FutureEngineers/blob/main/video/video.md), code snippets, technical explanations, and [Engineering Journal](https://docs.google.com/spreadsheets/d/1C1dG2nG-GNh3qbOAAdMjjLXDwYX-iINyFB89ZOEz_A0/edit?usp=sharing).

### TABLE OF CONTENTS
- [Mobility Management](#-Mobility-Management-)
- [Power and Sense Management](#-Power-and-Sense-Management-)
- [Obstacle Management](#-obstacle-management-)


# 🛑 MOBILITY MANAGEMENT 🛑
## Motor and Steering System
<img align="right" width="300" height="auto" src="https://github.com/user-attachments/assets/da1f7ea8-02c0-4a99-b11e-f5ee9c429ad7">The robot was built using the Lego EV3 Mindstorm Robot System. The robot moves using one **(1) EV3 large motor** at the back and one **(1) EV3 medium motor** at the front. The large motor pushes the robot forward and backward, while the medium motor turns a gear rack to steer the front wheel. As required by the rules, the robot employs an **Ackermann** steering system, just like a real car, so it can turn smoothly. The steering system is powered by a worm gear, giving it much power to turn. The robot is also guided by an **OpenMV Camera**, **Time-of-Flight sensor**, **color sensor**,  and  **gyro sensor**. 

The robot uses **Proportional-Derivative** control to make steering more accurate. By continuously calculating the error  `balance`  and its rate of change `d_error` , the robot adjusts its steering angle using a control signal derived from the proportional  `kp`  and derivative  `kd`  gains. The front wheel motor  uses this signal to steer toward the correct heading, while the back wheel motor adjusts its speed to maintain smooth movement without overpowering the turn. The robot uses PD control to respond not only to how far off-center it is, but also to how quickly that error is changing, resulting in more precise and stable steering behavior.



```
def PID(kp, kd, balance):

    global prev_error, error
    

    # Calculate the error
    error = balance

    # Calculate the derivative of the error
    d_error = error - prev_error

    # Calculate the control signal
    control_signal = kp * error + kd * d_error

    # Update previous error
    prev_error = error

    # Set motor speeds based on control signal
    fwspeed = control_signal
    # bwspeed = 100 - control_signal

    # Limit motor speeds to max values
    fwspeed = max(-1000, min(1000, fwspeed))
    # bwspeed = max(-1000, min(1000, bwspeed))

    if fwspeed > 0:
        fwspeed = 1000
    elif fwspeed < 0:
        fwspeed = -1000
    else:
        fwspeed = 1
    
    # if control_signal == 0:
    #     control_signal = 1

    # print("Error:", left-right, "control_signal:", control_signal, " fwspeed:", fwspeed)

    # Set motor speeds
    control_signal = int(control_signal)

    if control_signal > max_turnR:
        control_signal = max_turnR
    elif control_signal < max_turnL:
        control_signal = max_turnL
    
    fw.run_target(fwspeed, control_signal, wait=False)

    if control_signal > maxPower or control_signal < -maxPower:
        control_signal = maxPower
    
    bwspeed = maxPower - abs(control_signal)
    
    if bwspeed < minPower:
        bwspeed = minPower
    
    bw.run(bwspeed)
    print(" bwspeed:", bwspeed ,"control_signal:", control_signal)
```

## Wheel and Motor Selection for Stability
<img align="right" width="300" height="auto" src="https://github.com/user-attachments/assets/4d7e9bac-02fc-4d3b-9a42-662e360fbae3">The motors were picked because they are strong, reliable, and have enough turning power (torque) to move the robot smoothly on the track. The robot also uses hard wheels with flat edges in both the front and back. This design gives the robot better grip and control when turning. Since the wheels are the same size and shape, the robot stays level, which helps the color sensor stay flat so it can read the floor properly. 


## Chassis Build and Balance
The robot’s body, or chassis, is made of LEGO parts. It is strong but not heavy, and it holds all the parts in place. The motors, EV3 intelligent brick, camera, and sensors are mounted carefully to keep the robot balanced. The weight is spread out evenly to stop the robot from tipping during sharp turns. The final design makes the robot steady, smooth, and ready to finish the course without losing control.

## 💡 <mark> How to Improve Mobility Management</mark>
One way to improve the robot is by **adding a rear camera**. This camera can help the robot see what is behind it, which is useful when it needs to move backward or park between objects. It can also help the robot avoid bumping into walls or blocks while reversing. With a rear camera, the robot can better understand its surroundings and move more safely and accurately. This helps the robot do its tasks more smoothly.

Making the robot **smaller and lighter** helps it turn faster and easier. A lighter robot is easier to steer because the motor doesn’t need to work as hard to move it. When the robot is smaller, it can make sharp turns more easily and quickly. This also puts less pressure on the steering parts, so they can move faster. Using compact designs, placing parts closer together, and choosing lightweight materials all help make the robot quicker and easier to steer.

# 🛑 POWER AND SENSE MANAGEMENT 🛑

## Bill of Material (BOM)
| **Component**              | **QTY**    |
|---------------------------|------------|
| EV3 Intelligent Brick     | 1          |
| EV3 Large Motor           | 1          |
| EV3 Medium Motor          | 1          |
| Time-of-Flight Sensor     | 1          |
| Color Sensors             | 2          |
| Gyro Sensor               | 1          |
| OpenMV H7 Plus Camera     | 1          |
| Wide Angle Lens		    | 1          |
| Chassis (LEGO Parts)      | Assorted   |
| Cables                    | 6          |

## Power Supply
<img align="right" width="150" height="auto" src="https://github.com/user-attachments/assets/e38cd337-aa66-42fe-b2eb-afc074b24a00">The robot is powered by a rechargeable EV3 battery pack. This battery gives steady and long-lasting power to the EV3 brick, motors, and LEGO sensors. It was chosen instead of AA batteries because it gives more consistent power, is easy to recharge, and helps avoid stopping often during testing or competitions. The EV3 brick works like the robot’s power and control center. It controls how electricity is shared with the motors and sensors.

## Sensor Setup
To help the robot understand its surroundings, it uses different sensors. 

### Time-of-Flight Sensor
<img align="right" width="150" height="auto" src="https://github.com/user-attachments/assets/6f4eb5e6-16e8-49a5-afc4-1644056fd011">The Time-of-Flight (ToF) sensor mounted at the front of the robot helps measure the distance to objects ahead. It allows the robot to detect obstacles and avoid collisions. This sensor helps the car make smart decisions while driving, such as choosing safe paths and stopping when needed.

### Color Sensor
<img align="right" width="150" height="auto" src="https://github.com/user-attachments/assets/e83a7cca-7d2d-4a16-84a6-e3c7736ddcd7">There is also a color sensor facing the ground. It can see colors like blue, orange, or white on the floor. The robot uses this to detect lines at corners to spot corners, and know when to turn. This is also used to determine the direction of movement of the robot whether clockwise or counter-clockwise. 

### Gyro Sensor
<img align="right" width="150" height="auto" src="https://github.com/user-attachments/assets/840c8dde-032e-4abe-a448-1e89d2b7d838">The gyro sensor enables the robot to determine its precise orientation and measure the degree of its rotation. This capability is crucial for parking, as the vehicle must align accurately with the designated spot. By providing real-time feedback, the gyro sensor enhances the robot’s ability to park with improved accuracy and consistency.

### Camera and Communication
<img align="right" width="150" height="auto" src="https://github.com/user-attachments/assets/67e23bfb-b31c-4e05-ad8a-e58067cbd020">The robot uses an OpenMV H7 Plus camera mounted at the front. The camera gives the robot visual data like where a block is, or when it is facing a wall. Data communication between the EV3 and the OpenMV Camera is achieved using UART communication via the SerialTalk library. 


<img align="center" src="https://github.com/user-attachments/assets/e2d0fb48-4ba0-4395-8dbb-bed20a0102cb">

### Image Processing
The OpenMV H7 Plus camera processes images by first dividing the screen into two regions of interest (ROIs). It is marked with red and blue boxes. These ROIs help the robot focus on specific areas for black walls. 

<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/b019b982-1f55-4bc6-9bae-dc0d7cc715f0"/></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/d1e8b70d-0028-4d21-b5c9-87ce42477607"/></td>
  </tr>
</table>

The camera block detection uses the whole screen as its ROI to prioritize actually locating a block. When a block is found, the camera draws a rectangle around it and places a blue circle at its center for debugging. It draws a target point that has an offset to the left or right of the nearest block and acts as the robot's desired direction. 

<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/b0731fa0-a843-4fc4-8278-d2ed4f29482a"/></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/074bd0e9-8ad0-4da8-a002-f74b28f84746"/></td>
  </tr>
</table>

If the robot’s center is off from the desired direction, the system calculates an “err” (error) value. Based on this error, the camera draws an arrow showing the direction the robot should turn to face the block correctly. This visual feedback loop helps the robot adjust its path smoothly and quickly.

All the parts, including motors, sensors, camera, and EV3 brick, are placed carefully to keep the robot balanced and steady. The robot uses two motors: a medium motor in front for steering, and a large motor at the back for driving. These motors are strong and reliable, letting the robot move forward, backward, and turn smoothly even when it’s carrying extra weight like the camera and sensors.

## 💡 <mark> How to Improve Power and Sense Management</mark>
**Moving the camera farther back** on the robot can improve both power and sense management. When the camera is placed farther from the front, it can see a wider area of the path ahead, including more of the walls and obstacles on both sides. This gives the robot more time to react, reducing the need for sudden turns or quick power changes that waste energy. With more visual information, the robot can plan smoother movements, making driving more efficient and helping it avoid sharp corrections. This improves overall control while saving power and making obstacle detection more reliable.


# 🛑 OBSTACLE MANAGEMENT 🛑
## Camera Assistance
<img align="right" width="350" height="auto" src="https://github.com/user-attachments/assets/3f60f6c6-5b83-4e1c-8707-b796545f7719">The OpenMV H7 Plus camera is used to help the robot see and understand its environment. It is set to color mode with a resolution of 320×240 pixels and adjusted using vertical flip and horizontal mirror so that the camera view matches the real-world direction of the robot. Automatic settings such as white balance and gain are turned off to keep the image consistent across different lighting conditions. The camera also applies lens correction to reduce distortion and make object shapes appear more accurate.

The following flowchart outlines a step-by-step image processing sequence for a vision-based robot system. It begins with setting up the camera, which prepares the OpenMV module to start capturing images. The next stage focuses on finding color balance. During this step, the screen is split into two regions - left and right Regions of Interest (ROIs) - to help the camera examine each side separately. In each ROI, the system searches for black blobs and counts how many black pixels are present. The camera then uses this pixel data to calculate a value called `black_balance`, which represents a cleaner and more useful interpretation of the scene.

After establishing color balance, the camera begins the process of detecting blocks. It identifies blocks by finding blobs and measuring how close they are to the robot. This distance is estimated using the formula `blob.y + blob.h`, where higher values indicate that the block is nearer. The camera then selects the blob that appears closest.
Once the closest block is found, the camera calculates an error value. This error represents the horizontal difference between the center of the image and a target point placed to the left or right of the detected block. The error is scaled based on how far the block is from the robot.

In the final step, the camera sends the `black_balance` value, the `error`, and the estimated `distance` to the EV3 robot. This data helps the robot adjust its position and angle so it can correctly face the block based on what the camera sees.

<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/5993c869-aabb-49ea-9a77-09fcefcd0261"/></td>
  </tr>
</table>
<br>
</br>

## Black Area Detection for Wall
To help the robot stay centered on the track or avoid walls, the camera checks how much black is seen on the left and right sides of the image. It uses a defined threshold to find black blobs and calculates a **"white balance"** value based on the difference in black pixel count between the two sides. 

If one side has more black than the other, the robot can adjust its path to stay in the middle or prepare for a turn. The robot avoids hitting walls by checking how much black it sees on the left and right sides of the camera view. This is done in the `check_color_balance()` function:

```
def check_color_balance(img,img_debug,ROI_Y,ROI_H):
    #image filter to make find_blobs more accurate
    img_contrast1=img.copy()
    img_contrast1.gamma_corr(gamma=1.3,contrast=2.2)
    img_contrast2=img.copy()
    img_contrast2.gamma_corr(gamma=1.9, contrast=1.1,brightness=-0.1)#(gamma=2.0,contrast=1.5,brightness=-0.15)
    MAGENTA_THRESH=(0, 100, 17, 64, -22, 6)#(46, 80, 25, 127, -30, 14)#(60, 100, 25, 127, -128, 10)#(0, 100, 19, 127, -34, 20)
    BLACK_THRESH=(0, 93, -128, 0, -7, 36)
    #find black blobs
    roi_leftP=(0,ROI_Y,img.width()//3,ROI_H)
    roi_rightP=(2*(img.width()//3),ROI_Y,img.width()//3,ROI_H)
    roi_leftH=(0,ROI_Y,img.width()//2,ROI_H)
    roi_rightH=(img.width()//2,ROI_Y,img.width()//2,ROI_H)
    Lblack_blobs=img_contrast1.find_blobs([BLACK_THRESH],roi=roi_leftP,pixels_threshold=200)#find left side black blobs
    Rblack_blobs=img_contrast1.find_blobs([BLACK_THRESH],roi=roi_rightP,pixels_threshold=200)#find right side black blobs
    Lblack_pixels=sum([b.pixels() for b in Lblack_blobs])#sum of black pixels that are located in the left ROI
    Rblack_pixels=sum([b.pixels() for b in Rblack_blobs])#sum of black pixels that are located in the right ROI
    Lmagenta_blobs=img_contrast2.find_blobs([MAGENTA_THRESH],roi=roi_leftH,pixels_threshold=400)#find left side magenta blobs
    Rmagenta_blobs=img_contrast2.find_blobs([MAGENTA_THRESH],roi=roi_rightH,pixels_threshold=400)#find right side magenta blobs
    Lmagenta_pixels=sum([b.pixels() for b in Lmagenta_blobs])#sum of magenta pixels that are located in the left ROI
    Rmagenta_pixels=sum([b.pixels() for b in Rmagenta_blobs])#sum of magenta pixels that are located in the right ROI
    BLUE_THRESH=(0, 57, -20, 10, -34, -11)#(0, 80, -128, 127, -128, -5)#(0, 100, -128, 127, -128, -11)
    #(140,0,40,240)
    blue_blobs=img_contrast2.find_blobs([BLUE_THRESH],pixels_threshold=10,roi=img_roi,merge=True)
    blue=240-merged_y(blue_blobs)
    #blackH_balance=99999
    #wall_distance=99999
    #LblackH_blobs=img_contrast1.find_blobs([BLACK_THRESH],roi=(0,40,img.width()//2,200),pixels_threshold=200,merge=True)#find left side black blobs
    #RblackH_blobs=img_contrast1.find_blobs([BLACK_THRESH],roi=(img.width()//2,40,img.width()//2,200),pixels_threshold=200,merge=True)#find right side black blobs
    #if len(LblackH_blobs)==1 and len(RblackH_blobs)==1:
        #blackH_balance=(RblackH_blobs[0].y()+RblackH_blobs[0].h())-(LblackH_blobs[0].y()+LblackH_blobs[0].h())
        #if abs(blackH_balance)<15:
        #wall_distance=240-(((RblackH_blobs[0].y()+RblackH_blobs[0].h())+(LblackH_blobs[0].y()+LblackH_blobs[0].h()))/2)
    if True: #drawings for debugging
        img_debug.draw_rectangle(roi_leftP,color=(255,0,0))#draw red box on left
        img_debug.draw_rectangle(roi_rightP,color=(0,0,255))#draw blue box on right
        for blob in Lblack_blobs:
            img_debug.draw_rectangle(blob.rect(),color=(0,255,255))
        for blob in Rblack_blobs:
            img_debug.draw_rectangle(blob.rect(),color=(0,255,255))
        for blob in Lmagenta_blobs:
            img_debug.draw_rectangle(blob.rect(),color=(255,100,255))
        for blob in Rmagenta_blobs:
            img_debug.draw_rectangle(blob.rect(),color=(255,100,255))
        for blob in blue_blobs:
            img_debug.draw_rectangle(blob.rect(),color=(0,0,255))
    black_balance=Lblack_pixels-Rblack_pixels#max(-1100,min(1100,(Lblack_pixels-Rblack_pixels)/8))
    magenta_balance=max(-1100,min(1100,(Lmagenta_pixels+Rmagenta_pixels)/12))
    return int(black_balance),int(magenta_balance),int(blue)
```
- If the left side has more black, the robot knows it is too close to the left wall and should move right, and vice versa.
- This `black_balance` value helps the robot stay in the middle.

## Block Detection and Color Filtering
The camera also looks for red and green blocks on the course by enhancing the image with contrast and gamma correction and applying color filtering in LAB (instead of grayscale) format. It searches for blobs that match red or green thresholds and selects the one that is closest to the bottom of the screen, which usually means it’s the nearest block. This information can help the robot decide how to approach blocks.

<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/e3d8cf27-f619-4579-837d-4d4063adc283"/></td>
  </tr>
</table>

The robot uses color to find objects in front of it. This is done in `find_block()`:
```
def find_block(img,img_debug,distance_cap):
    img_contrast=img.copy()
    #img_contrast.gamma_corr(gamma=1.5,contrast=1.0,brightness=0)
    img_contrast=img_contrast.lens_corr(strength=1.53,x_corr=-0.04)#correct lens distortion
    #img_contrast=img_contrast.lens_corr(strength=1.62,x_corr=-0.04)#correct lens distortion
    img_contrast.gamma_corr(gamma=1.9,contrast=1.1,brightness=-0.1)
    color="None"
    blob=None
    nearestRed=None
    nearestGreen=None
    red_val=0
    green_val=0
    blocks=0
    #thresholds in LAB format for color filtering
    threshold_red=(0, 45, 23, 127, 5, 31)#(0, 32, 10, 127, 9, 127)#(0, 31, 10, 127, 9, 127)#(20, 45, 25, 40, 10, 127)#(0, 45, 25, 49, 13, 127)#(0, 46, 18, 49, 13, 127)#(0, 43, 9, 127, 4, 127)#(0, 35, 10, 127, -10, 127)#(0, 43, 10, 127, -12, 127)
    threshold_green=(25, 50, -65, -25, 21, 79)#(20, 45, -40, -15, 4, 57)#(21, 43, -128, -19, 16, 127)#(0, 100, -56, -25, 17, 50)#(28, 46, -56, -25, 17, 50)#(20, 45, -40, -15, 4, 57)#(20, 54, -40, -17, 0, 57)#(0, 100, -98, -25, -10, 127)
    #detect blobs with minimum size; no merging to prevent combining distant blocks
    red=img_contrast.find_blobs([threshold_red],pixel_threshold=100,roi=img_roi,merge=True)
    green=img_contrast.find_blobs([threshold_green],pixel_threshold=100,roi=img_roi,merge=True)
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
```
- It checks for red and green blocks using color filters.
- It picks the block closest to the bottom of the image because that one is nearest:
```
   val=b.y()+b.h()
```
## Calculating Target Error
Once a block is detected, the camera calculates an "error" value by comparing the block’s position with the center of the image. If the block is red, the target is slightly offset to the right; if it is green, it is offset to the left. 

<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/63b2a3f2-bb3b-4c54-902b-34c8c69dfbb4"/></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/838445fb-3571-498f-b8a8-872357acd197"/></td>
  </tr>
</table>

This gives the robot a point to aim for and the error tells how far it needs to turn left or right to face the target. Visual feedback, like circles and arrows, is drawn on the image for debugging and testing. This is done in the `target_point()` function:

```
def target_point(img_debug,block,color,k,near,nearMulti):
    offset_scaler=int(k*(block.y()+block.h()))
    if (block.y()+block.h())>near:offset_scaler=int(offset_scaler*nearMulti)
    if color=="red":offset_x=(block.x()+block.w())+offset_scaler#right side-offset
    if color=="green":offset_x=block.x()-offset_scaler#left side-offset
    draw_x=max(0,min(319,offset_x))
    img_debug.draw_circle(draw_x,block.cy(),3,color=(150,255,150),fill=True)#target for cursor
    img_debug.draw_arrow(img_center[0],img_center[1],draw_x,img_center[1],color=(0,0,255))
    error=offset_x-img_center[0]
    if color=="red" and error<0:error=error/2
    if color=="green" and error>0:error=error/2
    img_debug.draw_string(img_center[0]-30,img_center[1]-10,"Err: "+str(error),color=(255,255,255),scale=1)
    return error
```

- `offset_scaler` shifts the target point depending on the block's vertical position, giving more offset the closer it is.
- `k` is a tuning value that controls how much the robot offsets its aim depending on how close the block is.
- For red blocks, the robot aims slightly to the right: `offset_x` is set to the right edge of the block plus offset.
- For green blocks, it aims slightly to the left: `offset_x` is set to the left edge minus offset.
- The error is calculated as the difference between the target position `offset_x` and the image center `img_center[0] = 160`.
- This error is used to steer the robot left or right to face the block.
- Circles, arrows, and error text are drawn on the image for debugging.

## Block Distance Detection
The robot checks how close the block is using the following code:

```
distance=240-(block["blob"][1]+block["blob"][3])#works even when there is occlusion
```
If the block is near the bottom of the image, it means the block is close to the robot.

<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/7a007f3c-e1a3-40b9-adb7-e339b21b47e8"/></td>
  </tr>
</table>

## Sending Data to the Robot

```
st=SerialTalk()
cam_data=[0,0,0,0,0,0,0]
def cam():return cam_data# global cam_data
st.add_command(cam, "repr") cam_data=[white_balance,error,distance,blue,magenta_balance,tof.ping()-95,block_x]
    st.process()
```

At the end of every frame, the camera sends six (6) important values to the EV3 brick using SerialTalk over UART communication. These values are measured from specific parts of the image called Regions of Interest (ROIs), which are focused areas the camera checks. The camera uses color thresholds, ROIs, and pixel thresholds in the  find_blobs()  function to detect walls and blocks. The six data values sent are as follows:
<br>
<br>
**1. White balance**
- Comes from `check_color_balance()` → `black_balance = Lblack_pixels - Rblack_pixels`.
- The difference in the total black pixels detected between the left and right regions of the image.
- Positive → more black on the left side.
- Negative → more black on the right side.
- Used for general left/right centering against walls or black lines.

**2. Error**
- Computed by `target_point()` only if a red or green block is detected.
- The horizontal offset from the image center to the adjusted “target point” on the detected block.
- Positive → target is to the right of center.
- Negative → target is to the left of center.
- Small error means the block is already well-aligned.

**3. Distance**
- `240 - (block["blob"][1] + block["blob"][3])` → bottom of the detected block’s bounding box measured from image bottom.
- Approximates how far the nearest red or green block is from the camera in pixels.
- Smaller number → block is closer to the camera.
- Larger number → block is farther away.

**4. Blue**
- From `blue = 240 - merged_y(blue_blobs)`.
- Indicates the vertical position of the bottom of blue areas in the image.
- Larger value → blue is higher up in the image (possibly farther away).
- Smaller value → blue is lower in the image (possibly closer).

**5. Magenta balance**
- From `magenta_balance = max(-1100, min(1100, (Lmagenta_pixels - Rmagenta_pixels)/6))`.
- The difference in magenta pixels between the left and right ROIs.
- Positive → more magenta on left side.
- Negative → more magenta on right side.
- Useful for parking alignment.

**6. Time of Flight**
- From the Time-of-flight sensor via `tof.ping()`.
- Measures actual physical distance in millimeters to the object in front (not camera-based).
- Subtracting 95 is a calibration offset to account for sensor mounting or noise.
- Lower number → object is closer physically.

This data is used by the robot to decide how to move, steer, or stop.



## Control Logic and Behavior
The robot's main control loop reads sensor values and camera data, then decides how to move. If the camera sends a large left or right balance, the error is deemed large, thus the robot adjusts steering strongly to keep centered. It uses Proportional-Derivative (PD) control in the function `PID()` to match the steering motor angle with the desired direction from the camera. The robot continually computes for this error value and adjusts the front motor speed to reduce this error.

```
def PID(kp, kd, balance):

    global prev_error, error
    

    # Calculate the error
    error = balance

    # Calculate the derivative of the error
    d_error = error - prev_error

    # Calculate the control signal
    control_signal = kp * error + kd * d_error

    # Update previous error
    prev_error = error

    # Set motor speeds based on control signal
    fwspeed = control_signal
    # bwspeed = 100 - control_signal

    # Limit motor speeds to max values
    fwspeed = max(-1000, min(1000, fwspeed))
    # bwspeed = max(-1000, min(1000, bwspeed))

    if fwspeed > 0:
        fwspeed = 1000
    elif fwspeed < 0:
        fwspeed = -1000
    else:
        fwspeed = 1
    
    # if control_signal == 0:
    #     control_signal = 1

    # print("Error:", left-right, "control_signal:", control_signal, " fwspeed:", fwspeed)

    # Set motor speeds
    control_signal = int(control_signal)

    if control_signal > max_turnR:
        control_signal = max_turnR
    elif control_signal < max_turnL:
        control_signal = max_turnL
    
    fw.run_target(fwspeed, control_signal, wait=False)

    if control_signal > maxPower or control_signal < -maxPower:
        control_signal = maxPower
    
    bwspeed = maxPower - abs(control_signal)
    
    if bwspeed < minPower:
        bwspeed = minPower
    
    bw.run(bwspeed)
    print(" bwspeed:", bwspeed ,"control_signal:", control_signal)
```
## Wall Avoidance and PD Control
If the robot gets too close to the left or right wall, it activates a PD control behavior using the `PIDultraL()` or `PIDultraR()` functions. These functions make the robot turn away from the wall smoothly and maintain a safe distance. The robot also adjusts its back motor speed using `setBWSpeed()`, which varies the speed based on how far it is to a wall.

```
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
```

```
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
```

```
def setBWSpeed(min, max, balance):  # min and max must be positive
    bwSpeed = (min - max) / 1100 * abs(balance) + max
    bwSpeed = int(bwSpeed)
    bw.run(bwSpeed)
```

## Color Detection and Turning
The robot uses the color sensor to detect turn markers on the ground. If it sees orange first, it triggers the clockwise direction movement; if it sees blue first, it activates the counterclockwise direction movement. The robot keeps track of direction using a direction variable. For each turn, the robot runs through a turn sequence: it steers hard toward the turn, keeps driving until the opposite color is seen, and then turns back to straighten. After each successful turn, it increases the run counter by one.

## Lap Counting and Finish
The robot completes turns until it reaches a total of 12 turns, stored in the run variable. This usually represents 3 full laps with 4 turns each. Once the robot finishes its laps, it resets the back wheel angle to 0 and drives forward a short distance to simulate parking or completing the course.

```
def mainThread():
    ...
    while run <= 11:
        ...
        if color == 1 and (direction == 0 or direction == 1):
            ...

        elif color == -1 and (direction == 0 or direction == -1):
            ...
        
        ...
    else:
        ...

def intersectionCount():
    global run
    while run <= 11:
        # print("Run:", run)
        color = rgb_to_color(sensor.rgb())
        if color == 1:
            run += 1
            wait(2000)

threading.Thread(target=intersectionCount).start()
mainThread()
```

The robot combines camera input, color detection, ultrasonic distance sensing, and precise motor control to navigate an obstacle-filled track. It automatically detects when to turn, avoids walls, and stays centered. By combining PD and PID control with color-based behaviors, it can complete the course smoothly and reliably.

## 💡 <mark> How to Improve Obstacle Management</mark>
**Adding a backup behavior** can help the robot recover when it gets too close to an obstacle and cannot turn safely. If the detected block is very close (for example, when the distance is below a certain threshold), the robot can briefly reverse to create more space before trying to turn again. This extra room gives the front wheel more angle to steer and reduces the chance of hitting the obstacle. The backup motion should be short and slow to stay controlled, and it should only happen when needed. This behavior makes the robot more flexible and helps it continue moving even in tight spots or when the path is partially blocked.

# RESOURCES:
- [EV3 Documentation](https://pybricks.com/ev3-micropython/)
- [OpenMV Documentation](https://docs.openmv.io/)


