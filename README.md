### Introduction
<img align="right" width="300" height="auto" src="https://github.com/user-attachments/assets/f133dc92-9622-44ed-b9c5-4ef0885c1dcc">

This is the Engineering Documentation of **TEAM GRACE CHRISTIAN COLLEGE (GCC)** for **PRO Future Engineers 2025**

This repository contains engineering materials of a self-driven vehicle's model. It also contains relevant information such as [wiring diagrams](https://github.com/GoodLogic456/PRO2025-FutureEngineers/tree/main/schemes), robot [images](https://github.com/GoodLogic456/PRO2025-FutureEngineers/tree/main/v-photos) and [videos](https://github.com/GoodLogic456/PRO2025-FutureEngineers/blob/main/video/video.md), code snippets, technical explanations, and [Engineering Journal](https://docs.google.com/spreadsheets/d/1C1dG2nG-GNh3qbOAAdMjjLXDwYX-iINyFB89ZOEz_A0/edit?usp=sharing).

### TABLE OF CONTENTS
- [Mobility Management](#-Mobility-Management-)
- [Power and Sense Management](#-Power-and-Sense-Management-)
- [Obstacle Management](#-obstacle-management-)




# 🛑 MOBILITY MANAGEMENT 🛑
## Motor and Steering System
The robot was built using the Lego EV3 Mindstorm Robot System. The robot moves using one **(1) EV3 large motor** at the back and one **(1) EV3 medium motor** at the front. The large motor pushes the robot forward and backward, while the medium motor turns a gear rack to steer the front wheel. As required by the rules, the robot employs an **Ackermann** steering system, just like a real car, so it can turn smoothly. The steering system is powered by a worm gear, giving it much power to turn. The robot is also guided by an **OpenMV Camera** and two **(2) Lego EV3 ultrasonic sensors**. 

The robot uses **Proportional-Derivative** control to make steering more accurate. This system helps the robot align its actual turning angle with the desired angle calculated from camera feedback. The difference between the actual turning angle and the computed desired angle is the error. The goal then is to drive this error to zero (or near zero). This is achieved by compensating for the error in the function `turnAngle2()`.

```
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
```
## Wheel and Motor Selection for Stability
The motors were picked because they are strong, reliable, and have enough turning power (torque) to move the robot smoothly on the track. The robot also uses hard wheels with flat edges in both the front and back. This design gives the robot better grip and control when turning. Since the wheels are the same size and shape, the robot stays level, which helps the color sensor stay flat so it can read the floor properly. 

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
| Ultrasonic Sensors        | 2          |
| Color Sensors             | 2          |
| OpenMV H7 Plus Camera     | 1          |
| OFDL Breakout Board       | 1          |
| Chassis (LEGO Parts)      | Assorted   |
| Cables                    | 6          |

## Power Supply
<img align="right" width="150" height="auto" src="https://github.com/user-attachments/assets/100d7292-a1b9-4c19-88a0-c4be4693d1b6">The robot is powered by a rechargeable EV3 battery pack. This battery gives steady and long-lasting power to the EV3 brick, motors, and LEGO sensors. It was chosen instead of AA batteries because it gives more consistent power, is easy to recharge, and helps avoid stopping often during testing or competitions. The EV3 brick works like the robot’s power and control center. It controls how electricity is shared with the motors and sensors.

## Sensor Setup
To help the robot understand its surroundings, it uses different sensors. 

### Ultrasonic Sensor
<img align="right" width="150" height="auto" src="https://github.com/user-attachments/assets/e19fb035-aa6b-420b-b1a8-0ac10d51db6c">There are two ultrasonic sensors located on the left and right of the robot. These sensors use sound waves to measure how close the robot is to walls or obstacles. This helps the robot stay in the middle of the path and avoid hitting anything.

### Color Sensor
<img align="right" width="150" height="auto" src="https://github.com/user-attachments/assets/69e7000d-ab9b-4903-9afd-d118d461ae51">There is also a color sensor facing the ground. It can see colors like blue, orange, or white on the floor. The robot uses this to detect lines at corners to spot corners, and know when to turn. This is also used to determine the direction of movement of the robot whether clockwise or counter-clockwise. 

### Camera and Communication
The robot uses an OpenMV H7 Plus camera mounted at the front. The camera is connected using an OFDL breakout board (https://github.com/GoodLogic456/PRO2025-FutureEngineers/tree/main/models), which makes it easier to attach to the EV3 brick. More information about the board can be found at: [https://github.com/ofdl-robotics-tw/EV3-OpenMV-Stuff](https://github.com/ofdl-robotics-tw/EV3-OpenMV-Stuff). The camera gives the robot visual data like where a block is, or when it is facing a wall. Data communication between the EV3 and the OpenMV Camera is achieved using UART communication via the SerialTalk library. 

<img align="center" width="60%" height="auto" src="https://github.com/user-attachments/assets/e2d0fb48-4ba0-4395-8dbb-bed20a0102cb">

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
One way to improve the robot’s power and sense management is by **adding a gyro sensor**. A gyro sensor measures how much the robot turns or rotates. This helps the robot stay on a straight path, even if the wheels slip or the floor is not even. It also makes turning more accurate and consistent, especially in tight spaces or when following a planned path. With better control of its direction, the robot doesn't waste power fixing its movement and can finish its tasks more smoothly and steadily. Adding a gyro sensor helps the robot move more correctly and stay balanced.

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
from serialtalk.auto import SerialTalk
st=SerialTalk()
cam_data=[0,0,0]
def cam():return cam_data# global cam_data
st.add_command(cam, "repr")

cam_data=[white_balance,error,distance]
    print(cam_data)
    st.process()
```

At the end of every frame, the camera sends three (3) important values to the EV3 brick using SerialTalk over UART communication. These values are measured from specific parts of the image called Regions of Interest (ROIs), which are focused areas the camera checks. The camera uses color thresholds, ROIs, and pixel thresholds in the `find_blobs()` function to detect walls and blocks. The three data values sent are as follows:
1. **White balance** – difference in black pixel areas between left and right, helps the robot stay in the middle of the path
2. **Error** – how far the target is from the center
3. **Distance** – how close the detected block is

This data is used by the robot to decide how to move, steer, or stop.

## Control Logic and Behavior
The robot's main control loop reads sensor values and camera data, then decides how to move. If the camera sends a large left or right balance, the error is deemed large, thus the robot adjusts steering strongly to keep centered. It uses Proportional-Derivative (PD) control in the function `turnAngle2()` to match the steering motor angle with the desired direction from the camera. The robot continually computes for this error value and adjusts the front motor speed to reduce this error.

```
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
while run <= 11:
    ...
    if rgb_to_color(sensor.rgb()) == 1: # orange
        if direction == 0 or direction == 1: # no direction or clockwise
            ...
            run = run + 1
            direction = 1
    if rgb_to_color(sensor.rgb()) == -1: # blue
        if direction == 0 or direction == -1: # no direction or counter-clockwise
		...
            run = run + 1
            direction = -1
    ...
    if direction == 0:
        ...
        run = 0
if run > 11:
   ...
    run = 20
if run ==20:
    fw.stop()
    bw.stop()
```

The robot combines camera input, color detection, ultrasonic distance sensing, and precise motor control to navigate an obstacle-filled track. It automatically detects when to turn, avoids walls, and stays centered. By combining PD and PID control with color-based behaviors, it can complete the course smoothly and reliably.

## 💡 <mark> How to Improve Obstacle Management</mark>
**Adding a backup behavior** can help the robot recover when it gets too close to an obstacle and cannot turn safely. If the detected block is very close (for example, when the distance is below a certain threshold), the robot can briefly reverse to create more space before trying to turn again. This extra room gives the front wheel more angle to steer and reduces the chance of hitting the obstacle. The backup motion should be short and slow to stay controlled, and it should only happen when needed. This behavior makes the robot more flexible and helps it continue moving even in tight spots or when the path is partially blocked.

# RESOURCES:
- [EV3 Documentation](https://pybricks.com/ev3-micropython/)
- [OpenMV Documentation](https://docs.openmv.io/)
- [OFDL Breakout Board GitHub Repository](https://github.com/ofdl-robotics-tw/EV3-OpenMV-Stuff)

