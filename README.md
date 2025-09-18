### Introduction
<img align="right" width="300" height="auto" src="https://github.com/user-attachments/assets/57859eae-9a47-4e67-ab5d-f5903471aeb7">

This is the Engineering Documentation of **TEAM GRACE CHRISTIAN COLLEGE (GCC)** for **World Robot Olympiad Open Championship
Asia & Pacific 2025 - Future Engineers**.

This repository contains engineering materials of a self-driven vehicle's model. It also contains relevant information such as [wiring diagrams](https://github.com/GoodLogic456/PRO2025-FutureEngineers/tree/main/schemes), robot [images](https://github.com/GoodLogic456/PRO2025-FutureEngineers/tree/main/v-photos) and [videos](https://github.com/GoodLogic456/PRO2025-FutureEngineers/blob/main/video/video.md), code snippets, technical explanations, and [Engineering Journal](https://docs.google.com/spreadsheets/d/1C1dG2nG-GNh3qbOAAdMjjLXDwYX-iINyFB89ZOEz_A0/edit?usp=sharing).

### TABLE OF CONTENTS
- [Mobility Management](#-Mobility-Management-)
- [Power and Sense Management](#-Power-and-Sense-Management-)
- [Obstacle Management](#-obstacle-management-)



# 🛑 MOBILITY MANAGEMENT 🛑
## Motor and Steering System
<img align="right" width="300" height="auto" src="https://github.com/user-attachments/assets/9a97cd82-10d3-4494-8b1e-2387e99ee5ca">The robot was built using the Lego EV3 Mindstorm Robot System. The robot moves using one **(1) EV3 large motor** at the back and one **(1) EV3 medium motor** at the front. The large motor pushes the robot forward and backward, while the medium motor turns a gear rack to steer the front wheel. As required by the rules, the robot employs an **Ackermann** steering system, just like a real car, so it can turn smoothly. The steering system is powered by a worm gear, giving it much power to turn. The robot is also guided by an **OpenMV Camera**, **Time-of-Flight sensor**, **color sensor**,  **gyro sensor**, and  **9-DOF sensor**. 

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
    control_signal = -control_signal
    fwspeed = control_signal

    # Limit motor speeds to max values
    fwspeed = max(-1000, min(1000, fwspeed))
    if fwspeed > 0:
        fwspeed = 1000
    elif fwspeed < 0:
        fwspeed = -1000
    else:
        fwspeed = 1
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
    
    bw.run(-bwspeed)

    print(" fwspeed:", fwspeed ," bwspeed:", bwspeed ,"control_signal:", control_signal)
```

## Wheel and Motor Selection for Stability
<img align="right" width="300" height="auto" src="https://github.com/user-attachments/assets/ba71dd6a-8557-4401-afd7-e159b95799b8">The motors were picked because they are strong, reliable, and have enough turning power (torque) to move the robot smoothly on the track. The robot also uses hard wheels with flat edges in both the front and back. This design gives the robot better grip and control when turning. Since the wheels are the same size and shape, the robot stays level, which helps the color sensor stay flat so it can read the floor properly. 


## Chassis Build and Balance
The robot’s body, or chassis, is made of LEGO parts. It is strong but not heavy, and it holds all the parts in place. The motors, EV3 intelligent brick, camera, and sensors are mounted carefully to keep the robot balanced. The weight is spread out evenly to stop the robot from tipping during sharp turns. The final design makes the robot steady, smooth, and ready to finish the course without losing control.

## Building Instructions
|       |       |
|:-----------:|:-----------:|
|   ![1](https://github.com/user-attachments/assets/fa9385a4-b065-499a-b367-f030f6d2e92a)    |   ![2](https://github.com/user-attachments/assets/b0e76ca0-0afd-4365-ac5f-f25c96457530)  |
|  ![3](https://github.com/user-attachments/assets/3daf9568-ab5f-4c73-95af-8548b5e655c6)     |  ![4](https://github.com/user-attachments/assets/f3803874-5ca8-423d-a910-0647b66b027c)   |
|   ![5](https://github.com/user-attachments/assets/8c81226a-2992-407f-8728-ff90054f942f)    |  ![6](https://github.com/user-attachments/assets/28c57701-8258-4ec4-b419-e7185ddf30fc)   |
|   ![7](https://github.com/user-attachments/assets/f05cf2d6-76eb-4adc-8fc4-b8dcbf2c8dff)    |   ![8](https://github.com/user-attachments/assets/6e25bff2-3a6c-465b-aa40-22ef72ae93dc)   |
|  ![9](https://github.com/user-attachments/assets/8dd086ac-b6f0-439a-a723-6c15299e82ad)     |   ![10](https://github.com/user-attachments/assets/833d18f0-655c-4962-901b-1f5837eadcf0)  |
|   ![11](https://github.com/user-attachments/assets/79070488-73ee-4af5-b095-e593bd752a89)     |   ![12](https://github.com/user-attachments/assets/23b2809e-2531-4e5b-ac57-268775eda7b5)  |
|   ![13](https://github.com/user-attachments/assets/7713518a-c5c7-4229-bf1e-89cf1fe553b1)    |   ![14](https://github.com/user-attachments/assets/849b792a-8082-4099-b64a-b620830b0efa)   |
|   ![15](https://github.com/user-attachments/assets/b8a95c2b-80fc-454d-acf1-d23ba6bacb77)    |  ![16](https://github.com/user-attachments/assets/e98eefb7-56ce-4ade-85d1-36de89a58b05)  |
|   ![17](https://github.com/user-attachments/assets/278c659a-1ab4-4783-81af-e1f102b05be4)    |   ![18](https://github.com/user-attachments/assets/d5d73d5e-afe4-439a-9b7d-5fbfc711250b)  |
|  ![19](https://github.com/user-attachments/assets/6341250c-37e4-4f1f-85a7-9b6c16d3eb0c)  |   ![20](https://github.com/user-attachments/assets/77c04bb6-8af4-4eea-ab93-79ba847e9b4d) |
|   ![21](https://github.com/user-attachments/assets/9a1ea50b-743c-4478-9ae0-b58640e999ac)    |   ![22](https://github.com/user-attachments/assets/86d66b06-92e1-41b3-9acc-d84a0e7f004f)  |
|   ![23](https://github.com/user-attachments/assets/47ce499a-a6a2-45a5-8705-4f1f97f1efd4)    |   ![24](https://github.com/user-attachments/assets/79199aa0-e369-4af8-b5bd-847e6df29311)2  |
|   ![25](https://github.com/user-attachments/assets/fcd24ec6-9f97-421b-bace-3b27e4e30bf6)    |   ![26](https://github.com/user-attachments/assets/7792b1ce-b256-46bd-9c1d-47ced71c821e) |
|  ![27](https://github.com/user-attachments/assets/c87ba1ea-df7a-4153-bfcb-292d5e145a64)   |   ![28](https://github.com/user-attachments/assets/c3f5dcc9-1f08-440c-a014-0973bd971847) |
|   ![29](https://github.com/user-attachments/assets/0a8f5957-4bfe-460b-8734-eca58b1f02a2)    |   ![30](https://github.com/user-attachments/assets/6151649a-1c8c-48ff-873b-fc585d7e9d00)  |
|   ![31](https://github.com/user-attachments/assets/f0b700ed-bd01-46d3-b015-64d8b492cb5f)|   ![32](https://github.com/user-attachments/assets/595962b7-8465-483e-a965-91044b932ca4)|
|   ![33](https://github.com/user-attachments/assets/a6b4ab4e-9022-4f3b-82c5-c1f0fdb340ae)   |   ![34](https://github.com/user-attachments/assets/15ea85c0-3a76-4726-b448-e9f77b1a1a7d)  |
|   ![35](https://github.com/user-attachments/assets/9eece7d6-f9c3-4cc4-96be-746c2125f3d2)   |   ![36](https://github.com/user-attachments/assets/15db550f-98bd-42cc-b1e0-eb90d32c56f9)  |
|   ![37](https://github.com/user-attachments/assets/5609a6aa-1a9e-4cc6-bc19-f36e9e0a0b64)  |   ![38](https://github.com/user-attachments/assets/6706b473-d4ed-4986-bc9e-3887bd03e351)  |
|   ![39](https://github.com/user-attachments/assets/774e9fef-2d47-4b21-afca-f4678eefb29e)   |   ![40](https://github.com/user-attachments/assets/5e533637-8636-4028-aebc-c50586ab3c22) |
|   ![41](https://github.com/user-attachments/assets/567525cc-79a2-43cd-9689-1fc62e4d0797)   |  ![42](https://github.com/user-attachments/assets/f148b01b-598e-4c7c-a356-9f6ec248808c)  |
|   ![43](https://github.com/user-attachments/assets/dabf82bc-599c-42c2-8930-db46d3900f62)   |   ![44](https://github.com/user-attachments/assets/de73d6b1-d52d-4ede-8a97-3081b2983103)  |
|   ![45](https://github.com/user-attachments/assets/d7fe63a5-ad91-416e-b337-85f593924822)   |   ![46](https://github.com/user-attachments/assets/d1870e23-c6f8-4ca0-bc7c-a4fefb239f71)  |
|   ![47](https://github.com/user-attachments/assets/e3067468-7459-4d05-aa5a-f7763d47b4df)   |   ![48](https://github.com/user-attachments/assets/5c53e227-dedc-493a-9b6e-e33f32fd9d7d) |
|   ![49](https://github.com/user-attachments/assets/cabc3051-f45d-46f7-a84a-38266f39a46d)   |  ![50](https://github.com/user-attachments/assets/693e3dcb-7c34-43c4-9bb6-ecc5e4b5f365)  |
|   ![51](https://github.com/user-attachments/assets/4b6d01c7-3b02-4cc5-9dca-eb11e05bbd1e)  |  ![52](https://github.com/user-attachments/assets/95b00a1d-2de8-45f7-9e79-7ac55fb6ed6e) |
|   ![53](https://github.com/user-attachments/assets/d2a8faea-641c-4bf0-8494-6ec07d91b29d)  |  ![54](https://github.com/user-attachments/assets/d8ac6611-7a7f-494b-8d3e-f85fe82c053d)  |
|   ![55](https://github.com/user-attachments/assets/e2031331-e851-4651-84e6-4fe7102a2309)   |   ![56](https://github.com/user-attachments/assets/da0d616d-b4ba-4b58-9a22-8503a02cf1db)|
|   ![57](https://github.com/user-attachments/assets/2f86c795-5d2c-4857-bd3f-8d9afa039934)   |   ![58](https://github.com/user-attachments/assets/bbec9be9-6bfb-4c21-a66d-99c1dd862dc3)  |
|   ![59](https://github.com/user-attachments/assets/3470635c-b18c-44d0-b122-8ad5b3f9bd06)    |   ![60](https://github.com/user-attachments/assets/e6048a47-0159-455f-94a0-f53f835898cc)  |

## 💡 <mark> How to Improve Mobility Management</mark>
One way to improve the robot is by **adding a rear camera**. This camera can help the robot see what is behind it, which is useful when it needs to move backward or park between objects. It can also help the robot avoid bumping into walls or blocks while reversing. With a rear camera, the robot can better understand its surroundings and move more safely and accurately. This helps the robot do its tasks more smoothly.

Making the robot **smaller and lighter** helps it turn faster and easier. A lighter robot is easier to steer because the motor doesn’t need to work as hard to move it. When the robot is smaller, it can make sharp turns more easily and quickly. This also puts less pressure on the steering parts, so they can move faster. Using compact designs, placing parts closer together, and choosing lightweight materials all help make the robot quicker and easier to steer.

# 🛑 POWER AND SENSE MANAGEMENT 🛑

## Bill of Materials (BOM)
![61](https://github.com/user-attachments/assets/158462e6-1736-4dc4-8705-5524fe24030a)
![62](https://github.com/user-attachments/assets/ce3f4679-cef4-41b6-8cdc-79420136bc80)
![63](https://github.com/user-attachments/assets/46b13d33-13d1-4f35-a522-17ac5527eaf8)
![64](https://github.com/user-attachments/assets/cfd31493-4224-4904-a096-200c063bf5b6)
![65](https://github.com/user-attachments/assets/95af64ee-c978-4a65-9801-ecf6d3724580)
![66](https://github.com/user-attachments/assets/224f6bd3-71ff-4a20-a656-87e6fe3ddee1)
![67](https://github.com/user-attachments/assets/643e1fb9-063f-44d9-8940-19f77279ee65)
![68](https://github.com/user-attachments/assets/df8072ac-e073-4934-a478-a517917c27f2)
![69](https://github.com/user-attachments/assets/1a91322c-db16-419f-af2f-9197690abe15)
| **Component**  | **QTY** |**PHOTO** |
|:-----------:|:-----------:|:-----------:|
| Time-of-Flight Sensor    | 1   | <img align="right" width="150" height="auto" src="https://github.com/user-attachments/assets/1c281547-4b13-4824-9026-66098ac47b98">  |
| 9-DOF Sensor    | 1  | <img align="right" width="150" height="auto" src="https://github.com/user-attachments/assets/500a48b8-a905-4693-9280-fe0456b58ef7">   |
| OpenMV H7 Plus Camera     | 1   | <img align="right" width="150" height="auto" src="https://github.com/user-attachments/assets/ca1d665c-cde8-455f-a26d-12d1ab6409a1">    |
| Wide Angle Lens	   |1   | <img align="right" width="150" height="auto" src="https://github.com/user-attachments/assets/7eb588bf-bb52-49d7-9370-03dba4abc7ed">    |



## Power Supply
<img align="right" width="150" height="auto" src="https://github.com/user-attachments/assets/e38cd337-aa66-42fe-b2eb-afc074b24a00">The robot is powered by a rechargeable EV3 battery pack. This battery gives steady and long-lasting power to the EV3 brick, motors, and LEGO sensors. It was chosen instead of AA batteries because it gives more consistent power, is easy to recharge, and helps avoid stopping often during testing or competitions. The EV3 brick works like the robot’s power and control center. It controls how electricity is shared with the motors and sensors.

## Sensor Setup
To help the robot understand its surroundings, it uses different sensors. 



### Time-of-Flight Sensor
<img align="right" width="150" height="auto" src="https://github.com/user-attachments/assets/56449262-6bec-4036-917c-d06d3153e6d7">The Time-of-Flight (ToF) sensor mounted at the front of the robot helps measure the distance to objects ahead. It allows the robot to detect obstacles and avoid collisions. This sensor helps the car make smart decisions while driving, such as choosing safe paths and stopping when needed.

### Color Sensor
<img align="right" width="150" height="auto" src="https://github.com/user-attachments/assets/f5033f01-560a-43f0-9e59-1c9295ce8c9a">There is also a color sensor facing the ground. It can see colors like blue, orange, or white on the floor. The robot uses this to detect lines at corners to spot corners, and know when to turn. This is also used to determine the direction of movement of the robot whether clockwise or counter-clockwise. 

### 9-DOF Sensor
<img align="right" width="150" height="auto" src="https://github.com/user-attachments/assets/adbf839a-a2bb-44d0-ac4d-6377ce6ed761">The BNO055 is a 9 Degrees of Freedom (DOF) sensor. It  tracks rotation and direction of the robot. an accelerometer, gyroscope, and magnetometer. It uses its gyroscope to give a clearer orientation with less drift , so the car can avoid obstacles more accurately.

### Camera and Communication
<img align="right" width="150" height="auto" src="https://github.com/user-attachments/assets/9096ae26-8217-4886-bbfb-1efc27bbac4e">The robot uses an OpenMV H7 Plus camera mounted at the front. The camera gives the robot visual data like where a block is, or when it is facing a wall. Data communication between the EV3 and the OpenMV Camera is achieved using UART communication via the SerialTalk library. 

### Gyro Sensor
<img align="right" width="150" height="auto" src="https://github.com/user-attachments/assets/840c8dde-032e-4abe-a448-1e89d2b7d838">The gyro sensor enables the robot to determine its precise orientation and measure the degree of its rotation. 

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
    <td><img src="https://github.com/user-attachments/assets/c75763e1-6a68-4be9-9aae-cef8e4c6a37f"/></td>
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

The following flowchart outlines a step-by-step image processing sequence for a vision-based robot system. It begins with setting up the camera, which prepares the OpenMV module to start capturing images. The next stage focuses on finding color balance. During this step, the screen is split into two regions - left and right Regions of Interest (ROIs) - to help the camera examine each side separately. In each ROI, the system searches for black blobs and counts how many black pixels are present. The camera then uses this pixel data to calculate a value called `white_balance`, which represents a cleaner and more useful interpretation of the scene.

After establishing color balance, the camera begins the process of detecting blocks. It identifies blocks by finding blobs and measuring how close they are to the robot. This distance is estimated using the formula `blob.y + blob.h`, where higher values indicate that the block is nearer. The camera then selects the blob that appears closest.
Once the closest block is found, the camera calculates an error value. This error represents the horizontal difference between the center of the image and a target point placed to the left or right of the detected block. The error is scaled based on how far the block is from the robot.

In the final step, the camera sends the  `white_balance`  ,  `color` ,  `distance`  ,  `tof.ping`  ,  `round(yaw)`  ,  `block_x`  to the EV3 robot. This data helps the robot adjust its position and angle so it can correctly face the block based on what the camera sees.
<br>
</br>

## Black Area Detection for Wall
To help the robot stay centered on the track or avoid walls, the camera checks how much black is seen on the left and right sides of the image. It uses a defined threshold to find black blobs and calculates a **"white balance"** value based on the difference in black pixel count between the two sides. 
![detect-black-wall](https://github.com/user-attachments/assets/9bbf9c6a-a324-4d3f-905e-7f5882d23243)

If one side has more black than the other, the robot can adjust its path to stay in the middle or prepare for a turn. The robot avoids hitting walls by checking how much black it sees on the left and right sides of the camera view. This is done in the `white_balance()` function:

```
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
```
- It checks for red and green blocks using color filters.
- It picks the block closest to the bottom of the image because that one is nearest:
```
   val=b.y()+b.h()
```

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
cam_data=[0,0,0,0,0,0]
def cam():return cam_data# global cam_data
st.add_command(cam, "repr") cam_data=[white_balance,color,distance,tof.ping(),round(yaw),block_x]
st.process()
```

At the end of every frame, the camera sends six (6) important values to the EV3 brick using SerialTalk over UART communication. These values are measured from specific parts of the image called Regions of Interest (ROIs), which are focused areas the camera checks. The camera uses color thresholds, ROIs, and pixel thresholds in the  find_blobs()  function to detect walls and blocks. The six data values sent are as follows:
<br>
<br>

**1. White balance**
- Comes from  `check_color_balance()`  →  `black_balance() = Lblack_pixels - Rblack_pixels` 
- The difference in the total black pixels detected between the left and right regions of the image.
- Positive → more black on the left side.
- Negative → more black on the right side.
- Used for general left or right centering against walls or black lines.

**2. Color**
- Gives the color of the block to the robot

**3. Distance**
-  `240 - (block[“blob”][1] + block[“blob”][3])`  → bottom of the detected block’s bounding box measured from image bottom.
- Approximates how far the nearest red or green block is from the camera in pixels.
- Smaller number → block is closer to the camera.
- Larger number → block is farther away.

**4. Time of Flight**
- From the Time-of-flight sensor via  `tof.ping()` 
- Measures actual physical distance in millimeters to the object in front (not camera-based).
- Subtracting 95 is a calibration offset to account for sensor mounting or noise.
- Lower number → object is closer physically.

**5. Yaw**
- From the 9-DOF sensor via  `imu.euler()`
- Gives the accurate direction of the robot

**6. Block_x**
- Gives the center of a block’s  x coordinate


This data is used by the robot to decide how to move, steer, or stop.



## Control Logic and Behavior
The robot's main control loop reads sensor values and camera data, then decides how to move. If the camera sends a large left or right balance, the error is deemed large, thus the robot adjusts steering strongly to keep centered. It uses Proportional-Derivative (PD) control in the function `PID()` to match the steering motor angle with the desired direction from the camera. The robot continually computes for this error value and adjusts the front motor speed to reduce this error.

```
def gyroPID(kp, kd, target_yaw, ki=0, minPower=200, maxPower=700, resetKI=True):
    
    global prev_error, error, yaw, section, sum_error

    if resetKI:
        sum_error = 0

    if minPower > 0 or maxPower > 0:
        error = calc_turn_error(yaw, target_yaw)
    else:
        error = -(calc_turn_error(yaw, target_yaw)) 

    # Calculate the error
    # if minPower > 0 or maxPower > 0:
    #     error = yaw - target
    # else:
    #     error = target -yaw

    # Calculate the derivative of the error
    d_error = error - prev_error

    sum_error += error

    # Calculate the control signal
    control_signal = kp * error + kd * d_error + ki * sum_error

    # control_signal = control_signal

    # Update previous error
    prev_error = error

    if control_signal > 0:
        fwspeed = -abs((maxPower - (abs(control_signal) * 20)))
    elif control_signal < 0:
        fwspeed = abs((maxPower - (abs(control_signal) * 20)))
    else:
        fwspeed = 1
    
    if int(fwspeed) == 0:
        fwspeed = 1

            # Set motor speeds

    control_signal = int(-(control_signal))
    fwspeed = int(fwspeed)
    fw.run_target(fwspeed, control_signal, wait=False)


    if minPower > 0 or maxPower > 0:
        if control_signal > maxPower or control_signal < -maxPower:
            control_signal = maxPower

    else:
        if control_signal < maxPower or control_signal > -maxPower:
            control_signal = maxPower

    if minPower > 0 or maxPower > 0:
        bwspeed = maxPower - (abs(control_signal) * 20) 

        if bwspeed < minPower:
            bwspeed = minPower
    else:
        bwspeed = maxPower + (abs(control_signal) * 20) 

        if bwspeed > minPower:
            bwspeed = minPower

    if bwspeed > 0:
        bw.run(-700)
    else:
        bw.run(700)

def gyroTurn(minpower, maxpower, steer, angle): # Ferror is the first value is not changing 

    global yaw
    begin_angle = yaw
    Ferror = calc_turn_error(begin_angle, (target_yaw + angle))
    error = calc_turn_error(yaw, (target_yaw + angle))
    steerToAngle(0, 300, steering = steer)
    
    while sign(error) == sign(Ferror) and sign(Ferror)!=0:
        error = calc_turn_error(yaw, (target_yaw + angle))

        # print("error",error,"sign",sign(error))
        bwspeed = int(((maxpower - minpower ) * (abs(error)/abs(Ferror)) + minpower))
        bw.run(-bwspeed)

    bw.stop()
    steerToAngle(0, 300, steering = 0)
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

The robot combines camera input, color detection, distance sensing, and precise motor control to navigate an obstacle-filled track. It automatically detects when to turn, avoids walls, and stays centered. By combining PD and PID control with color-based behaviors, it can complete the course smoothly and reliably.

## 💡 <mark> How to Improve Obstacle Management</mark>
**Adding a backup behavior** can help the robot recover when it gets too close to an obstacle and cannot turn safely. If the detected block is very close (for example, when the distance is below a certain threshold), the robot can briefly reverse to create more space before trying to turn again. This extra room gives the front wheel more angle to steer and reduces the chance of hitting the obstacle. The backup motion should be short and slow to stay controlled, and it should only happen when needed. This behavior makes the robot more flexible and helps it continue moving even in tight spots or when the path is partially blocked.

# RESOURCES:
- [EV3 Documentation](https://pybricks.com/ev3-micropython/)
- [OpenMV Documentation](https://docs.openmv.io/)


