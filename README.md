### Introduction
<img align="right" width="250" height="auto" src="https://github.com/user-attachments/assets/f133dc92-9622-44ed-b9c5-4ef0885c1dcc">

 This is the Engineering Documentation of **TEAM GRACE CHRISTIAN COLLEGE (GCC)** for **PRO Future Engineers 2025**

This repository contains engineering materials of a self-driven vehicle's model. It also contains relevant information such as wiring diagrams, robot images and videos, code snippets, technical explanations, and engineering journal.

**Official GitHub Repository** – GCC Future Engineers 2025 
This is the official GitHub repository of GCC Future Engineers 2025. This repository contains the engineering materials for a self-driving vehicle model created for the **PRO Future Engineers** competition in the season 2025. The repository serves as a comprehensive archive of the team’s work, including source code, technical documentation, and supporting resources.

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
The OpenMV H7 Plus camera is used to help the robot see and understand its environment. It is set to color mode with a resolution of 320×240 pixels and adjusted using vertical flip and horizontal mirror so that the camera view matches the real-world direction of the robot. Automatic settings such as white balance and gain are turned off to keep the image consistent across different lighting conditions. The camera also applies lens correction to reduce distortion and make object shapes appear more accurate.

The following flowchart outlines a step-by-step image processing sequence for a vision-based robot system. It begins with setting up the camera, which prepares the OpenMV module to start capturing images. The next stage focuses on finding color balance. During this step, the screen is split into two regions - left and right Regions of Interest (ROIs) - to help the camera examine each side separately. In each ROI, the system searches for black blobs and counts how many black pixels are present. The camera then uses this pixel data to calculate a value called `black_balance`, which represents a cleaner and more useful interpretation of the scene.

After establishing color balance, the camera begins the process of detecting blocks. It identifies blocks by finding blobs and measuring how close they are to the robot. This distance is estimated using the formula `blob.y + blob.h`, where higher values indicate that the block is nearer. The camera then selects the blob that appears closest.
Once the closest block is found, the camera calculates an error value. This error represents the horizontal difference between the center of the image and a target point placed to the left or right of the detected block. The error is scaled based on how far the block is from the robot.

In the final step, the camera sends the `black_balance` value, the `error`, and the estimated `distance` to the EV3 robot. This data helps the robot adjust its position and angle so it can correctly face the block based on what the camera sees.

![Obstacle Management](https://github.com/user-attachments/assets/d3a3c617-cc3f-46df-97a0-300f5e605dd0)


