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




## 🛑 MOBILITY MANAGEMENT 🛑
### Motor and Steering System
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
### Wheel and Motor Selection for Stability
The motors were picked because they are strong, reliable, and have enough turning power (torque) to move the robot smoothly on the track. The robot also uses hard wheels with flat edges in both the front and back. This design gives the robot better grip and control when turning. Since the wheels are the same size and shape, the robot stays level, which helps the color sensor stay flat so it can read the floor properly. 

### Chassis Build and Balance
The robot’s body, or chassis, is made of LEGO parts. It is strong but not heavy, and it holds all the parts in place. The motors, EV3 intelligent brick, camera, and sensors are mounted carefully to keep the robot balanced. The weight is spread out evenly to stop the robot from tipping during sharp turns. The final design makes the robot steady, smooth, and ready to finish the course without losing control.

### 💡 <mark> How to Improve Mobility Management</mark>
One way to improve the robot is by **adding a rear camera**. This camera can help the robot see what is behind it, which is useful when it needs to move backward or park between objects. It can also help the robot avoid bumping into walls or blocks while reversing. With a rear camera, the robot can better understand its surroundings and move more safely and accurately. This helps the robot do its tasks more smoothly.

Making the robot **smaller and lighter** helps it turn faster and easier. A lighter robot is easier to steer because the motor doesn’t need to work as hard to move it. When the robot is smaller, it can make sharp turns more easily and quickly. This also puts less pressure on the steering parts, so they can move faster. Using compact designs, placing parts closer together, and choosing lightweight materials all help make the robot quicker and easier to steer.

## 🛑 POWER AND SENSE MANAGEMENT 🛑


## 🛑 OBSTACLE MANAGEMENT 🛑
