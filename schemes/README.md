<h1 align="center">🛑 SCHEMATIC WIRING DIAGRAM 🛑</h1>

<img width="1920" height="1080" alt="Schematic Wiring Diagram" src="https://github.com/GoodLogic456/PRO2025-FutureEngineers/blob/main/schemes/Schematic%20Wiring%20Diagram.png" />

This is the schematic wiring diagram for a self-driving EV3 robot. It uses the EV3 brick as the main controller. A **large motor** at the back (Port B) moves the robot forward and backward, while a **smaller motor** in front (Port C) helps it turn using a gear rack. On the front, there's an **OpenMV H7 Plus camera** (Port S4) and **time-of-flight sensor** that look ahead - helpful for spotting objects or following a path. There's also a **9-DOF sensor** that tracks the orientation of the robot and a **color sensor** (Port S2) that checks the ground to follow lines or detect corners. A **gyro sensor** (Port S3) measures the robot’s turning angle and direction. Together, these parts let the robot move, see, and make smart decisions while driving.


| **COMPONENT**              | **FUNCTION**                                                                 | **PORT**          |
|---------------------------|------------------------------------------------------------------------------|-------------------|
| ![brick](https://github.com/user-attachments/assets/0982eeab-965d-4400-b72f-9835b56fa64d)    | **EV3 Intelligent Brick**<br>The robot has an intelligent brick, which acts as its brain. It controls the motors, reads data from the sensors, and runs the program that tells the robot what to do. The intelligent brick processes all the information and makes decisions, like when to turn, stop, or change direction. It connects all the parts of the robot and helps it work as one complete system.                 |  |
| ![large-motor](https://github.com/user-attachments/assets/dd46ed5f-9364-47d9-809c-cd49239200f2)         | **EV3 Large Motor**<br>Drives wheels or heavy mechanisms  The EV3 large motor is mounted at the back of the robot and is used to drive the vehicle forward and backward. It provides strong power and consistent speed, allowing the robot to move smoothly across the track. The large motor also helps the robot maintain a steady pace, which is important for following lines, avoiding obstacles, and completing laps efficiently.                  | Port B   |
| ![medium-motor](https://github.com/user-attachments/assets/e8b69b61-682d-465c-9bd6-9bd77d23a485)         | **EV3 Medium Motor**<br>The EV3 medium motor is placed at the front of the robot and is used for steering. It controls the direction of the front wheel, allowing the robot to turn left or right. Because the medium motor is smaller and lighter, it can adjust the steering quickly and precisely. This helps the robot make smooth and accurate turns, especially when navigating curves or switching between clockwise and counterclockwise paths.               | Port C   |
| ![time-of-flight-sensor](https://github.com/user-attachments/assets/01b93beb-95ec-4d88-838b-ecc78fc73f71)        | **Time-of-Flight Sensor**<br>The Time-of-Flight (ToF) sensor measures how far objects are in front. It gives fast readings, even for small or thin objects, so the robot can quickly decide when to slow down, stop, or turn. Because it has a narrow view, it can focus straight ahead and avoid being confused by objects on the side. |    |
| ![color-sensor](https://github.com/user-attachments/assets/514344b0-6de1-4463-9635-46e9fba00502)              | **EV3 Color Sensor**<br>The robot has a color sensor that helps it detect and respond to different colors on the ground and surroundings. This sensor is used to identify certain areas of the course. By reading the color of the surface, the robot can decide when to do certain actions like turning. This helps the robot stay on track and complete the tasks correctly during the competition. | Port S2   |
| ![camera](https://github.com/user-attachments/assets/816eff7d-a9c9-4b20-b6eb-407be2072f52)      | **OpenMV H7 Plus Camera**<br>The robot uses an OpenMV H7 Plus camera to see in front of it and guide its movement. The camera can detect key elements on the track, such as obstacle blocks, walls, and colored lines on the floor. It sends this information to the robot through SerialTalk, allowing the robot to understand what’s ahead and decide how to move. This helps the robot navigate the course more accurately and respond to different situations during the competition. | Port S4 |
| ![wide-angle-lens](https://github.com/user-attachments/assets/da917b02-3e57-40bc-b735-6138b657e443)      | **Wide Angle Lens**<br>A wide-angle lens on the camera helps the robot see more of its surroundings at once. This lets the robot see a much wider area, which helps it drive better and avoid things in its way.  |  |
| ![9-dof-sensor](https://github.com/user-attachments/assets/004e3bdc-2463-440b-ae41-3a56c5b41747)      | **9-DOF Sensor**<br>The BNO055 is a highly integrated 9-DOF (Degrees of Freedom) sensor. It is designed to simplify motion tracking and orientation detection by combining multiple sensors and processing capabilities into one compact chip.  |  |
| ![gyro-sensor](https://github.com/user-attachments/assets/c83b62f5-cd90-4a02-afe0-a36359e082d1)    | **Gyro Sensor**<br>The gyro sensor helps the robot know its exact direction and how much it has turned. This is important for parking because the car needs to line up accurately with the parking spot. The gyro’s feedback allows the car to park with greater accuracy and consistency.     |   Port S3   |





















