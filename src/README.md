<h1 align="center">🛑 CONTROL SOFTWARE 🛑</h1>
This repository contains the core components for a robotics vision and control system, integrating an EV3 robot with a camera module. Each script plays a specific role in enabling autonomous navigation and object interaction. Below is a breakdown of the main files:

#### `cameraMain`
Handles camera initialization and frame acquisition. This module sets up the camera stream and feeds live visuals into the processing pipeline, serving as the foundation for visual tasks such as block detection and white balance adjustment.

#### `robot-obstacle-challenge`
Helps the robot move safely through areas with obstacles. It reads data from camera and sensors to find objects in the way, then chooses a path to avoid them. This module makes sure the robot can drive smoothly without crashing during the Obstacle Challenge.

#### `robot-open-challenge`
Controls the robot’s main tasks and movements. It starts up the motors, sensors, and camera, then decides what the robot should do based on what it sees and senses. This module helps the robot make 3 laps during the Open Challenge.
