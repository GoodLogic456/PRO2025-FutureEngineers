<h1 align="center">🛑 CONTROL SOFTWARE 🛑</h1>
This repository contains the core components for a robotics vision and control system, integrating an EV3 robot with a camera module. Each script plays a specific role in enabling autonomous navigation and object interaction. Below is a breakdown of the main files:

#### `cameraMain`
Handles camera initialization and frame acquisition. This module sets up the camera stream and feeds live visuals into the processing pipeline, serving as the foundation for visual tasks such as block detection and white balance adjustment.

#### `detectBlocks`
Implements block detection using image processing techniques. It analyzes frames from the camera to identify and locate colored or shaped blocks, which are then used to guide the robot’s decision-making process.

#### `main`
Controls the EV3 robot’s motors and sensors. This script acts as the robot’s command center, interpreting input from the vision system and executing movement logic based on the detected blocks and environment conditions.

#### `whiteBalance`
Applies white balance correction to camera images. This module ensures color consistency under varying lighting conditions, improving the accuracy of visual processing tasks like block recognition.
