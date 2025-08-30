# turtlebot3_color_detection

## Table Of Contents

* [Overview](#-overview)
* [Structure](#-structure)
* [Requirements](#-requirements)
* [Parameters](#-parameters)
* [Usage](#-usage)
* [Results](#-results)
* [Author](#author)

## 🔍 Overview

This repository contains a ROS application that uses OpenCV to detect a specific color in the scene and directs the TurtleBot3 Waffle to move toward the target.

> [!NOTE]
>This is one of the exercises I use to introduce Computer Vision with ROS to students.

 ## 📁 Structure

- **launch/**
  - `color_detection.launch`: launches Gazebo simulation with TurtleBot3 and the custom world.
- **src/**
  - `color_detection.py`: main code for launching the node and running the program.
  - `color_image.py`: handles the color identificantion pipeline. 
  - `velocity.py`: defines the speeds of the robot. 
- **worlds/**
  - `color_detection.world`: custom world with cylinders of different colors.
- `CMakeLists.txt`
- `README.md`
- `package.xml`


## 💻 Requirements
- ROS and turtlebot3 (in this case Noetic was used)
  ```
  sudo apt-get install ros-noetic-gazebo-ros ros-noetic-turtlebot3*
  ```
- Python
- Gazebo
- OpenCV

## 🔧 Parameters

All the parameters are defined within the code. 
You can change the `color` to detect, adjust the `pixel ranges` used for color recognition, and configure different `velocities` to observe how the robot’s movement behavior changes.

## 🚀 Usage

Launch the Gazebo simulation with turtlebot3:
```bash
roslaunch color_detection color_detection.launch
```

After that, you can execute the code with:
```bash
python color_detection.py 
```

## 📊 Results
As a result, the robot will rotate to identify the color specified in the code using its camera. Once the color is detected, it will move toward the cylinder. If the robot gets too close to the object due to higher speeds, it will automatically go backwards.

![color_detection](https://github.com/user-attachments/assets/832e31ba-307a-49ec-a54d-6ddc55d330f3)

## Author

**Esther Vera Moreno**
 * [Personal Github](https://github.com/EstherRobotics)
 * [ICAERUS Github](https://github.com/ICAERUS-EU/UC1_Crop_Monitoring)
 * [Medium Projects](https://medium.com/@esthervera99)
 * [LinkedIn](https://www.linkedin.com/in/estherverarobotics/) 



