# ROS 2 Gazebo and OpenCV Project

<p align="center">
  <img src="https://github.com/user-attachments/assets/52c307a1-2463-484b-b286-897a9b1ac7f2" alt="Imagen 1" width="250" />
  <img src="https://github.com/user-attachments/assets/5be885e3-b179-417e-84e2-627b6869d34b" alt="Imagen 2" width="400" />
  <img src="https://github.com/user-attachments/assets/ebf3549a-53c3-40d4-8126-274a79189442" alt="Imagen 3" width="200" />
</p>


This project integrates a simulated robot in **ROS 2** using **Gazebo** and image processing with **OpenCV**. The project includes launch configurations for simulation and visualization in RViz.

## Table of Contents
- [Description](#description)
- [Requirements](#requirements)
- [Installation](#installation)
- [File Structure](#file-structure)
- [Usage](#usage)
- [License](#license)

## Description

![image](https://github.com/user-attachments/assets/601c1360-51ef-43f5-bb9b-b9e0d2590350)

This project simulates a mobile robot in ROS 2 using Gazebo for physical simulation and RViz for visualization. The robot model is designed in a modular way, with specific components for the base, wheels, and an onboard camera that provides real-time images for processing.

The robot’s base is configured as a differential drive system, allowing independent control of the side wheels to perform complex turning and movement. Each wheel and the main body of the robot are modeled with detailed physical properties, including mass, inertia, and collisions. These properties are defined using reusable macros, making future expansions and adjustments easier. The Gazebo setup includes specific controllers for differential drive, enabling velocity commands through ROS topics and providing odometry data to track the robot’s real-time position and orientation within the simulated environment.

The robot's camera is designed to transmit real-time images via ROS topics, allowing integration with image-processing nodes based on OpenCV. The camera is positioned on the robot through links and joints that ensure proper placement on the robot’s body. Additionally, it includes sensors configured to provide views in both RViz and Gazebo, facilitating visual monitoring and data capture.

A dedicated image-processing node, implemented in Python and using OpenCV, complements the camera’s functionality. In the project, you will see two topics with the same functionality (`/camera_sensor_rviz` and `camera_sensor_opencv`). This is because during the development of this project, I read that some users reported certain errors in the compatibility between the images obtained from Gazebo with OpenCV. Therefore, in the camera definition, the axes are rotated to have the origin in the upper left corner, as is the convention in image processing. For the example, the node subscribes to the `/camera_sensor_opencv/image_raw` topic, where it receives and processes images in real time. With the help of cv_bridge, we will ensure that the images received in ROS format are converted to a format compatible with OpenCV, enabling advanced image-processing tasks. In this case, the images are displayed in a window titled "Robot Camera Feedback," which supports the development and testing of vision algorithms, such as object detection and navigation.

The design of this project in ROS 2 leverages the flexibility of URDF and Xacro files to maintain a dynamic and easily adaptable model. This allows the robot to realistically simulate navigation, obstacle detection, and image processing, providing a versatile platform for testing and development in controlled environments.


### Main Components
- **my_robot_bringup**: Launch scripts to start Gazebo, RViz, and the vision node.
- **my_robot_description**: URDF and Xacro files describing the robot model.
- **my_robot_vision**: OpenCV-based vision node for image processing.

## Requirements

- **Ubuntu 22.04**
- **ROS 2 Humble**
- **Gazebo Fortress**
- **Python 3.10**
- **OpenCV** and **cv_bridge**
- **Imported in Gazebo the models from the [gazebo_models](https://github.com/osrf/gazebo_models.git) directory**

## Installation

### Step 1: Clone the Repository

```bash
git clone https://github.com/IvanFernande/ros2_gazebo_with_opencv.git
cd ROS2_GAZEBO_AND_OPENCV
```

### Step 2: Set Up the ROS 2 Workspace

```bash
# From the workspace root
colcon build
source install/setup.bash
```

### Step 3: Install Additional Dependencies
If you don’t have `cv_bridge` or `OpenCV` installed, you can install them with the following commands:
```bash
sudo apt update
sudo apt install ros-humble-cv-bridge python3-opencv
```

## File Structure
The project file structure is as follows:
```bash
.
└── src
    ├── my_robot_bringup
    │   ├── CMakeLists.txt
    │   ├── launch
    │   │   ├── my_robot_complete_launch.py
    │   │   ├── my_robot_gazebo.launch.py
    │   │   └── my_robot_gazebo.launch.xml
    │   ├── package.xml
    │   ├── rviz
    │   │   └── urdf_config.rviz
    │   └── worlds
    │       └── test_world.world
    ├── my_robot_description
    │   ├── CMakeLists.txt
    │   ├── launch
    │   │   ├── display.launch.py
    │   │   └── display.launch.xml
    │   ├── package.xml
    │   ├── rviz
    │   │   └── urdf_config.rviz
    │   └── urdf
    │       ├── camera.xacro
    │       ├── common_properties.xacro
    │       ├── mobile_base_gazebo.xacro
    │       ├── mobile_base.xacro
    │       └── my_robot.urdf.xacro
    └── my_robot_vision
        ├── launch
        │   └── camera_launch.py
        ├── my_robot_vision
        │   ├── camera_opencv_node.py
        │   └── __init__.py
        ├── package.xml
        ├── resource
        │   └── my_robot_vision
        ├── setup.cfg
        ├── setup.py
        └── test
            ├── test_copyright.py
            ├── test_flake8.py
            └── test_pep257.py
```

## Usage
### Launch Simulation in Gazebo and Vision Node with OpenCV
To start the Gazebo simulation and process the camera image in OpenCV, use the following command:
```bash
ros2 launch my_robot_bringup my_robot_complete_launch.py
```
This will launch:
- Gazebo with the robot and the defined environment in test_world.world.
- RViz to visualize the robot model and camera.
- The image processing node using OpenCV.

## License
This project is licensed under the Apache 2.0 License. For more details, see the LICENSE file.

This version of the `README.md` should help make the project accessible to a broader audience. Let me know if you’d like any additional details added!
