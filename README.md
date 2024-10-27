# ROS 2 Gazebo and OpenCV Project

This project integrates a simulated robot in **ROS 2** using **Gazebo** and image processing with **OpenCV**. The project includes launch configurations for simulation and visualization in RViz.

## Table of Contents
- [Description](#description)
- [Requirements](#requirements)
- [Installation](#installation)
- [File Structure](#file-structure)
- [Usage](#usage)
- [License](#license)

## Description

This project simulates a robot in a Gazebo environment. In addition to the simulation, the robot’s camera output is processed with **OpenCV**, allowing for real-time image analysis.

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
git clone https://github.com/your_username/ROS2_GAZEBO_AND_OPENCV.git
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
