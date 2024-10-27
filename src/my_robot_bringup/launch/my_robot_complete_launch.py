from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ruta al archivo de lanzamiento de Gazebo (ahora en Python)
    gazebo_launch_path = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'launch',
        'my_robot_gazebo.launch.py'
    )

    # Ruta al archivo de lanzamiento de la cámara con OpenCV
    camera_launch_path = os.path.join(
        get_package_share_directory('my_robot_vision'),
        'launch',
        'camera_launch.py'
    )

    return LaunchDescription([
        # Lanzar Gazebo con el robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path)
        ),
        
        # Lanzar el nodo de la cámara
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_path)
        ),
    ])
