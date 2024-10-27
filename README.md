# ROS 2 Gazebo and OpenCV Project

Este proyecto integra un robot simulado en **ROS 2** usando **Gazebo** y procesamiento de imágenes con **OpenCV**. El proyecto incluye configuración de lanzamientos para simulación y visualización en RViz.

## Tabla de Contenidos
- [Descripción](#descripción)
- [Requisitos](#requisitos)
- [Instalación](#instalación)
- [Estructura de Archivos](#estructura-de-archivos)
- [Uso](#uso)
- [Licencia](#licencia)

## Descripción

Este proyecto simula un robot en un entorno de Gazebo. Además de la simulación, se procesa la salida de la cámara del robot con **OpenCV**, lo que permite realizar análisis de imágenes en tiempo real.

### Componentes Principales
- **my_robot_bringup**: Scripts de lanzamiento para iniciar Gazebo, RViz y el nodo de visión.
- **my_robot_description**: Archivos URDF y Xacro para describir el modelo del robot.
- **my_robot_vision**: Nodo de visión basado en OpenCV para procesamiento de imágenes.

## Requisitos

- **Ubuntu 22.04**
- **ROS 2 Humble**
- **Gazebo Fortress**
- **Python 3.10**
- **OpenCV** y **cv_bridge**
- **Descargar modelos para gazebo del directorio [gazebo_models](https://github.com/osrf/gazebo_models.git)

## Instalación

### Paso 1: Clonar el Repositorio

```bash
git clone https://github.com/tu_usuario/ROS2_GAZEBO_AND_OPENCV.git
cd ROS2_GAZEBO_AND_OPENCV
