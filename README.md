# Autonomous Stretcher

An autonomous Stretcher for hospitals or enclosure closed to remotely send a stretcher from point A to point B. Based on ROS and simulated on Gazebo


# Table of contents

* [What is this?](#what-is-this?)
* [Description](#description)
* [Robot hardware requerimets](#robot-hardware-requerimets)
* [Simulation requeriments](#simulation-requeriments)
* [Instalation guide](#instalation-guide)
* [Hardware Scheme](#hardware-scheme)
* [3D piece](#3d-piece)
* [Software architecure diagram](#software-architecure-diagram)
* [ROS modules](#ros-modules)
    + [SLAM](#slam)
    + [A* algorithm](#a--algorithm)
    + [Dynamic Window Approach](#dynamic-window-approach)
* [RRT](#rrt)
* [Simulation](#simulation)
    + [2D Simulation goal](#2d-simulation-goal)
    + [3D Simulation goal](#3d-simulation-goal)
* [Video](#video)
* [Authors](#authors)

## What is this?

This is a project created in our third year of Computer Engineering with major on Comuputer Science at the Autonomous University of Barcelona for the subject _Robotics, Language and Planning_.

We have a budget of € 100 to create a robot that doesn't exists. Due to the coronavirus we have not been able to implement it in reality. Consequently, we have created a robot using [**ROS**](https://www.ros.org/about-ros//), [**Gazebo**](http://gazebosim.org/) and [**rviz**](http://wiki.ros.org/rviz) for the simulation part and [**Fritzing**](https://fritzing.org/) for the hardware scheme. 

Enjoy it!

## Description

The aim of the project is to make able any automous stretcher to move autonomously on a hospital from a point A to a point B avoiding obstacles in real time.

So, just changing some components of the stretcher it will become an autonous stretcher!

## Robot hardware requerimets

First of all, the stretcher will need some modifications, these are the hardware parts to make an stretcher autononous (Remember that we had a 100€ budget, so if you want more battery capacity or power you will need to do some changes): 

- Shaft Hub 4mm
- Arduino Uno
- 2200mAh 2S 25C Lipo Pack
- L298n Motor Driver
- Breadboard (Generic)
- Adafruit Compass
- Wifi module
- Ultrasound distance sensor
- Camera


## Simulation requeriments

In order to simulate the robot you will need You will need a working version of ROS. We have used and tested in ROS Kinetic Kame.  

- Ubuntu 16.04 Xelenial version
    - ROS kinetic
    - Gazebo 7 
    - Rviz
    
## Instalation guide

In order to facilitate the installation we have created a step-by-step pdf file. (See it here todo: aqui añadir link). This is a brief introduction. 

If you don't have installed ROS yet, you can use this quick and easy guide from ROS official web page: 

[Wiki ROS web page ](http://wiki.ros.org/ROS/Tutorials)

After ROS is intalled, you will need to install the turtlebot3 dependency package??? todo: revisar sin

To import our folder, you have to create a **_catkin workspace_**, we named ```catkin_ws```, create the packages into ```src**``` and paste de content of the folders (**important**: don't copy the files). Now, you are able to do ```catkin_make``` on your catkin workspace.

Lastly, import the folder ```_AS_robot_``` into ```~/model_editor_models``` and the folder ```_models_``` into ```~/.gazebo/models```. 

Towards faciliate the command promt line, add these commands into your ```_~/.bashrc_``` file:

```
source ~/catkin_ws/devel/setup.bash 
export TURTLEBOT3_MODEL=burger
```

Now you are able tu run all the packages as follow: 

```
roslaunch <package> <file>  
```


## Hardware Scheme

This is the Hardware Scheme we planned via [fritzing](https://fritzing.org/) within the 100€ budged. 

![Hardware Scheme](https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/hardware_scheme.png)


## 3D piece

Remember that the purpose of the project is make able an existing stretcher to be autonomous, so this is just the STL model of a stretcher adding a structure at the bottom for de hardware part (battery, arduiino motherboard and actuators). 

![Stretcher](https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/as.gif)
 

## Software architecure diagram

The diagram of the software scheme consists on sending the orders to the corresponding stretcher via wifi, this activates the **A star  algorithm global path planner module**, (In order to make faster the path planning rute nd more efficient route construction to get you off site as quickly as possible we also developed a _RRT_ algorith planner but not integrated with _ROS_). Afterwards, the stretcher will move following the glboal planner, in case it encounters an obstacle using the built-in sensors, the **local path planner** module will act, which consists of a **Dynamic Window Approach**, once the object or person is avoided, the stretcher will return to the route indicated by the global path planner until it reaches the destination.


<img src="https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/software_architecture.png" width="500">



## ROS modules
### SLAM

The **SLAM (Simultaneous Localization and Mapping)** is a technique to draw a map by estimating current location in an arbitrary space. To make possible the path planner work, we need a 2D map to make possible calculate the route. That's why we used the [Turtlebot3 SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/) system to get a 2D map.

Quick video of the mapping process: 

![Mapping process](https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/mapping_process.gif)

This is the result we got: 

<img src="https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/hospital_2d.png" width="600">


due to the map it was not created perfect and the route planner was going through walls or trying to find routes outside the hospital, we have applied knowledge of ***mathematical morphology*** learned in computer vision to create the following map in [_MATLAB_](https://matlab.mathworks.com/):

<img src="https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/hospital_2d_refined.png" width="600">


### A* algorithm

The **A*** algorithm included in the ROS package is used as a **global path planner**, It conssists of finding routes plans starts from a map coordinate, which it interprets as a vertex of a graph, and begins to explore adjacent nodes until the destination node (map destination coordinate) is reached. This algorithm generally finds very low cost routes in a very short time.

Example from [AtsushiSakai](https://github.com/AtsushiSakai/PythonRobotics#a-algorithm) of how it works: 

![A star](https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/a_star.gif)

### Dynamic Window Approach

The **DWA** algorithm included in the ROS package is used as a local path planner, it takes into account the dynamic and kinematic limitations of the stretcher. After entering the global plan and cost map, the **local path planner** will generate commands that will be sent to the move base. The objective of this planner is to follow the path indicated by the global planner, penalizing when deviating from the route in order to also be able to avoid obstacles.

Example from [AtsushiSakai](https://github.com/AtsushiSakai/PythonRobotics#dynamic-window-approach) of how it works: 

![DWA](https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/dwa.gif)


## RRT 

We created but not implemeted a **RRT** algorith in _MATLAB_ to perfom the global path planner. 

Example of path obtained:
<img src="https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/rrt_matlab.png" width="600">


## Simulation
### 2D and 3D Simulation of route using Gazebo and rviz

Using Gazebo and rviz , we can see in how by determining a goal destination it creates a route and follows the path: 

![2D simulation](https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/as_route_example.gif)


### Simulation of DWA 
We can see how putting one objects on the scene, first the global plalaner makes a route trough the objects but once the sensors detects them it dodges them using the local planner:

![3D Scheme](https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/as_dodging.gif)

## Video

TODO: CAMBIAR LINK
Larger video on youtube: [video](https://youtu.be/DAzql4cAIHs)

## Authors

Antonio Fernández Campagna [Github](https://github.com/ninofdz) [LinkedIn](https://www.linkedin.com/in/antoniofernandezcampagna/)
Enrique Ruíz Akamache [Github](https://github.com/enriamak) [LinkedIn](https://www.linkedin.com/in/enriamak/)
Sergio González Ariza [Github](https://github.com/gonza298) [LinkedIn](https://www.linkedin.com/in/sergio-gonz%C3%A1lez-ariza-ba4835165/)

