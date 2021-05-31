# Autonomous Stretcher

An autonomous Stretcher for use in hospitals or closed enclosures. A stretcher is sent from point A to point B by means of remote control. Engineered by application of ROS and simulated on Gazebo.


# Table of contents

* [What is this?](#what-is-this?)
* [Description](#description)
* [Robot hardware requerimets](#robot-hardware-requerimets)
* [Simulation requeriments](#simulation-requeriments)
* [Instalation guide](#instalation-guide)
* [Hardware Scheme](#hardware-scheme)
* [3D piece](#3d-piece)
* [Software architecure diagram](#software-architecure-diagram)
* [RQT Graph](#rqt-graph)
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

This is a project created in our third year of Computer Engineering with a major in Computer Science at the Autonomous University of Barcelona for the subject _Robotics, Language and Planning_.

We had a budget of **100€** to create a robot that doesn't exist. Due to the pandemic caused by covid-19, we have not been able to implement it in reality. Consequently, we have created a robot using [**ROS**](https://www.ros.org/about-ros//), [**Gazebo**](http://gazebosim.org/) and [**rviz**](http://wiki.ros.org/rviz) for the simulation part, and [**Fritzing**](https://fritzing.org/) for the hardware scheme. 

Enjoy it!

## Description

The prorgams approach is to enable any stretcher to move autonomously in a hospital or given closed enclosured avoiding obstacles in real-time.

So, just by changing some components of a stretcher it will become autonomous.

## Robot hardware requirements

First of all, the stretcher will need some modifications, these are the hardware parts to make a stretcher autonomous (Remember that we had a 100€ budget, so if you want more battery capacity or power you will need to do some changes): 

- Shaft Hub 4mm
- Arduino Uno
- 2200mAh 2S 25C Lipo Pack
- L298n Motor Driver
- Breadboard (Generic)
- Adafruit Compass
- Wifi module
- Ultrasound distance sensor
- Camera
- 4 Wheels

## Simulation requirements

In order to simulate the robot, you will need you need a working version of ROS. We have used and tested in ROS Kinetic Kame.  

- Ubuntu 16.04 Xelenial version
    - ROS kinetic
    - Gazebo 7 
    - Rviz
    
## Instalation guide

With the intention of facilitating the installation, we have created a **step-by-step** [pdf file](https://github.com/ninofdz/Autonomous_Stretcher/blob/main/instalation_guide.pdf).

#### This is a brief introduction:

If you don't have installed ROS yet, you can use this quick and easy guide found on the official ROS web page: 

[Wiki ROS web page ](http://wiki.ros.org/ROS/Tutorials)

Once ROS is installed, you will need to install the turtlebot3 dependency package.

To import our folder, you have to create a **_catkin workspace_**, we named ```catkin_ws```, create the packages into ```src**``` and paste de content of the folders (**important**: don't copy the files). Now, you can do ```catkin_make``` on your catkin workspace.

Lastly, import the folder ```AS_robot``` into ```~/model_editor_models``` and the folder ```_models_``` into ```~/.gazebo/models```. 

In order to facilitate the command prompt line, add these commands into your ```_~/.bashrc_``` file:

```
source ~/catkin_ws/devel/setup.bash 
export TURTLEBOT3_MODEL=burger
```

Finally, you should be able to run all the packages as follow: 

```
roslaunch <package> <file>  
```


## Hardware Scheme

This is the Hardware Scheme we planned via [fritzing](https://fritzing.org/) within the 100€ budget. 

![Hardware Scheme](https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/hardware_scheme.png)


## 3D piece

Remember that the purpose of the project is to enable an existing stretcher to autonomous mode, so this is just the STL model of a stretcher adding a structure at the bottom for de hardware part (battery, Arduino motherboard, and actuators). 

![Stretcher](https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/as.gif)
 

## Software architecture diagram

The diagram of the software scheme shows that orders are sent to the corresponding stretcher via wifi, this activates the **A star algorithm global path planner module**, (To accelerate the path planning process and to obtain a more efficient route mapping to start as quickly as possible, we have also developed an _RRT_ algorithm planner but not integrated with _ROS_). Afterwards, the stretcher will move following the global planner, in case it encounters an obstacle using the built-in sensors, the **local path planner** module will act, which consists of a **Dynamic Window Approach**, once the object or person is bypassed, the stretcher will return to the route indicated by the global path planner until it reaches the destination.


<img src="https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/software_architecture.png" width="500">

## RQT Graph

Using the **ROS rqt graph** we have generated a graph that allows us to observe the nodes that are active and the connections between the editors and subscribers who are in charge of moving the base of our stretcher. These nodes are involved in the entire operation of our robot. We can see reflected in the graph, the sensors, events, and actions of our robot, events of the ROS simulator itself, and how they are all connected to each other. In green, we have the route that is currently being followed, and therefore, the nodes that are being used.

![rqt](https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/rtq_graph.png)

## ROS modules
### SLAM

The **SLAM (Simultaneous Localization and Mapping)** is a technique to draw a map by estimating the current location in an arbitrary space. To enable the path planner we need a 2D map as base for calculating the route. That's why we used the [Turtlebot3 SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/) system to get a 2D map.

Quick video of the mapping process: 

![Mapping process](https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/mapping_process.gif)

This is the result we got: 

<img src="https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/hospital_2d.png" width="600">


due to the deficiencies of the map planning was not perfect and the route planner did not respect walls or searched a way outside the premises, we then applied knowledge of ***morphological transformations*** learned in computer vision to create the following map in [_MATLAB_](https://matlab.mathworks.com/):

<img src="https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/hospital_2d_refined.png" width="600">


### A* algorithm

The **A*** algorithm included in the ROS package is used as a **global path planner**, It develops route plans starting from map coordinates, which it interprets as a vertex of a graph, and begins to explore adjacent nodes until the destination node (map destination coordinate) is reached. This algorithm generally finds very low-cost routes in a very short time.

Example from [AtsushiSakai](https://github.com/AtsushiSakai/PythonRobotics#a-algorithm) of how it works: 

![A star](https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/a_star.gif)

### Dynamic Window Approach

The **DWA** algorithm included in the ROS package is used as a local path planner, it takes into account the dynamic and kinematic limitations of the stretcher. After entering the global plan and cost map, the **local path planner** will generate commands that will be sent to the move base. The objective of this planner is to follow the path indicated by the global planner, penalizing when deviating from the route in order to also be able to avoid obstacles.

Example from [AtsushiSakai](https://github.com/AtsushiSakai/PythonRobotics#dynamic-window-approach) of how it works: 

![DWA](https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/dwa.gif)


## RRT 

We created but not implemented a **RRT** algorithm in _MATLAB_ to perform the global path planner. 

Example of path obtained:

<img src="https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/rrt_matlab.png" width="600">


## Simulation
### 2D and 3D Simulation of the route using Gazebo and rviz

Using Gazebo and rviz, we can see how by determining a goal destination creates a route and follows the path: 

![2D simulation](https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/as_route_example.gif)


### Simulation of DWA 
We can see how putting some objects on the scene, the global planer makes a route through the objects but once the sensors detect them it dodges them using the local planner:

![3D Scheme](https://github.com/ninofdz/Autonomous_Stretcher/blob/main/images/as_dodging.gif)

## Video

Larger video on youtube: [video](https://youtu.be/DAzql4cAIHs)

## Authors

Antonio Fernández Campagna [Github](https://github.com/ninofdz) [LinkedIn](https://www.linkedin.com/in/antoniofernandezcampagna/)

Enrique Ruíz Amakrache [Github](https://github.com/enriamak) [LinkedIn](https://www.linkedin.com/in/enriamak/)

Sergio González Ariza [Github](https://github.com/gonza298) [LinkedIn](https://www.linkedin.com/in/sergio-gonz%C3%A1lez-ariza-ba4835165/)

## LICENSE MIT
