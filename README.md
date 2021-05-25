# Autonomous Stretcher

An autonomous for hospitals or enclosure closed to remotely send a stretcher from point A to point B. Based on ROS and simulated on Gazebo

[![N|Solid](https://cldup.com/dTxpPi9lDf.thumb.png)](https://nodesource.com/products/nsolid)

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

# Table of contents

[Autonomous Stretcher](#autonomous-stretcher)
- [Table of contents](#table-of-contents)
  * [What is this](#what-is-this)
  * [Description](#description)
  * [Robot hardware requerimets](#robot-hardware-requerimets)
  * [Robot software requeriments](#robot-software-requeriments)
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

## What is this

This is a project created in our third year of computer engineering (major on comuputer science) at the Autonomous University of Barcelona for the subject of Robotics, Language and Planning.

We had a budget of € 100 to create a robot that doesn't exists. Due to the coronavirus we have not been able to implement it in reality.

Enjoy!

## Description

The aim of the project is to make able any automous stretcher to move autonomously on a hospital from a point A to a point B avoiding obstacles.

So, just changing some components to your stretcher it will become an autonous stretcher!

## Robot hardware requerimets

First of all, you will need to add some hardware parts your stretcher: 
(Remember that we had a 100€ budget, so if you want more battery capacity or power you will need to do some changes).

- Shaft Hub 4mm
- Arduino Uno
- 2200mAh 2S 25C Lipo Pack
- L298n Motor Driver
- Breadboard (Generic)
- Adafruit Compass
- Wifi module
- Ultrasound distance sensor
- Camera

## Robot software requeriments

- ROS Kinetic
- A 2d map.pgm of the environment

## Simulation requeriments

In order to simulate the robot you will need these COMPONENTS!!!??: 

- Ubuntu 16.04 Xelenial version
    - ROS kinetic
    - Gazebo 7 
    - Rviz
    
## Instalation guide

In order to facilitate the installation we have created a step-by-step pdf file. (See it here) todo: aqui añadir link

Once you have installed the ubuntu version we followed the ROS official web page: 

[Wiki ROS web page ](http://wiki.ros.org/ROS/Tutorials)

After ROS you will need to install the turtlebot3 dependency package

todo: aqui añadir instalcion
bla bla bla 
bla bla
bla

## Hardware Scheme

This is the Hardware Scheme we planned via [fritzing](https://fritzing.org/) within the 100€ budged. 

todo: aqui añadir imagen


## 3D piece

Remember that the purpose of the project is make able an existing stretcher to be autonomous, so this is just the .STL model why used for the simulation. You dont need extra parts!

todo: añadir aqui gif de la camilla girando 

## Software architecure diagram

The diagram of the software scheme consists on sending the orders to the corresponding stretcher via wifi, this activates the **A star  algorithm global path planner module**, (In order to make faster the path planning rute nd more efficient route construction to get you off site as quickly as possible we also developed a _RRT__ algorith planner but not integrated with _ROS_). Afterwards, the stretcher will move following the glboal planner, in case it encounters an obstacle using the built-in sensors, the **local path planner** module will act, which consists of a **Dynamic Window Approach**, once the object or person is avoided, the stretcher will return to the route indicated by the global path planner until it reaches the destination.


todo: aqui insertar imagen

## ROS modules
### SLAM

The **SLAM (Simultaneous Localization and Mapping)** is a technique to draw a map by estimating current location in an arbitrary space. To make possible the path planner work, we need a 2D map to make possible calculate the route. That's why we used the [Turtlebot3 SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/) system to get a 2D map.

This is the result we got: 

todo: aqui insertar imagen

due to the map it was not created perfect and the route planner was going through walls or trying to find routes outside the hospital, we have applied knowledge of ***mathematical morphology*** learned in computer vision to create the following map in _MATLAB_:

todo: aqui insertar imagen

### A* algorithm

The **A*** algorithm included in the ROS package is used as a **global path planner**, It conssists of finding routes plans starts from a map coordinate, which it interprets as a vertex of a graph, and begins to explore adjacent nodes until the destination node (map destination coordinate) is reached. This algorithm generally finds very low cost routes in a very short time.

todo: insertar aqui gif del rviz dandole a 2D nav goal

### Dynamic Window Approach

The **DWA** algorithm included in the ROS package is used as a local path planner, it takes into account the dynamic and kinematic limitations of the stretcher. After entering the global plan and cost map, the **local path planner** will generate commands that will be sent to the move base. The objective of this planner is to follow the path indicated by the global planner, penalizing when deviating from the route in order to also be able to avoid obstacles.

todo: insertar aqui 2 gif en camara rapida de como sortea obstaculo (gazebo y rviz)


## RRT 

We created but not implemeted a **RRT** algorith in _MATLAB_ to perfom the global path planner. 
How it works: 

todo: aqui insertar gif

## Simulation
### 2D Simulation goal 

Using Rviz, we can see in 2D how by determining a goal destination it creates a route and follows the path: 

todo: aqui insertar gif entero 


### 3D Simulation goal 
Using Gazebo, we can see in 3D the stretcher moving autonomously on the hospital till it reaches the navigation goal.

todo: aqui insertar simulacion 3d

## Video



## Authors



