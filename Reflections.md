# Reflection on the Highway Driving Project

## What is it?

This project implements highway driving using Udacity's simulator. The simulator provides sensor fusion data, which are pre-processed to obtain x/y coordinates of cars (including the ego car), 

## Installation and Execution Instructions

1. Download the Udacity simulator [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  The installation process will vary between, Linux, MacOS and Windows. I developed this project on MacOS, and the stand-alone application was available without any installation.
2. Clone this repo:
```
git clone https://github.com/mkolod/CarND-Path-Planning-Project.git
```
or if you're using ssh instead of https:
```
git clone git@github.com:mkolod/CarND-Path-Planning-Project.git
```
3. Build the path planner project: 
```
cd CarND-Path-Planning-Project
mkdir build
cd build
cmake ..
make
```
Compilation is very fast, but if you're on Linux or MacOS, you can use all cores for running make by instead typing
```
make -j$(nproc)
```

4. Start the path planner application:
```
./path_planning
```
This will start the application, which will open up uWebSocket on port 4567, which the simulator will use to talk to it. On MacOS, you may need to click "Allow" after a pop-up shows up asking you if you want the application to accept incoming network connections.

5. Start the Udacity simulator - on MacOS, the application is called term3_sim. A dialog box will ask you about the resolution you'd like to choose. It's best to choose the top resolution, which is a bit low anyway (1,024x768). Then click Select to choose the Path Planning simulator. If all goes well, the car will start driving.


## Recorded Demo

I recorded a demo of my code executing, with the car driving without incidents for 9 miles (rubric requires a minimum of 4.32 miles), and passing slower cars by changing lanes whenever it was safe to do so. The car stays within the lane and doesn't get into any collisions. It also doesn't go beyond the acceleration (<=10m/s<sup>2</sup>)and jerk (<=10 m/s<sup>3</sup>) limits. The demo can be seen on YouTube [here]([https://youtu.be/AkXrxMOYvwU](https://youtu.be/AkXrxMOYvwU)).

## Algorithmic solution
