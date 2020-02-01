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


## Algorithmic Solution

First, let's discuss the helper functions in [helpers.cpp]([https://github.com/mkolod/CarND-Path-Planning-Project/blob/master/src/helpers.cpp](https://github.com/mkolod/CarND-Path-Planning-Project/blob/master/src/helpers.cpp)). 

Since the simulator is reporting the ego car's speed in mph but all the (x, y) coordinates and other measurements are in meters, we need a conversion between meters per second and mph. This is covered in `mph_to_mps` and `mps_to_mph`.

Next, we need to determine the safe following and leading distance for the ego car. This will depend on the velocity, because velocity determines momentum, and momentum determines the braking distance. This is implemented in the `physical_braking_distance` function, which is based on the formulas from [Wikipedia](https://en.wikipedia.org/wiki/Braking_distance). The Wikipedia page derives the formula used from the fundamental formulas for kinetic energy (0.5*m*v<sup>2</sup>, where m is mass and v is velocity) and work done while braking (mu * m * g * d, where mu is the coefficient of friction, m is the mass, g is the gravitational constant, and d is the distance). When we equate the kinetic energy with the work, we get the formula for distance: v = sqrt(2 * mu * g * d). The usual value for the coefficient of friction on a dry paved road is about 0.7, so I used that as the default. Total distance traveled while braking is the sum of the braking distance above and the distance traveled due to reaction latency. This could be both data processing latency, or the actuator latency. This is covered in the `reaction_distance` functin. The total distance traveled sums up braking distance and reaction distance, and is implemented in `total_braking_distance`. 

I moved the other functions that came with the project skeleton from `main.cpp` into `helpers.cpp` as well, including the Euclidean distance function `distance`, `deg2rad` and `rad2deg` functions, `ClosestWaypoint` and `NextWaypoint` which provide us with the closest and upcoming waypoints, etc. The same goes for the `getFrenet` and `getXY` functions which convert between Frenet (d, s) and (x, y) coordinates. I will not discuss them here, since they were covered at length in the lectures, and come from the course. I would instead like to focus on the code I added to implement path planning.

Most of the code related to planning can be found in `planner.cpp`. This is implemented as a `Planner` class, to make it easy to handle state. There are some states we want to initialize once and keep around, while others change as the car is driving. You can find these states as class fields listed in `planner.h`. The immutable, initialized at start-up states include `map_waypoints_x` , `map_waypoints_y`, `map_waypoints_s`, `map_waypoints_dx` and `map_waypoints_dy`.  These are passed into the `Planner` constructor and never changed again. Basically, we get waypoint information in `(x, y)` coordinates, and convert them to Frenet coordinates in terms of Frenet distance `s`. We also keep the deltas around as `dx` and 	`dy`, as well as the maximum allowed velocity on this road (`max_velocity`). The variables that change over time and are also encapsulated as class fields include the current lane (`lane`), current velocity (`velocity`), whether we are in the middle of a lane change (`lane_change_in_progress`), the time that elapsed since lane change start (`time_since_lane_change_start`), and the time when the simulation started (`sim_start_time`). 

Still in `planner.cpp`, the `closest_cars_in_lane` method locates the nearest cars to the ego car in a given lane, in terms of the Frenet (s) coordinate. This applies to each lane, not just the one in which the car is traveling. The method collects distances between the ego car's s and the s of the car being examined. The observations are split per lane and according to whether the other car is in front of or behind the ego car. Knowing the nearest cars within the ego car's lane of travel is important, because car in front determines the velocity that the ego car needs to adjust to, as well as to know the velocity of the current lane vis-a-vis the other lanes to determine which lane is the fastest. For lanes other than the one in which the ego car is currently traveling, both the car in frond of and behind the ego car is important. That's because the car behind might for instance be traveling with a higher velocity than the ego car, so we need to predict where that car will be as the ego car changes lanes, to determine if the lane change is safe. The closest car in front is important for the same reason as in the ego car's lane, i.e. to determine the lane's velocity, but also to determine where the closest car in front will be, to determine if the lane change is safe, just as in case of the car behind the ego car.

The `fastest_safe_lane` method ([here](https://github.com/mkolod/CarND-Path-Planning-Project/blob/master/src/planner.cpp#L74:L126) determines which lane is safe to change to first, and then prioritizes them based on the velocity. The lanes that we can choose from are limited, depending on which lane the car is currently in. For instance, being in lane 1 (middle), we can hop over one lane to get to lanes 0 (left) or 2 (right). However, traveling in lane 0 (left), we can either continue in that lane, or move to lane 1 (middle). The restriction here is that we're allowing a lane change by at most one lane - we don't want to hop 2 lanes over, because that will likely be unsafe. Most drivers use the same logic - even if they want to change by a few lanes, they will likely make one safe move at a time. This is hard-coded as a switch-case statement, but could be made more flexible for more lanes. Also, this method could use more refactoring to for instance make the calculation about which lane the car is in into a separate function or method, rather than hard-coding the predicate as `d < (2 + 4 * req_lane + 2) && d > (2 + 4 * req_lane - 2)`. As I will also mention later, there are many opportunities for refactoring, but I was pressed for time to finish. For now, let's focus on correctly functioning code, and talk about how to make it a bit nicer later. I won't list all the refactorings, just the most obvious ones. In the `fastest_safe_lane` method, we determine the safe distance by looking at the total braking distance (braking time and reaction time, with reaction time assuming 0.5 seconds for added safety margin) to determine how far we need to be from the other car. We add this to that car's Frenet s coordinate to determine where the car would be by the time the ego car changes lanes. This provides us with the information whether we are a safe distance away both in front and in the back. 

Note that the planner's [constructor]([https://github.com/mkolod/CarND-Path-Planning-Project/blob/master/src/planner.cpp#L13:L24](https://github.com/mkolod/CarND-Path-Planning-Project/blob/master/src/planner.cpp#L13:L24)) saves the immutable data such as `map_waypoints_x`, as well as the beginning of the simulation (`sim_start_time`). It also sets `lane_change_in_progress` to true. This prevents further lane changes when set to true. The reason that I set `lane_change_in_progress` to true at the very beginning is to prevent the car from immediately changing lanes, before it gathers some data about trajectories of other cars, which is used in any calculation that depends on the `prev_size` argument. The reason here is that without having any history, it would be pretty unsafe to change lanes.

The `plan` method ([here](https://github.com/mkolod/CarND-Path-Planning-Project/blob/master/src/planner.cpp#L126:L310)) largely implements the logic mentioned in the code walk-through found in the Highway Driving Project walk-through and Q&A (section 6 of the project overview). 

