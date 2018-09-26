# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner receives should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

***

[//]: # (Image References)

[image0]: ./writeup/5_20mi_in_traffic.PNG
[image1]: ./writeup/5_56mi_at_speed.PNG

## Rubric Points

#### The code compiles correctly.
The code uses cmake to compile, it can be compiled on any system.

#### The car is able to drive at least 4.32 miles without incident.
The car has completed over 4.32miles of driving without incident:
![5.2 miles driving in traffic][image0]
![5.56 miles driving at speed][image1]

#### The car drives according to the speed limit.
The car is programmed to drive at 49.5mph unless otherwise inhibited by other cars. 

#### Max Acceleration and Jerk are not Exceeded.
The car doesn't exceed a total acceleration of 10 m/s^2 or a jerk of 10 m/s^3.

#### Car does not have collisions.
The car has a safety radius that it checks before changing lanes and slows down if it gets too close to another car to help it avoid collisions. 

#### The car stays in its lane, except for the time between changing lanes.
The car is programmed to stay in its lane unless passing another.

#### The car is able to change lanes
The car changes lanes when it senses that the car(s) in front of it are not going the speed limit and there aren't cars in the other lane that will further slow it down or collide with it.

#### There is a reflection on how to generate paths.
I’ve chosen to use a couple simple heuristics to generate paths for this project, proximal distance to cars in the speed of the self-driving car’s (ego’s) lane and the lane it intends to change into, and speed of cars in ego’s lane and adjacent lanes.

Proximal distance is checked first for cars in ego’s lane (main.cpp line 368), where if there is a car that ego is approaching it will start slowing down until it is within a safe following distance, where it will match the car’s speed (main.cpp lines 429-443). Other cars that are close to ego are stored and used if ego determines that it is advantageous to change lanes based on a simple cost function for speed (main.cpp line 87). If ego wants to change lanes the safe_to_change_lanes (main.cpp line 98) function is called and checks that there aren’t any cars in the intended lane within a given distance before allowing ego to change lanes. 

The speed cost function is also simple in that it checks the speeds of the car in its lane and adjacent lanes and determines which lane is going closest to the speed limit. If an adjacent lane is faster than ego’s current lane it will attempt to change lanes, given that it passes the safety function (main.cpp line 400-425).

Determining when to stay in the current lane or change lanes is only have the battle. A path has to be developed and laid out for ego to follow. 

Following the example in the walk-through provided in the classroom I decided to download the spline library to use for path generation. I create a vector of points created from the last path and three more waypoints beyond that to create the spline. After that I map 50 points onto the spline based on the distance I'm projecting out (the variable I named "radius") and the reference speed (ref_speed) to get the desired reference speed and path of the vehicle. 

I did look into other methods to fit a path, such as JMT from class and Bezier curve fitting with control points. I had more success with the Bezier curve fitting than JMT due to the way the data came in from the simulator. Ultimately I chose to use the spline library for ease of use and a lack of time to develop the Bezier curve fitting functions. 