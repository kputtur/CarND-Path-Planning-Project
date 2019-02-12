# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program


## Introduction
   
**[Path planning](https://en.wikipedia.org/wiki/Motion_planning)** systems enable an autonomous vehicle to generate safe, drivable [trajectories](https://en.wikipedia.org/wiki/Trajectory) to get from one point to another. Given information about it's environment from computer vision and sensor fusion such as  vehicle detection and localization and a destination, a path planner will produce speed and turning commands for control systems to actuate.

![Highway Path Planning][path-planning]

A path planner includes mainly three components: a vehicle predictor, a behavioral planner and trajectory generator. **Vehicle Prediction** invloves estimating twhat other vehicles in the local environment might do next. **Behavioral Planning** decides what action to take next, given the end destination and the estimate from the vehicle prediction step. **Trajectory generation** computes an actual path to follow based on the output from the behavior planning step.


This repostirory contains a vehicle planner implementation which navigates a simulated vehicle around a virtual hightway with other oncoming or same direction traffic. The vehicle attempts to keep a close to the speed limit (50kmph) as possible, while maintaining the safe driving distances from other vehicles and from the side of the road as well. This also involves comfortable ridiing experience without any jerks. This involves swithcing lanes, passing vehicles, speeding up and slowing down.

Techniques used while designing 
*  Vehicle Tracking
*  Lane change decision making
*  speed control
*  Trajectory generation


[//]: # (Image References)
[path-planning]: ./images/pathplanning.png
[highway]: ./images/diagram.png
[result]: ./images/myvideo.png
[youtube]: ./images/youtube.png

### Technologies used

* C++
* uWebSockets
* Eigen
* Cubic Spline Interpolation
* PID control


### Repository Contents

* [README.md](README.md) - this readme
* [main.cpp](src/main.cpp) - program entry point; communicates with simulator, executes path planning algorithm
* [spline.h](src/spline.h) - spline interpolation
* [PID.cpp](src/PID.cpp) - basic PID controller


### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)]

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

## Running code from this repository

Running the code in this repository requires the Udacity Term 2 Simulator contents to be properly installed. Click [here](https://github.com/udacity/self-driving-car-sim/releases) for details. This will include installing the simulator itself and the uWebSocketIO library.

Once that is complete,
```sh
mkdir build && cd build
cmake .. && make
./path_planning
# in a separate terminal, start the simulator
```

## Highway Path planning

![Path planning diagram][highway]

### Search and Cost Functions

The fundamental problem in path planning is to find the optimal path from the start to the goal, given a map of the world, locations of all objects in the map, a starting location, a goal location, and a [cost function](https://en.wikipedia.org/wiki/Loss_function). The "cost" in this case refers to the penalty involved in taking a particular path; for example, driving primarily on two lane 30 MPH roads to navigate between New York City and Los Angeles would have a high cost; most of the driving instead should occur on Interstate highways, which would have a much lower cost. Various actions that the vehicle can take incur different costs; left turns, right turns, driving forward, driving backward, stopping, and other actions will all incur costs, which may vary depending on the current speed or location of the vehicle.

Various [algorithms](https://en.wikipedia.org/wiki/Motion_planning#Algorithms) exist for searching for a path; [grid](https://en.wikipedia.org/wiki/Motion_planning#Grid-based_search)- and [interval](https://en.wikipedia.org/wiki/Motion_planning#Interval-based_search)-based search algorithms (such as [A*](https://en.wikipedia.org/wiki/A*_search_algorithm)) are popular, and many other algorithms involving [rewards](https://en.wikipedia.org/wiki/Motion_planning#Reward-based_algorithms) or [probabilistic sampling](https://en.wikipedia.org/wiki/Motion_planning#Sampling-based_algorithms) exist as well.

### Prediction

Prediction is a mechanism which forecasts what other vehicles does in a complex environment.For example, when a vehicle is detected by the radar/lidar or perception systems, the prediction system must be able to answer "where will be the vehicle in next 5 seconds". This happens by taking as input a map of the world and data from sensor fusion such as RADAR or LIDAR data and then generating the future state of all other vheicles and moving objects in the local environment. Moving objects in this case could be animals or pedestrians as well. Prediction system generate [multi-modal probability distributions](https://en.wikipedia.org/wiki/Multimodal_distribution), which means that the estimates they create may have multiple likely outcomes. Information about what vehicles generally do at certain locations in the world, coupled with the current and recent past behavior of a vehicle, inform the prediction to make certain outcomes more or less likely. For example, a car slowing down as it approaches a turn increases the likelihood of a turn.

### Behavior Planning

Behavior planning answers the question of "what do to next" in an autonomous vehicle. Behavior planners use inputs from a map, route to destination, and localization and prediction information, and  output a high-level driving instruction which is sent to the trajectory generator. Behavior planners do not execute as frequently as other autonomous vehicle systems (such as a sensor fusion system), partially because they require a large volume of input to generate a plan, but also because their output plans will not change as frequently as outputs from other systems. A behavior planner acts as a navigator, whose output might be "speed up", or "get in the right lane", including behaviors which are feasible, safe, legal, and efficient.


### Trajectory generation

A trajectory generator creates a path that the vehicle should follow and the time sequence for following it, ensuring that the path is safe, collision-free, and comfortable for the vehicle and passengers. This happens by using the current vehicle state (location, speed, heading, etc), the destination vehicle state, and constraints on the trajectory (based on a map, physics, traffic, pedestrians, etc). Many methods exists for this: combinatorial, potential field, optimal control, sampling, etc. For example, Hybrid A* is a sampling method, which looks at free space around a vehicle to identify possible collisions, and is great for unstructured environments such as parking lots (without many constraints).
Behavior planners often use [finite state machines](https://en.wikipedia.org/wiki/Finite_state_machine), which encode a fixed set of possible states and allowable transitions between states. For example, when driving on a highway, the states might include "stay in lane", "change to right lane", or "get ready for left lane change". Each state has a specific set of behaviors; for example, "stay in lane" requires staying in the center of the lane and matching speed with the vehicle ahead up to the speed limit. Selecting a new state from a given state requires a cost function, which allows for selecting the optimal state for driving to the goal. Cost functions often include as inputs current speed, target speed, current lane, target lane, and distance to goal, among others.


### Implementation and Goal
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


### Vehcile Prediction

The [vehicle tracking](src/main.cpp#L226-L275) section of code is responsible for using data from the sensor fusion measurements to determine where other vehicles are relative to controlled vehicle. This includes detecting the vehicles directly to the left, ahead and right and determining how much empty space exists between these vehicles. This information is used for lane change decision making system.

### Behavioral Planning - speed control - lane change decision

The [lane change decision making](src/main.cpp#L278-L291) system determines if the vehicle should attempt to change lanes if it is currently travellling at less than 90% of the maximum speed (50 MPH) and the vehicle is following less than 50 units behind the vehicle in front of it. Once that determination is made, the vehicle looks to both the left and right for an opening with no other vehicles. In case both lanes are open then the lane with farthest vehicle in that lane is chosen.


### Trajectory generation

Once we finalize on the target lane and at what velocity car should move, a trajectory is calculated several steps into the future [trajectory generation](src/main.cpp#L23-L121). During the first run, the vehicles' location and previous point using the vehciles' yaw angle are computed. These points are converted to x-y co-ordinates, and they are angle corrected for vehicles orientation in the environment. Then the spline interpolation is used for computing the additional trajectory points which when combined with any previous trajectory points from a previous iteration not traveled, to bring the total number of points upto 50. These points are then handed off to the simulator for the vehicle to follow.


## Final Video

Using the vehicle prediction, behavioral planning and trajectory generation and the input data from the simulator and existing map data. The program is able to drive the simulated vehicle safely and efficiently around the loop maximizing the speed while avoiding the collisions. 

[thumbnail](http://i3.ytimg.com/vi/gitIcOqQcXI/hqdefault.jpg)

[youtube link of the video](https://youtu.be/gitIcOqQcXI) 




#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.## Basic Build Instructions

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

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

