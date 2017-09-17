# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Project summary
Path planning is one of the hardest problem in autonomous vehicles (AV) and the complexity of it made deciding which parts to tackle first a little tricky. The highway simulator provides two primary data feeds: a processed sensor fusion feed that contains the cartesian / fernete coordinates for other vehicles on the road and localization data for our own car. I decided to go with basic lane keeping as a starting point and began working on path planning for a single lane around the track (ignoring other vehicles and not attempting lane changes).

Controlling the AV in the highway simulator is done by sending a path (an array of cartesian coordinates) to the simulators version of the CAN Bus. The vehicle's speed and acceleration are derived from the distances between each point so the coordinates must be generated to avoid jerk and follow the speed limit.

While the simulator takes Cartesian coordinates it's a lot simpler to do a lot of the planning work in fernete coordinates and then transform them after. Let's say we have a target lane we want to keep and we store that in a variable lane with values 0-2 (for the 3 lanes on our side of a 6 lane highway). If each lane is 4 meters then we can get the D coordinate for the center of the target lane by doing 2+lane*4.

Now we need to pick a few S coordinates (starting at current S) to use for fitting a spline to the planned path. In order to spread lane change maneuvers out smoothly (and avoid jerk) I chose to use 3 S coordinates at 45 point increments from the cars current (S+45, S+90, S+135). Now that we have 3 sets of fernete coordinates we can convert them back to cartesian coordinates before fitting our spline.

While we could've fit a polynomial to the points I decided to use a spline tool as it made the task simpler and unlike polynomials the spline is guaranteed to go directly through our target coordinates. Because acceleration is derived from the distance between path coordinates we then select points along the spline in accordance with our jerk minimization and target velocity goals.

At this point we can plan a path for our AV around the highway track but we’re ignoring all the other cars on the road so we’ll slam into cars in our lane and don't know how to change lanes yet to avoid them. In order to make decisions based on the highway traffic we need to parse the sensor fusion data and calculate the variable trajectories for nearby vehicles. We also need to keep track of our own vehicles intention using a finite state machine with four distinct states: keep lane, prepare lane change, change lanes left, change lanes right.

At each app cycle the sensor fusion data is processed so we can track the location of nearby vehicles. The vehicle data also includes everything we need to know in order to calculate its trajectory at a future state: cartesian coordinates, fernete coordinates, and current x/y velocity in m/s.

When processing the vehicles I first narrow it down to only nearby vehicles by looking at the delta between our AV cars S coordinate and the vehicles S coordinate. I then check if there are any vehicles in our current range within the immediate path for lane keeping, changing into the left lane, or changing into the right lane.

If there's no vehicles in our current lane that's great, don't make any lane changes and stay close to the target trajectory (< 50 mph). If there is a vehicle in our current path we decrease the reference velocity sequentially (-.3 m/s per planning cycle) so as to avoid jerk but keep a safe distance. Avoiding a front collision and keeping a safe space between you in the lead car is paramount (whoever wrote the vehicle AI made them do brake checks too :D).

If we are stuck behind a car that's slowing us down below the target trajectory we then need to try to safely change lanes. To change lanes we'll need a big enough gap between other cars so that we can make the move without creating jerk. Because we're recycling any unexecuted path points from the previous cycle a future lane change will be executed starting at the end of the recycled path. That means in order to make sure we have enough room to make the lane change we have to calculate the trajectory of other vehicles at our own vehicles future state. Additionally we need to make sure we can account for a sudden acceleration or deceleration from those cars that might close the gap and cause a collision. To handle these cases I calculated the variable trajectories for each car at +3 m/s and -3 m/s from their current speed. 

But what if we’re in the center lane and have the option to change lanes in both directions, how do we decide which way to go? In this scenario we look ahead beyond the trajectory window needed for the lane change and search for the closest cars S coordinate in both lanes. Whichever lane has the furthest distance to the next car will offer us the best chance to maintain our target velocity and a higher likelihood of maneuverability if we need make a subsequent lane change.

There's a lot of opportunities to improve on this approach to path planning but I felt like this solution was the best practical fit for accomplishing the rubric and keeping it simple. The areas I'd like to spend some more time on in the future are more complex lane change trajectories incorporating accel / decel to fit in tight windows. Additionally developing cost functions for the finite state machine to make decisions rather than hard coding decision trees would make a more flexible path planning system.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

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
