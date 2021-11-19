[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
# turtlebot_obstacle_avoidence
A simple project for turtlebot to detect and avoid obstacles
## Dependencies

-   ROS Melodic : installation instructions [here](http://wiki.ros.org/melodic/Installation/Ubuntu)
-   Ubuntu 18.04
## Steps to run

Make a catkin workspace catkin_ws and run the following commands :

```
cd <path_to_ws>/catkin_ws/src
git clone https://github.com/aditiramadwar/turtlebot_obstacle_avoidence.git
cd ..
catkin_make
```
### Simulation

    roslaunch turtlebot_obstacle_avoidence move.launch
    
#### Enable rosbag recording:
By default the rosbag recording is *disabled*, to enable the recording, run this:

    roslaunch turtlebot_obstacle_avoidence move.launch record:=true

#### Inspecting the recorded rosbag file: stop the launch file and run this,

    rosbag info results/topics.bag
   
#### Play back the bag file with the Listener node demonstration: 
1. Stop the launch file and, Terminal 1:
```
    roscore
```
 2. Run the listener node, Terminal 2:
```
    rosrun turtlebot_obstacle_avoidence move
```
  3. Play the recorded rosbag, Terminal 3:
```
    rosbag play results/topics.bag
```

