# ROS_RRT
First create your workspace and enter it:
``` shell
mkdir rrt_ws/src
cd ~/rrt_ws/src
```
then clone it:
``` shell
git clone https://github.com/wrld/ROS_RRT.git
```
then make it:
``` shell
cd ~/rrt_ws
catkin_make
source devel/setup.bash
```
then launch it:
``` shell
roslaunch nav_sim myrobot_world.launch 
roslaunch nav_sim move_base.launch
```
pick the goal point, and then you will see the simulation.
