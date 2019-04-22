# ROS_RRT
## How to use?
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

## Use different algorithm
``` shell
vim nav_sim2/cfg/new.yaml
```
Change the value of **choice** to change the method:
0---rrt based on probability
1---rrt connect
2---rrt*
Change the value of **gravity** to set if using gravity of the goal point:
0---no
1---yes
Change the value of **smooth** to set if smooth the road:
0---no
1---yes
and more parameters to change the steps or probability.

