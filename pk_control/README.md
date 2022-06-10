# pk_control
catkin package to control the kinova arm, based on a python example from ROS_kortex

Use with 'roslaunch pk_control control.launch' after building the package with catkin_make.
- 'launch roscore' and 
- 'roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3_lite'
first though

## Control_old
Example from github which uses an old version of ros_kortex to move the arm.
Uses deprecated function, but does work with both gazebo and the real arm.

## arm.rviz
Rviz preset for the kinova arm
