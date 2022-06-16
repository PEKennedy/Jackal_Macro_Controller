# pk_control
catkin package to control the kinova arm, based on a python example from ROS_kortex

To use, run this in 4 separate terminals:

    export ROS_MASTER_URI=http://localhost:11311
    unset ROS_IP

Then Launch each of these, each in their own terminal

Start ROS
    
    roscore

Launch the Simulation
    
    roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3_lite

Start the Joystick    

    rosparam set joy_node/dev "/dev/input/js2
    rosrun joy joy_node

Launch this control program

    roslaunch pk_control control.launch

A future improvement would be to make the simulation + joystick just be
parameters of pk_control control.launch,
reducing this to just roscore and pk_control.

## control_moveit.py
Built from the moveit example, works with gazebo
and the real robot without deprecated functions, 
but doesn't work well with pre-recorded actions.

## control_kortex.py
Example from github which uses an old version of ros_kortex to move the arm.
Uses deprecated function, but does work with both gazebo and the real arm.

** This is the version we use since it works well
with the pre-recorded actions made with the web app

Attempting to add constraints to arm movement breaks
gazebo it would seem.

## arm.rviz
Rviz preset for the kinova arm
