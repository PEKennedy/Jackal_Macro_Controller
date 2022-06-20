# pk_control Package
catkin package to control the kinova arm, based on a python example from ROS_kortex


## Running In Simulation

To use, run this in 4 separate terminals:

    export ROS_MASTER_URI=http://localhost:11311
    unset ROS_IP

Then Launch each of these, each in their own terminal

Start ROS
    
    roscore

Launch the Simulation
    
    roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3_lite


Launch this control program

    roslaunch pk_control control.launch

optional parameters are:
- robot_name (default := my_gen3_lite)
- start_delay_seconds (default :=0)
- use_joy (default :=True)
- joy_index (default :=2)

Start the Joystick    

    rosrun joy joy_node

A future improvement would be to make the simulation + joystick just be
parameters of pk_control control.launch,
reducing this to just roscore and pk_control.
Some work has been done on this

## Run on Jackal (real robot)

To use this on jackal, first move this project onto jackal

    scp -r ~/catkin_ws/src/UNB_HCI administrator@CPR-J100-0574:~/catkin_ws/src

Then build the package on Jackal (not any more, only the first time)

    catkin_make

Launch the kinova arm in one terminal ssh'd into Jackal
    
    sudo ifup br0:1
    roslaunch jackal_kinova_bringup jackal_gen3_lite_bringup.launch

Then in a different ssh'd terminal, launch the control node once the first terminal says "The Kortex driver has been initialized correctly"

    roslaunch pk_control control.launch robot_name:=kinova_arm use_joy:=False joy_index:=0

Note the different robot name to refer to the real arm as opposed to the simulated arm

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
