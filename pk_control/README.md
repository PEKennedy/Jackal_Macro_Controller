# pk_control Package
catkin package to control the kinova arm, based on a python example from ROS_kortex


## Running In Simulation

To use, run this in 4 separate terminals ** not anymore:

    export ROS_MASTER_URI=http://localhost:11311
    unset ROS_IP

Then Launch each of these, each in their own terminal

Start ROS
    
    roscore

Launch the Simulation
    
    roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3_lite start_rviz:=false


Launch this control program

    roslaunch pk_control control.launch

optional parameters are:
- robot_name (default := my_gen3_lite)
- start_delay_seconds (default :=0)
- use_joy (default :=True)
- joy_name (default :="js2")

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

    roslaunch pk_control control.launch robot_name:=kinova_arm use_joy:=True joy_name:="ds4x"

Note the different robot name to refer to the real arm as opposed to the simulated arm and the changed joystick name
If you need to find what joystick you should use:

    ls /dev/input

To display the available gamepad/joystick file descriptors, then

    jstest /dev/input/>>joystick or gamepad<<

to view the inputs coming to that joystick, test different ones until you find the one which corresponds to your controller.
On Jackal this was ds4x, on my pc it was js2.
Beware with bluetooth controller, it can disconnect whilst still on if not used in a while. If this happens, hold psbtn+share until blue light goes off,
then hold psbtn again to rebind.

remember to run the joy node
Start the Joystick    

    rosrun joy joy_node

Also: To view cameras, do the following in a new terminal

    export ROS_MASTER_URI=http://cpr-j100-0574:11311
    python3 ./pk_control/src/viewer.py




## control_kortex.py
Example from github which uses an old version of ros_kortex to move the arm.
Uses deprecated function, but does work with both gazebo and the real arm.

** This is the version we use since it works well
with the pre-recorded actions made with the web app

Attempting to add constraints to arm movement breaks
gazebo it would seem.

To add a sequence, use the kinova webapp, where you can record positions, and then combine
them into sequences. The very first position in a sequence will be considered the origin coordinates
the rest of the action is relative to, and will not be played.

## viewer.py
Based on Aditya's axis_camera.py, this program uses opencv and tkinter to provide a couple windows,
one with the forward facing flir camera (in colour mode), and the rear facing axis camera 

## control_moveit.py
Built from the moveit example, works with gazebo
and the real robot without deprecated functions, 
but doesn't work well with pre-recorded actions.
Stopped development of this version, use control_kortex.py instead

## arm.rviz
Rviz preset for the kinova arm, however, Rviz isn't really needed for this project
