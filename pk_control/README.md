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

Also: To view cameras, do the following in a new terminal (local to your machine)

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
Built from the moveit example, works with gazebo and the real robot without deprecated functions, 
but doesn't work well with pre-recorded actions.
Stopped development of this version, use control_kortex.py instead

## arm.rviz
Rviz preset for the kinova arm, however, Rviz isn't really needed for this project

## old_code.txt
Old code that ended up not working or simply unused in control_kortex.py, kept for reference just
in case.

# Notes on how to use the Jackal+Kinova gen3 lite setup
## Battery and power on
Jackal is powered by a single large battery. Find the axis camera (small camera mounted fixed to the jackal)
and pull up on the Jackal's metal lip. Inside will be the battery housing.
There is a battery charger at the back of the HCI room for it, don't leave the battery to charge
over night (especially over the weekend).

With the battery connected inside the Jackal, press Jackal's power button to turn it on. 

## Connecting to and controlling Jackal with a controller
Get the bluetooth PS controller and hold the PS button to start the bluetooth binding process
To unbind, hold the PS button + Share, eventually the blue "connected" light will turn off.
Hold the right shoulder button and move the left stick to drive the jackal.

## Turning on the kinova arm
Flip the power switch to on, hold the arm in a position where it isn't stuck on anything until
the power light turns from blue to green (and the arm goes stiff).

When powering off, set the arm down in a safe position where it won't move.

## Using the Kinova arm's webapp
At the base of the kinova arm, there is a micro-usb port, unplug the Jackal's connection to it
and plug your own computer into this port.
Turn the kinova arm on.
Then, in a browser, go to 192.168.1.10 to see the webapp. Login is admin, (ask)
The webapp allows you to control the arm, and record sequences.

## Connecting to Jackal over SSH
Follow the steps in the Jackal + unb007 manuals for setting up the ip of cpr-j100-0574.

To connect to Jackal over SSH, you will need a computer with wifi, and a router setup in the HCI
lab. Then in the terminal, run 'ssh administrator@cpr-j100-0574', password ... (ask).

# In case no SSH connection can be established...
Bad news, but fixable.
Bring the Jackal to a screen with HDMI input (such as the TV screen in the HCI lab)
Open up Jackal
Inside the opened Jackal, find on the top a couple of circular nuts. Un-screw them.
This will open the computer bay of the Jackal. At the back, there are USB ports (for keyboard)
and a free HDMI port for the TV. Find something to keep the lid propped open.

log into administrator, and have another computer on hand to search up solutions.
Some places of interest are systemctl, iptables and firewalls.

When you figure out the problem, make sure your solution persists, and put the nuts back.

When this problem was encountered before, our solution was to just accept all network traffic,
hopefully a better solution exists.

# In case no bluetooth connection can be made...
Check bluetoothctl, what devices are available?
Find your device's MAC address, and make sure it is set to trusted, and try connecting to it
from Jackal's end.

## Basic overview of ROS
ROS ("Robot Operating System") is software which sits above the real operating system meant for 
controlling robots.

The main architecture is as follows: various scripts/programs ("nodes") can run on a network of
any number of computers/robots. "nodes" will read and write data to what are essentially 
variables/data streams ("topics"). All nodes can see all available topics on your network
can read/write to any topic.

ROS also has "rosparam" and the parameter server (essentially global variables used for configuration).
ROS also has "rosservice"s, which is a way for a node to provide a function for other nodes to use.

Since we only have 1 computer and 1 robot, this means in practice that you will be running
the main program on the Jackal, it will call the kinova arm's rosservices to move it around,
and subscribe to a few of the kinova arm's topics to gather data whenever a rosservice doesn't
do the job. On your computer, a program will be running to listen to the camera topics and display
the camera feeds on the screen.

On one of the machines, you will also have a "joy" node for receiving controller inputs. 

## More ROS stuff you should know...
roslaunch, .launch files, compiling, catkin, rviz, gazebo etc.
ROS nodes are typically started by the use of the roslaunch command line program. These take the
following form:

    roslaunch package_name launch_file.launch param1:=True param2:="Hello"

The launch files can be used to specify default parameters, set rosparams (ie. global configuration),
and launch multiple nodes at once if your node needs them (though I never got this to work)
In this project, control.launch is used to load the combined_sequences.json file into rosparam's
parameter server so control_kortex can use the different sequences, and to tell the program
whether or not to play a demo sequence, or use the controller.

To use roslaunch though, you must build a ros package (what pk_control is). To do this use
catkin_make at the root of your workspace. For setting up a catkin workspace, see the ROS
tutorials online (http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
Running catkin_make has already been done on Jackal, and does not have to be done when you make
further changes to a package.

There are 2 main ways you will run ROS packages, one is on the real robot you are programming for,
the other is on a simulator. ROS provides a simulator known as gazebo.

Separately from gazebo, ROS also provides a program called RVIZ which reads ROS data to display
what the robot can actually "see"/sense. Since it just reads ROS topics, it works on both gazebo
and the real robot. A configuration for rviz is provided in this package, but is unused and 
unneeded.

## The network bug
One of the persistently frustrating parts of development is a particular network bug at UNB.
Symptoms are:
- Can SSH onto Jackal, publish to nodes such as /cmd_vel, and echo nodes such as /tf just fine.
- Can rostopic list on your UNB machine just fine.
- Cannot rostopic pub to nodes such as /cmd_vel from your UNB machine.

From Clearpath support, this is something to do with UNB blocking needed network packets when sent
from your computer, not over ssh. It is possible to play with network and network adapter settings
enough to avoid the issue (I managed it briefly), but it likely isn't worth the hassle. You should
probably just bring a non-UNB laptop instead and use that, or work around the issue. But if you do
decide to try to solve the issue, it likely involves cutting at the network adapter level all
connections to the UNB network (ethernet), plus a few steps.

Due to me being stuck using a UNB machine, there were some things I wasn't able to test (Aditya's
project). Further, I tested my code with the PS controller connected to the Jackal through bluetooth
which could sometimes be unreliable compared to a wired connection to the computer.

## Basic overview of ros_kortex, and alternatives
You have a few alternatives for programming the kinova arm in ROS:
- ros_kortex, python3 or C++, using the kortex api or the moveit api.
- using the kortex api directly without the ros_kortex layer, python3 or C++

Kortex is the library made by kinova for controlling the kinova gen3, gen 3 lite, etc. arms
ros_kortex is a layer on top of this to make it ros compatible. It doesn't mean that
you can't use ROS and the direct kortex api together, merely that the function calls to the arm
will be made through rosservices and data available through the ros nodes rather than only the kortex
api.

The moveit api is the standard ros api for controlling generic robot arms.

More or less I had originally intended to use the moveit api since to move the arm I wasn't
forced to choose between cartesian waypoints (not supported in the gazebo simulation) and deprecated
functions. However, I eventually ran into a snag with real time control of the arm, whose separate
library I wasn't able to get working with the kinova gen3 lite.

I have not used any of the C++ versions, nor the kortex api so have no comment on those methods.
Aditya's experiment uses the kortex api, so that's a good starting point for reference.

## Finding things in ros_kortex



## installing ROS, ros_kortex



## Difficulties in programming for the kinova arm


# The experiment and research
## What is the goal/question asked? 



## The experiment rigs



## REB and consent forms



## outline of the experiment



## Things to research for the paper











