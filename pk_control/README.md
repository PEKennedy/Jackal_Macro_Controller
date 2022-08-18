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

What did you learn that others will need to know
gotchas
quick code readme
abstract, motivations
things that will break the robot
1 day crash course

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

## Using the default Kinova arm controls
Plug a controller into the micro-usb port, and turn the kinova arm on.

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

## Some basic ROS commands
ROS provides command line tools that are useful for debugging, here is a sample:
For rostopics:
- rostopic list >>> lists all available rostopics
- rostopic info /cmd_vel >>> lists information about a certain topic
- rostopic echo /cmd_vel >>> prints all data being sent to this topic
- rostopic pub /cmd_vel '' >>> publishes data to this rostopic
For rosparams:
- rosparam list
- rosparam get /run_id
- rosparam set /my_param 'Hello'
For rosservice:
- rosservice list
- rosservice info /rosout_get_loggers

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

## Finding services in ros_kortex
In the repo for ros_kortex (https://github.com/Kinovarobotics/ros_kortex)
you will find example code under (https://github.com/Kinovarobotics/ros_kortex/tree/noetic-devel/kortex_examples/src/full_arm)

By running 'rosservice list', you will get a list of rosservices that are available on the kinova
arm. To call the service you need to have some code like this:

    execute_action_full_name = '/path/to/service/like/the/rosservice/list'
    rospy.wait_for_service(execute_action_full_name)
    self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

Notice the "ExecuteAction", as the second parameter of rospy.ServiceProxy. This is a data type
for the rosservice to use. You can find the wanted data type using "rosservice info /path/..."

Then, you go to this part of the repo:
https://github.com/Kinovarobotics/ros_kortex/tree/noetic-devel/kortex_driver/srv/generated

Here you will find the various definitions for these data types. Here's ExecuteAction.srv:
    
    Action input
    ---
    Empty output

The action data type can be found under:
https://github.com/Kinovarobotics/ros_kortex/tree/noetic-devel/kortex_driver/msg/generated

Making the call to the ExecuteAction service involves calling self.execute_action (setup in the
python above) using the correct datatype.

    req = ExecuteActionRequest()
    req.input = res.output
    rospy.loginfo("Sending the robot home...")
    try:
        self.execute_action(req)

But notice how in the actual code, instead of creating an ExecuteAction type, nor even an Action
type, we instead created an ExecuteActionRequest type to send to self.execute_action()?

Well, I don't really know why either, searching the ros_kortex repository only brings up example
code, rather than a definition for the type: https://github.com/Kinovarobotics/ros_kortex/search?q=ExecuteActionRequest
So I had trouble getting most new rosservice types I saw and wanted to use to work (I was only
ever able to add the stop type). For the most part, I would try to create objects like 
req = SendTwistJoystickCommandRequest(), only to either get an error that I wasn't creating the
object correctly, or absolutely nothing (no movement, simulation or on the real robot).
One possibility is that there was a mode switch that needed to be made in order to use the twist
or velocity controllers. At line 340 of old_code.txt, you will find an attempt to use a service
I found promising for doing a mode switch, but found no success in using.

Here's some official documentation of rosservices: http://wiki.ros.org/rospy/Overview/Services

If you ever are able to get extra rosservices working, then one improvement you could consider
making is using the TwistJoystick to recreate the default control scheme for the kinova arm*, and/or
use a velocity command to create a continuous smooth motion for the circular motion sequence
in this project. Recreating the default control scheme means that you can control the arm remotely
from the computer, rather than having a wired connection. It further means switching control schemes
could be made much simpler (button press to switch, rather than stopping the control_kortex.py
controls, then starting something else)

## installing ROS, ros_kortex

See the .sh files in UNB_HCI/scripts, these bash scripts aren't tested, but should at least
demonstrate most of the commands that need to be run for setup.

See Starting a Ubuntu-ROS env.docx for some further steps, though the document does quickly just
become notes.

# The experiment and research
## experiment outline
The goal of the project is to see if a simplified robot control scheme (only some simple pre-planned
movements) can perform better than a more complex one with more exact controls. The hope
is that a user will be able to combine the provided actions in unique/creative ways to achieve
complex goals.

To do this, users will complete several tasks (ex. turning a crank, moving and rotating objects)
using two different control schemes: 
- the reference control scheme which is built into the kinova arm (users use the sticks to
control the x+y+z axis velocity of the arm, and rotation)
- Our simplified control scheme which has pre-planned actions built into the buttons (ex.
circle button to grab, put back, and poke) 

The tasks will be timed, have the number of mistakes recorded, and have user opinions recorded to
provide an objective measure of how the two different control schemes compare.

## The experiment rigs and tasks
The following are the rigs I built/materials I gathered for the experiment:
- Plastic bucket with foam blocks
- Inverted bicycle with wooden handle
- Open box with clamps (6x) and spare boards to form variable sized openings
- Vertical board with 3x3 grid of circular holes

The following were the initial ideas for tasks to do:
- How long does it take for a user to pour the foam blocks out of the bucket
- Can the user turn the crank 360 degrees when facing the crank from the front? from the side?
- Have the user pick up blocks from one table and put them down on another at a different height
- Have the user pick up a block, and rotate it to fit in a small opening presented by the open box.
- Have the user enter a number combination on the 3x3 grid of holes (somewhat like entering a phone number) 

The tasks were not picked with the new control scheme being able to do them in mind (that would be
unfair to the reference), but as challenging tasks which requires a full use of the controls,
and possibly inventing out-of-the-box solutions if the pre-planned actions are insufficient at
face value.

## The pre-planned actions control scheme thus far
The primary idea of the pre-planned actions control scheme is that users will have a very simple
control scheme and be able to combine actions to achieve complex goals. To this end, users
should be able to do simple modifications to the actions.

The control scheme currently has the following button layout:
- O >>> moves the arm forwards, and grabs or opens based on current gripper position
- O (2x) >>> opens then closes gripper, moves arm forwards to "poke"
- X >>> conducts a "screwing" motion (rotate an object about our "y" axis), then drops
- Square >>> cancels the current motion
- Triangle >>> Moves the gripper up (cycling between 3 preset heights)
- Triangle (2x) >>> Moves the gripper down (^^^)
- Share >>> rotates the gripper between a horizontal and vertical orientation 
- Share (2x) >>> moves the gripper around in a circle (about the vertical and side-to-side plane in front of the robot)
- Option (2x )>>> moves the gripper around in a circle (opposite direction)
- Left Stick button >>> sends the arm to the default position
- Left Stick button (2x) >>> sends the arm to the home position and then the default position
- Right Shoulder button + Left stick still controls Jackal

Some expected combinations are that the user can use the circle or height changes to change 
where the arm is when the do a grab or put back motion. One of the things Daniel would like to see
are perhaps orientation changes ("I want grab down/up"), for additional flexibility. This would 
likely involve changing the relative movement code (see go_to_cart_pose_relative at line 598),
which presently does a subtraction on positions to find relative movement (allowing different
heights, but never different directions). A new step would need to be added here which rotates
the relative movement vector to face the direction of the gripper.

One of the issues with adding or modifying actions is that the arm can easily get stuck.
What happens is that the arm's software finds the arm can't reach a position, and so stops the arm,
however, it doesn't send any notification (that I know of) about this, and continues "trying"
to complete the sequence. The only solution for the user presently is to use Square to cancel
a motion, and then left stick button to reset. If square is not pressed, then the moment the
arm starts moving back to home, the sequence will "overlap", and move the arm into an unpredictable
position. The most common way of getting the arm stuck this way is to use the screw/RotY motion,
especially when its position is modified from the default position.

To try and mitigate or fix the stuck issues, the first place to look in control_kortex.py is around
line 90 with min_dist_bound. The distance bounds will attempt to keep the arm from going too far
in any direction, thus avoiding many issues with getting stuck by the controller trying to put the
arm in a position too far for it to reach. The biggest improvement that could be made would be
to reach the orientation of the gripper, and shorten the distance bounds based on this. 

## REB approval and consent forms
This project has obtained REB approval (REB 2022-113). Consent forms to hand out to participants
are available?? TODO

You will however, need to update the consent forms to reflect primary investigators and phone numbers.

## Things to research for the paper
One part of writing a paper is the related work section. This section is used to show where
in the field (HCI) this research is (ie. where and how does it relate to other research, what
is the significance of it?). However, I wasn't able to find much that was very related to this
project.
TODO
I found the following interesting:
-x
-y
-z

Perhaps the following are areas you could research:
-x
-y
-z

What has been done is a few abstract drafts, the rest is up to you.




