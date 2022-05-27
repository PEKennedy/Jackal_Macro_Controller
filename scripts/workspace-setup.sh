#! usr/bin/bash
echo "Starting the ROS workspace setup script"
echo "make sure you used 'chmod u+x workspace-setup.sh' if this doesn't work"

#make the workspace as per http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make

#add the workspace to terminal start
echo "source devel/setup.bash" >> ~/.bashrc
source ~/.bashrc


