#! usr/bin/bash
echo "Starting the ROS installation script"
echo "make sure you used 'chmod u+x ROS-installer.sh' if this doesn't work"

#follow the install scripts seen on http://wiki.ros.org/noetic/Installation/Ubuntu
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop

#setup the script sourcing for bash terminals (still ros's wiki guide)
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

#some ros packages

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

sudo apt install python3-rosdep

sudo rosdep init
rosdep update

#fix a catkin_make not working problem
#sudo apt install python3-catkin-tools python3-osrf-pycommon

source ~/.bashrc
