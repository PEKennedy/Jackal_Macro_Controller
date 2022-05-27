#! usr/bin/bash
echo "Starting the Jackal and Kinova gen 3 lite setup script"
echo "make sure you used 'chmod u+x robot-setup.sh' if this doesn't work"



#setup the source as per http://wiki.ros.org/ClearpathRobotics/Packages
wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -
sudo sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'
sudo apt-get update

#setup jackal as per https://www.clearpathrobotics.com/assets/guides/noetic/jackal/simulation.html
sudo apt-get install ros-noetic-jackal-simulator ros-noetic-jackal-desktop ros-noetic-jackal-navigation

#get gen3lite firmware here: https://github.com/Kinovarobotics/kortex#download-links


#env setup modified to use catkin_ws instead of catkin_workspace from https://github.com/Kinovarobotics/ros_kortex

sudo apt install python3 python3-pip
sudo python3 -m pip install conan
conan config set general.revisions_enabled=1
conan profile new default --detect > /dev/null
conan profile update settings.compiler.libcxx=libstdc++11 default
#mkdir -p catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Kinovarobotics/ros_kortex.git
cd ../
rosdep install --from-paths src --ignore-src -y
catkin_make
source devel/setup.bash
