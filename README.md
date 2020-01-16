# Stingray-Simulation

## Installation

### Install ROS Melodic

Follow the instructions below to install ROS Melodic for Ubuntu 18.04.

http://wiki.ros.org/melodic/Installation/Ubuntu 

### Install Gazebo 9

Follow the installation instructions below to install Gazebo 9 for Ubuntu 18.04.

http://gazebosim.org/tutorials?tut=install_ubuntu

### Install ROS package for gazebo

`sudo apt-get install ros-melodic-gazebo-ros-pkgs`

### Install rviz

`sudo apt-get install ros-melodic-rviz`

### Download Stingray Simulation Repo

`cd ~`
`git clone https://github.com/kristinfarris/Stingray-Simulation.git`

## Running the Simulation

### Source ROS

`source /opt/ros/melodic/setup.bash`

### Setup environment variables
For the simulation to work, you have to set up environment variables in whatever terminal you want to run the simulation from. We are working on creating a script to do this automatically so that this is easier. 
Assuming you have placed the repo in your home directory, adjust accordingly if not:
`export GAZEBO_RESOURCE_PATH=~/stingray-simulation/catkin_ws/stingray_sim`
`export GAZEBO_MODEL_PATH=~/stingray-simulation/catkin_ws/stingray_sim/models`
`export GAZEBO_PLUGIN_PATH=~/stingray-simulation/catkin_ws/stingray_sim/plugins/build`

### Run the simulation

`roslaunch stingray-simulation/catkin_ws/stingray_sim/launch/triton_world.launch`

### Send move commands

To make the robot move in the simulation, open another terminal, source ROS, and publish to the vel_cmd topic. To do this, you must tab complete the last portion of the command for it to be formatted correctly and then change the x,y,and theta values to what you would like them to be.

`source /opt/ros/melodic/setup.bash`
`rostopic pub /triton/vel_cmd geometry_msgs/Pose2D <tab>`

We are trying to figure out a way where you don't have to tab complete. Also the theta move command is not implemented yet. 

## Known Issues
- If you run a gazebo instance and then close it, it may still be running in the background. Kill the gzserver and gzclient processes from the terminal if a new instance of gazebo wont load because of this. 
- Sometimes adding/deleting models into the world crashes gazebo. We are still working on solving this. 
