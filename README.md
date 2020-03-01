# Outline
- Installation
- Running the Simulation
- Using the Simulation



# Installation

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

### Install Python packages

`pip install pynput`

### Install Other Dependencies

`sudo apt-get install ros-melodic-depthimage-to-laserscan ros-melodic-gmapping`

### Download Stingray Simulation Repo

`cd ~`

`git clone https://github.com/kristinfarris/Stingray-Simulation.git`

### Run catkin_make

`cd ~/Stingray-Simulation/catkin_ws`

`catkin_make`



# Running the Simulation

Before running the simulation, you must first setup your environment. These setup steps must be taken each time you open a terminal that you want to launch the simulation from.

## Setup

### Source ROS root

`source /opt/ros/melodic/setup.bash`

### Source Stingray Simulation Package and Setup

`source ~/Stingray-Simulation/catkin_ws/devel/setup.bash` 

`source ~/Stingray-Simulation/stingray_setup.bash` 

## Running the Base Gazebo World

To run the simulation with the default world file, which is empty_world.world, run the following. This world is empty other than the robot.

`roslaunch stingray_sim gazebo.launch`

## Running other Gazebo Worlds

Running another gazebo world is as easy as passing a world argument to whatever launch file you are launching. There is an example world "triton_world.world" in the repo and the example argument passing shown below:

`roslaunch stingray_sim gazebo.launch world:=triton_world.world`

All worlds run this way need to be placed in the worlds/ directory of the stingray_sim package. 



# Using the Simulation

## Sending Move Commands

To make the robot move in the simulation, open another terminal, source ROS root like above, and publish to the /triton_lidar/vel_cmd topic. To do this, you must tab complete the last portion of the command for it to be formatted correctly and then change the x, y, and theta values to the desired setpoints.

`rostopic pub /triton_lidar/vel_cmd geometry_msgs/Pose2D <tab>`

You can press Ctrl+C to end the current message and can retype the command to send a new one. 

## Moving the Robot with Keyboard Control

To make the robot move using your keyboard, open another terminal, run the sourcing steps as explained in the 'Running the Simulation' Setup section, and run:

`rosrun stingray_sim teleop_robot.py`

You can now use the keyboard to control the robot with WSAD/Arrow keys to translate and with Q & E to rotate it. The input from the keyboard is converted to a ros message sent to the same /triton_lidar/vel_cmd topic.

## Visualizing Data

To visualize data, we will use RViz (ROS Visualizer).

In another terminal, open RViz (ROS visualizer) to view ros data:

`rviz rviz`

### Visualizing Camera Data

In RViz, alter the Global Options Fixed Frame paramenter to be 'triton_link'

Add an Image to the displays with the 'Add' button on the lower left of the screen. Configure the Image Topic to be '/camera/color/image_raw'. You should be able to see an image of the robots view in the window in the lower left.  

### Visualizing Point Cloud Data 

In RViz, alter the Global Options Fixed Frame paramenter to be 'triton_link'

Add a PointCloud2 to the displays with the 'Add' button on the lower left of the screen. Configure the Topic to be 'camera/depth/points'. 3D Point cloud data should now be up showing up in the 3D pane.  

### Visualize Path Data

In RViz, alter the Global Options Fixed Frame paramenter to be 'odom'

Add a Path to the displays using a similar method to the Odometry. Configure the Topic to be '/triton/path'. You should see green lines that depict the path the robot has followed since startup.  
<!---
### Visualize Odometry Data

In RViz, alter the Global Options Fixed Frame paramenter to be 'triton_link'

Add an Odometry to the displays with the 'Add' button on the lower left of the screen. Configure the Topic to be '/triton/odom' and Keep to be 1 (or any value you wish, depending on how much history of odometry data you would like to show). You should see red arrows that depict the location and rotation of your robot in the simulation.  

### Visualizing SLAM Data

To run the simluation with slam mapping enabled in the empty world:

`roslaunch stingray_sim slam.launch`

To run the simluation with slam mapping enabled in a custom world:

`roslaunch stingray_sim slam.launch world:=<world name>.world`

In RViz, alter the Global Options Fixed Frame paramenter to be 'map'

Add a Map to the displays with the 'Add' button on the lower left of the screen. Configure the Topic to be '/map'. You should see a map of your simulated robot's surroundings begin to be created. Navigate around your world to build up the map.   
-->

# Known Issues
- If you run a gazebo instance and then close it, it may still be running in the background. Kill the gzserver and gzclient processes from the terminal if a new instance of gazebo wont load because of this. 
- Sometimes adding/deleting models into the world crashes gazebo. 
<!---
- The slam mapping feature has a bug related to its frame of reference, which results in maps that look placed on the plane disconnected. 
-->
