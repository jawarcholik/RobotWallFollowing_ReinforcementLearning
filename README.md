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

### Install Python packages

`pip install pynput`

### Install Other Dependencies

`sudo apt-get install xterm ros-melodic-depthimage-to-laserscan ros-melodic-gmapping`

### Download Stingray Simulation Repo

`cd ~`

`git clone https://github.com/kristinfarris/Stingray-Simulation.git`

## Running the Simulation

### Run catkin_make

`cd ~/Stingray-Simulation/catkin_ws`

`catkin_make`

### Source ROS root

`source /opt/ros/melodic/setup.bash`

### Source Stingray Simulation package

`source ~/Stingray-Simulation/catkin_ws/devel/setup.bash` 

### Running the simulation

`roslaunch stingray_simulation gazebo_base.launch`

### Send move commands

To make the robot move in the simulation, open another terminal, source ROS root like above, and publish to the /triton/vel_cmd topic. To do this, you must tab complete the last portion of the command for it to be formatted correctly and then change the x,y,and theta values to the desired setpoints.

`rostopic pub /triton/vel_cmd geometry_msgs/Pose2D <tab>`

You can press Ctrl+C to end the current message and can retype the command to send a new one. 

### Running the simulation with keyboard control

`roslaunch stingray_simulation gazebo_teleop.launch`

You can now use the keyboad to control the robot with WSAD/Arrow keys to translate and with Q & E to rotate it. The input from the keyboard is converted to a ros message sent to the same /triton/vel_cmd topic.

## Visualize Camera and Pointcloud Data

In another terminal, open rviz (ROS visualizer) to view the SLAM mappings:

`rviz rviz`

Alter the Global Options Fixed Frame paramenter to be 'map'

Add an Image to the displays with the 'Add' button on the lower left of the screen. Configure the Image Topic to be '/camera/color/image_raw'. You should be able to see an image of the robots view in the window in the lower left.  

Add a PointCloud2 to the displays with the 'Add' button on the lower left of the screen. Configure the Topic to be 'camera/depth/points'. 3D Point cloud data should now be up showing up in the 3D pane.  

## Visualize Odometry Data

Add an Odometry to the displays with the 'Add' button on the lower left of the screen. Configure the Topic to be '/odom' and Keep to be 1 (or any value you wish, depending on how much history ofodometry data you would like to show). You should see red arrows that depict the location and rotation of your robot in the simulation.  

## Running SLAM

To run the simluation with slam mapping enabled:

`roslaunch stingray_simulation slam.launch`

And similarly with keyboard control:

`roslaunch stingray_simulation slam_teleop.launch`

In another terminal, open rviz (ROS visualizer) to view the SLAM mappings:

`rviz rviz`

Alter the Global Options Fixed Frame paramenter to be 'map'

Add a Map to the displays with the 'Add' button on the lower left of the screen. Configure the Topic to be '/map'. You should see a map of your simulated robot's surroundings begin to be created. Navigate around your world to build up the map.   

## Known Issues
- If you run a gazebo instance and then close it, it may still be running in the background. Kill the gzserver and gzclient processes from the terminal if a new instance of gazebo wont load because of this. 
- Sometimes adding/deleting models into the world crashes gazebo. 

