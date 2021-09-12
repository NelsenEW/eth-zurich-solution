# ETH Zürich ROS Solution
This is an unofficial solution for [eth zurich ros](https://rsl.ethz.ch/education-students/lectures/ros.html) 2021 course for MLDA Robotics. The solution has been well tested for both ROS Melodic and Noetic.

The main solution package created for this is the `smb_highlevel_controller` which was created with the following command:

`catkin_create_pkg smb_highlevel_controller`

## Setup
As the exercise requires the use of ROS in gazebo simulation, it is assumed that the computer is properly setup with [`ros-melodic-desktop-full`](http://wiki.ros.org/melodic/Installation/Ubuntu) or [`ros-noetic-desktop-full`](http://wiki.ros.org/noetic/Installation/Ubuntu)  installation. Once ROS is installed, you can run the following command to create a Catkin workspace:

``` bash
sudo apt-get install python3-catkin-tools
mkdir -p catkin_ws/src
cd ~/catkin_ws
catkin build

# Automatically source setup.bash for convenience.
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

This ROS stack requires some dependencies which can be installed with the following command:

``` bash
sudo apt install -y ros-<distro>-hector-gazebo-plugins \
                    ros-<distro>-velodyne \
                    ros-<distro>-velodyne-description \
                    ros-<distro>-velodyne-gazebo-plugins \
                    ros-<distro>-pointcloud-to-laserscan \
                    ros-<distro>-twist-mux
```
where `<distro>` can be either melodic or noetic.

Once everything is fully setup, you can clone the package into the `catkin_ws/src` directory and build the entire package:
``` bash
cd ~/catkin_ws/src
git clone https://github.com/NelsenEW/eth-zurich-solution
cd ..
catkin build
```