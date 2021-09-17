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

## [Exercise 1](<docs/exercise/Exercise Session 1.pdf>)
This exercise is based on [lecture 1](<docs/lecture/ROS Course Slides Course 1.pdf>).

Run the launch file with the following command:

`roslaunch smb_highlevel_controller smb_highlevel_controller.launch`

### Output

|![solution_1.png](docs/image/solution_1.png)|
|:--:|
| <b>Gazebo with SMB and teleop twist keyboard</b>|

#### Command line
* To control smb robot in gazebo through command line (press tab for autocompletion):

    `rostopic pub /cmd_vel geometry_msgs/Twist '[0.5,0,0]' '[0,0,0]'`
#### [smb_highlevel_controller.launch](smb_highlevel_controller/launch/smb_highlevel_controller.launch)
* The world file argument is hardcoded as follow:

    `<arg name="world_file" value="/usr/share/gazebo-9/worlds/robocup14_spl_field.world"/>`
* To launch the teleop keyboard in a new terminal, set the `launch-prefix` to `xterm -e`


## [Exercise 2](<docs/exercise/Exercise Session 2.pdf>)
This exercise is based on [lecture 2](<docs/lecture/ROS Course Slides Course 2.pdf>).

Run the launch file with the following command:

`roslaunch smb_highlevel_controller smb_highlevel_controller.launch`

The solution package template is based on [ros_best_practices](https://github.com/leggedrobotics/ros_best_practices)

### Output
The solution output should be as follow:
|![solution_2.png](docs/image/solution_2.png)|
|:--:|
| <b>Rviz with laserscan, terminal with output and gazebo</b>|

##### pointcloud_to_laserscan
![pointcloud_to_laserscan.png](docs/image/pointcloud_to_laserscan.png)|

As can be seen from the `rqt_graph`, the `pointcloud_to_laserscan` node is subscribing to `/rslidar_points` which is a `PointCloud2` message and `/tf` and converts it into a `LaserScan` topic `/scan`.

### Files
#### [default_parameters.yaml](smb_highlevel_controller/config/default_parameters.yaml): 
* consist of parameters that are passed to the launch file.

#### [SmbHighlevelController.hpp](smb_highlevel_controller/include/smb_highlevel_controller/SmbHighlevelController.hpp): 
* Header file for **SmbHighlevelController** class and method declaration.
* Include `roscpp` and `sensor_msgs` that are used in the executable file.

#### [smb_highlevel_controller_node.cpp](smb_highlevel_controller/src/smb_highlevel_controller_node.cpp):
* Create a ROS node with private node handler `(~)`.
#### [SmbHighlevelController.cpp](smb_highlevel_controller/src/SmbHighlevelController.cpp): 
* Implementation of the class method including fetch parameters from launch
* Subscribe to topics name based on the parameters
* Implementation of callback method such as `scanCallback` and `pclCallback`.

#### [smb_highlevel_controller.rviz](smb_highlevel_controller/rviz/smb_highlevel_controller.rviz): 
* contains rviz file format which were created by running rviz seperately, adding the required display, and saving it into the rviz file.

#### [smb_highlevel_controller.launch](smb_highlevel_controller/launch/smb_highlevel_controller.launch):

* Add `<rosparam>` to load [default_parameters.yaml](smb_highlevel_controller/config/default_parameters.yaml) to parameter server.
* Add `node` to launch the [smb_highlevel_controller_node](smb_highlevel_controller/src/smb_highlevel_controller_node.cpp) script.

#### [CMakeLists.txt](smb_highlevel_controller/CMakeLists.txt):
* Use `C++11` with `add_compile_options`
* Add `find_package` to find libraries such as `roscpp` and `sensor_msgs`.
* Add `catkin_package` to include `INCLUDE_DIRS`.
* Define the include directories using `include_directories`.
* Add executable based on the project name from two different files which are [smb_highlevel_controller_node.cpp](smb_highlevel_controller/src/smb_highlevel_controller_node.cpp) and [SmbHighlevelController.cpp](smb_highlevel_controller/src/SmbHighlevelController.cpp)
* Link the libraries based on the `catkin_LIBRARIES` which is defined on the top.

#### [package.xml](smb_highlevel_controller/package.xml)
* Add `depend` for the dependencies which are `roscpp`, `sensor_msgs` and `smb_gazebo`
## [Exercise 3](<docs/exercise/Exercise Session 3.pdf>)
**Note: Change `smb_common` package to `smb_common_v2` package**

This exercise is based on [lecture 3](<docs/lecture/ROS Course Slides Course 3.pdf>).

Run the launch file with the following command:

`roslaunch smb_highlevel_controller smb_highlevel_controller.launch`

### Output
The solution output should be as follow:
|![solution_3.png](docs/image/solution_3.png)|
|:--:|
| <b>Rviz with marker visualization indicate with the green color ball and tf marker, terminal with printed output such as the angle , and smb is heading towards the pillar in gazebo</b>|

### Files
#### [CMakeLists.txt](smb_highlevel_controller/CMakeLists.txt) and [package.xml](smb_highlevel_controller/package.xml):
* Add dependencies such as `geometry_msgs`, `tf2_ros`, and `visualization_msgs` package.

#### [SmbHighlevelController.cpp](smb_highlevel_controller/src/SmbHighlevelController.cpp) and [SmbHighlevelController.hpp](smb_highlevel_controller/include/smb_highlevel_controller/SmbHighlevelController.hpp):

* Include `geometry_msgs`, `tf2_ros`, and `visualization_msgs` package.
* Add two publisher for topics `visualization_marker` and `cmd_vel` during initialization.
* Create a `goalPose` of type `geometry_msgs::PoseStamped` which is the pillar from the lidar reading with respect to the `rslidar` frame.
* Create TF listerner and TF buffer to transform the `goalPose` from the `rslidar` frame to `odom` on `transformOdom`.
* Utilize a P controller from the error angle to drive the error to zero on `moveToGoal`, the x velocity is set to constant without P controller to ensure that the SMB hits the pillar.
* Publish a visualization marker on `visMarkerPublish` that can be displayed in Rviz.

#### [smb_highlevel_controller.launch](smb_highlevel_controller/launch/smb_highlevel_controller.launch):
* Change the world argument value to `"$(find smb_highlevel_controller)/world/singlePillar.world"`
* Add two arguments under `laser_scan_min_height` and `laser_scan_max_height` to -0.2 and 1.0 respectively.
* Remove the `teleop_twist_keyboard` node from the launch.

#### [smb_highlevel_controller.rviz](smb_highlevel_controller/rviz/smb_highlevel_controller.rviz):
* Add `Marker` display to visualize the pillar marker indicated with green color ball.

## [Exercise 4](<docs/exercise/Exercise Session 4.pdf>)

This exercise is based on [lecture 3](<docs/lecture/ROS Course Slides Course 3.pdf>) and [lecture 4](<docs/lecture/ROS Course Slides Course 4.pdf>).

This exercise requires the use of rqt_multiplot. Run the following command to install rqt_multiplot:
`sudo apt install -y ros-<distro>-rqt-multiplot`

where `<distro>` can be either melodic or noetic based on your computer ROS_DISTRO.
### Simulation
The simulation can be run with the following command:
`roslaunch smb_highlevel_controller smb_highlevel_controller.launch`
#### EKF Localization Node
To understand the EKF Localization Node, open another terminal, then open it with `rqt_graph`.

The output is the following:
|![solution_4_ekf_localization.png](docs/image/solution_4_ekf_localization.png)|
|:--:|
| <b>ekf_localization node in rqt_graph</b>|

As can be seen from the graph, the ekf localization subscribes to `/imu/data` and `/smb_velocity_controller/odom` topics and publishes `/odometry/filtered` topic by applying extended kalman filter. In this case, the topic will be displayed in both rqt_multiplot and rviz.

#### Plot of simulation x/y-plane
The solution output should be as follow:
|![solution_4_simulation.png](docs/image/solution_4_simulation.png)|
|:--:|
| <b>Plot of x/y-plane that is taken by SMB (Kp = 30, x_vel = 3 m/s) until it hits the pillar on rqt_multiplot</b>|

### Recorded (rosbag)
#### ROS Topic inside `smb_navigation.bag`
To get all the topics and messages inside the rosbag, run the following command:

`rosbag info smb_navigation.bag`

The solution should be as follow:
![solution_4_rosbag_info.png](docs/image/solution_4_rosbag_info.png)

To run the recorded rosbag, use the following command:

`roslaunch smb_highlevel_controller ekf_localization.launch`

#### Plot of recorded x/y-plane
The solution output should be as follow:
|![solution_4_recorded.png](docs/image/solution_4_recorded.png)|
|:--:|
| <b>Plot of x/y-plane plot that is taken by SMB until the rosbag recording ends</b>|


#### Visualization of 3D point cloud and TF marker in Rviz
The 3D point cloud as well as `smb_top_view` frame can be visualize in rviz:
|![solution_4_rviz.png](docs/image/solution_4_rviz.png)|
|:--:|
| <b>3D lidar point cloud and smb_top_view frame visualize in rviz</b>|

The `smb_top_view` frame will move according to the `base_link` frame. As such, the `smb_top_view` is moving together with the robot in rviz when the rosbag is played.

### Files

#### [smb_navigation.bag](smb_highlevel_controller/bag/smb_navigation.bag):
* Contains 59.7 seconds of a recorded simulation.
* The size of the bag is 158.9 MB with total messages of 1545.
* The topics recorded are `/imu/data`, `join_states`, `rslidar_points`, and `smb_velocity_controller/odom`
#### [xy_multiplot.xml](smb_highlevel_controller/config/xy_multiplot.xml):
* Create an x/y-plane plot of the smb based on the output of the `ekf_localization` node which is `/odometry/filtered` with type `nav_msgs/Odometry`.
#### [ekf_localization.rviz](smb_highlevel_controller/rviz/ekf_localization.rviz):
* Display TF, PointCloud2, and RobotModel of the smb
#### [smb_highlevel_controller.launch](smb_highlevel_controller/launch/smb_highlevel_controller.launch):
* Add rqt_multiplot node with [xy_multiplot.xml](smb_highlevel_controller/config/xy_multiplot.xml) to plot the path of smb in x/y plane.

#### [ekf_localization.launch](smb_highlevel_controller/launch/ekf_localization.launch):
* Refer from [control.launch](smb_common_v2/smb_control/launch/control.launch) file that is located on the `smb_control` package.
* Add `ekf_robot_localization` node and load the required parameters from the [localization.yaml](smb_common_v2/smb_control/config/localization.yaml)
* Add `smb_robot_state_publisher` to publish state of the robot to tf2 that is visualize in rviz.
* Create a frame called `smb_top_view` with `static_transform_publisher` node which is 2 meters above the `base_link` frame.
* Add `rosbag` node to play rosbag with full speed or half speed.
* Launch rviz with [ekf_localization.rviz](smb_highlevel_controller/rviz/ekf_localization.rviz) configuration.
* Add rqt_multiplot node with [xy_multiplot.xml](smb_highlevel_controller/config/xy_multiplot.xml) to plot the path of smb in x/y plane.

## [Exercise 5](<docs/exercise/Exercise Session 5.pdf>)

This exercise is based on [lecture 4](<docs/lecture/ROS Course Slides Course 4.pdf>).

### Manual Service call
The service name `/startService` is defined inside [default_parameters.yaml](smb_highlevel_controller/config/default_parameters.yaml).

Run the launch file with the following command:

`roslaunch smb_highlevel_controller smb_highlevel_controller.launch`

To start the robot, run the following command on another terminal:

`rosservice call /startService "data: true"`

Alternatively you can run the robot during the startup with the following command:

`roslaunch smb_highlevel_controller smb_highlevel_controller.launch start_robot:="true"`

To stop the robot **manually** from colliding, open another terminal and run the following command:

`rosservice call /startService "data: false"`

The robot can always continue its path by calling the service by setting the data to `true`.

### Automatic emergency
#### Prior collision
Run the launch file with the following command:

`roslaunch smb_highlevel_controller smb_highlevel_controller.launch start_robot:="true" auto_emergency:="true"`

By default, the robot will stop **before** hitting the pillar with a distance of `max_distance_to_pillar` from the robot's lidar that is specified in the [default_parameters.yaml](smb_highlevel_controller/config/default_parameters.yaml).


The solution output should be as follow:
|![solution_5_prior_collision.png.png](docs/image/solution_5_prior_collision.png)|
|:--:|
| <b>The SMB stops before hitting the pillar. In the terminal, the service was called to stop the robot. </b>|

#### Post collision
Run the launch file with the following command:

`roslaunch smb_highlevel_controller smb_highlevel_controller.launch start_robot:="true" auto_emergency:="true" prior_collision:="false"`


By default, the robot will stop **after** hitting the pillar based on `collision_threshold` which is the maximum change of IMU on x axis before the service is called that is specified in the [default_parameters.yaml](smb_highlevel_controller/config/default_parameters.yaml).

To get a proper `collision_threshold`, `rqt_multiplot` is launched with the [xy_imu_multiplot.xml](smb_highlevel_controller/config/xy_imu_multiplot.xml) config.

During the collision, the x-axis IMU plot can be seen as follow:

|![solution_5_imu_plot.png](docs/image/solution_5_imu_plot.png)|
|:--:|
| <b>The plot of IMU x-axis and y-axis plot during collision, there is a sudden spike of IMU x-axis with the value change of around 1.8 </b>|

The overall output should be as follow:
|![solution_5_post_collision.png](docs/image/solution_5_post_collision.png)|
|:--:|
| <b>The SMB stops after hitting the pillar. In the terminal, the service was called to stop the robot. </b>|

### Files
#### [SmbHighlevelController.cpp](smb_highlevel_controller/src/SmbHighlevelController.cpp) and [SmbHighlevelController.hpp](smb_highlevel_controller/include/smb_highlevel_controller/SmbHighlevelController.hpp):
* Add parameters for the service (i.e. service name, start_robot).
* Add start/stop server, service name and service callback based on the service call with `SetBool`.
* Add bool `isStart_` to move the robot only if it is enabled.
#### [stop_condition_node.cpp](smb_highlevel_controller/src/stop_condition_node.cpp)
* As the client, call the service to stop SMB robot with `SetBool` based on the parameter `prior_collision`. 
* The parameters that can be adjusted are `max_distance_to_pillar` for prior collision and `collision_threshold` for post collision.
* This node is called from the launch file if the parameter `auto_emergency` is enabled.

#### [xy_imu_multiplot.xml](smb_highlevel_controller/config/xy_multiplot.xml):
* Create an x/y-plane plot of the smb and the x and y IMU data over time.
#### [smb_highlevel_controller.launch](smb_highlevel_controller/launch/smb_highlevel_controller.launch):
* Use arguments to load some of the params instead of `rosparam` from [default_parameters.yaml](smb_highlevel_controller/config/default_parameters.yaml). This allow arguments to be passed from command line.
* Add `stop_condition_node` to automatically stop the SMB prior/post collision if enabled.
* Change rqt_multiplot config with [xy_imu_multiplot.xml](smb_highlevel_controller/config/xy_imu_multiplot.xml) to plot the path of smb in x/y plane as well as x and y IMU data over time.
#### [CMakeLists.txt](smb_highlevel_controller/CMakeLists.txt)
* Add std_srvs in `find_package`
* Add executable `stop_condition` from `stop_condition_node.cpp`
* Add `target_link_libraries` for `stop_condition`
#### [package.xml](smb_highlevel_controller/package.xml)
* Add depend `std_srvs` to use `SetBool` service.
