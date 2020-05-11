## Final Project - System Integration
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


Overview
---
In this project, we will implement a Real Self Driving Car in python to maneuver the vehicle around the track while following the traffic rules.

Goal
---
The car follows a smooth trajectory and performs its maneuvers smoothly within the advised thresholds of acceleration and jerk.
It detects traffic lights with a higher accuracy and responds to the traffic lights accordingly.


## Output

![Output_gif](./imgs/output.gif)

## The Team

|         Name      |      Role     |    Location   | LinkedIn    |     email   |
|---------------|----------------|---------------|-------------|-------------|
| Sandeep Ashwatnarayana| __Lead__ | KA, India | [Sandeep](http://linkedin.com/in/sandeep-a/) | <saswathnaray@smu.edu> |
| Malith Ranaweera | Member | Australia | [Malith Ranaweera](https://au.linkedin.com/in/prasanga-ranaweera)| <rkmalith@gmail.com> |
| Arun Sagar | Member | DEL, India  | [Arun](www.linkedin.com/in/arun-sagar) |<asgar60@gmail.com> |
| Shuang Li| Member | China |  [ShuangLi]( ) | <shuangli2048@gmail.com> |



## Autonomous Vehicle Architecture

![image alt text](imgs/Autonomous_system_architecture.png)

#### Brief Description on Autonomous Vehicle Architecture

**Sensor Subsystem**
It is a Hardware component that gathers data about the environment . These hardware includes IMU, LIDAR, RADAR, Camera, GPS etc.

**Vehicle Subsystem**
It is a Software component that processes data from sensors into structured information. Most of the analysis of the environment takes place here. Ex - Traffic Light Detection , Obstacle Detection, Localization etc.

**Planning Subsystem**
Once the information is processed by the perception system, Vehicle can use that information to plan its path

**Control Subsystem**
Software Component to ensure that vehicle follows the path specified by planning subsystem



## CARLAs System Architecture

The following is a system architecture diagram showing the ROS nodes and topics used in the project. You can refer to the diagram throughout the project as needed

![image alt text](imgs/system_architecture.png)

*Note: The obstacle detection node is not implemented for this project as there will be no obstacles around the test track*

#### Brief Description on CARLAs Sytem Architecture

#### Perception Subsystem
This subsystem gathers data about the environment, processes it and sends information to further subsytems. It determines the state of closest traffic light , closest waypoint from the traffic light and its stop line , then it publishes the information to <ins>/obstacle_waypoints</ins> and <ins>/traffic_waypoints</ins> topic.

**Traffic light Detection Node**

This Node takes in camera images and predicts the state of traffic light . We use a deep neural net with pretrained FCNN to detect if the upcoming traffic light is red or not.

#### Planning Subsystem

This subsystem plans the vehicle's trajectory using current pose of vehicle, vehicle velocity. These trajectory keeps on updating based on state of traffic lights.

For this project, the waypoints are marked as green dots to visually observe the vehicle's trajectory.

The output of this module is to pass the list of waypoints to control subsystem.

**Waypoint Loader Node**

Waypoint Loader Node loads a CSV file that contains all the waypoints along the track and publishes them to the topic <ins>/base_waypoints</ins>

**Waypoint Updater Node**

Waypoint Updater Node subscribes to three topics list of waypoints, the vehicle’s current position, and the state of upcoming traffic lights. Based on that the node publishes a list of waypoints to follow - each waypoint contains a position on the map and a target velocity

#### Control Subsystem

This subsystem takes target trajectory information as input and sends control commands to navigate the vehicle

**Waypoint Follower Node**

This node  parses the list of waypoints to follow and publishes proposed linear and angular velocities to the /twist_cmd topic

**Drive By Wire (DBW) Node**

This node takes target linear and angular velocity and adjust the vehicle’s controls accordingly. Specifically, we control 3 things: throttle, steering, brakes.

## Testing Instructions

1. Clone this project repository
```bash
git clone https://github.com/asagar60/Udacity-SCND-Programming-a-Real-Self-Driving-Car
```

2. Install python dependencies
```bash
cd Udacity-SCND-Programming-a-Real-Self-Driving-Car
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

## Installation Instructions
### Please use **one** of the two installation options

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.
