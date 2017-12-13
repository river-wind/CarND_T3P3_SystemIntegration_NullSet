## Project: Capstone Project - System Integration
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This is the project repository for Team NullSet's final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

-------------------README framework v1-----------

### Prerequisites:

## System environment

Either set up a native install or use Docker to create a docker container to work in.  Udacity has provided a VM image in case you are not running Linux natively, and also a Docker container for this purpose.  There is no GPU support under either option, however GPU-enabled docker containers for this project can be found online.

## VM Image

Udacity has made available a pre-configured VM image for use with Virtualbox.  You can find a link to the image on the project summary page in the Udacity classroom.  This is the quickest way to start this project - download the VM image and launch it from within Virtualbox per the provided instructions.

## Native Installation

* It is recommended that you use the provided Ubuntu VM image which already has ROS and Dataspeed DBW installed, but if you want to set up your own development environment natively, you can follow these steps to do so.

* Be sure that you are running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using the provided Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

## Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/) and launch the Quickstart Terminal.

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage

In any environment 

1. Clone the project repository  (this step may need to be run from the user's home directory)
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```
2. Update pillow
```bash
apt-get remove pillow
apt-get install pillow
```
3. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
4. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
5. Run the simulator in your host OS.  The Docker terminal should show that the simulator has connected.

### Scope and Purpose

This project is designed to both teach about the Robot Operating System, and to allow the students to employ what they have learned during the Udacity Self Driving Car Nano Degree.  This project can be divided up into 4 major areas: 

## ROS

The Robot Operating System is employed by many robotics projects in order to facilitate the organized communication between discreet subsystems handling perception, localization, prediction and control.  In this project, we rely on it to pass messages between task-specific nodes, either streaming them to a "topic" and allowing other nodes to listen for those messages, or by creating a two-way communication via "services".  

# Nodes 

Nodes are specific logical units which handle particular tasks.  In this project, many nodes are used to handle sensor input and control of the vehicle.  As just a sample of the nodes involved, we used the tl_detector node to identify stoplights in input video frames, the waypoint_updater nodes to manage path planning, and the dbw and twist_controller nodes to handle vehicle controls.

# Topics

Topics are one communication route for data to be sent between nodes.  A node publisher posts data to a "topic" which other nodes can subscribe to.  This communication is one-way, and there is no confirmation if a message has been received, much like the UDP internet protocol.

# Services
Using topics vs services depends on the requirements of each task at hand - if a message might trigger actions by other nodes, and the original node does not need to know about that action, a topic can.  Alternately, if a given message needs to trigger and action and provide a response back to the originating node, then a service would be used.

### Functional Project sections:

## Object and Traffic Light Detection

Relying on Image recognition and ____________ (Tensorflow/Keras??)  Further information 

## Waypoint finding

The waypoint finding subsystem relies on visual input to locate the desired best path for the car.  This must manage the obvious situations, such as remaining on the road itself and not drifting into other lanes, to less obvious, such as avoiding an routing around unexpected obstacles in the road.  The waypoint updater node takes in a list of current desired path nodes, and updates them by first ________________   Further information needed.  __are we using A\* for low-speed planning?__

## DBW and Twist Controller

The DBW system handles the control output, and the Twist Controller node takes in messages from the Waypoint updater and calculates the best throttle, brake and steering commands needed to reach the planned waypoints.   Further information

### Udacity Simulator

<embed image>  For training purposes, the Udacity Term 3 simulator is used as a stand-in for the real vehicle Carla during training.  As of this project's completion, the simulator included a 3 mile long virtual track and a virtual representation of the real-world gravel parking area where the real-world test on Carla will be performed. 


### Real world testing

There is example output data from prior Carla test runs available for testing purposes via a "training bag".

1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstrating the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

Additional information about testing against RosBag data can be found here:
https://discussions.udacity.com/t/cant-find-the-rosbag-which-dbw-test-used-for-testing/390138?u=chris-672330

### Results


### Conclusion and Additional Thoughts
