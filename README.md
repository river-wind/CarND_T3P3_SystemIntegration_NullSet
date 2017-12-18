# Project: Capstone Project - System Integration
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This is the project repository for Team NullSet's final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

## Team Members

|     Name      | Location | github |
|---------------|----------|----------|
| Chris Lawrence | Dresher, PA | [https://github.com/river-wind/](https://github.com/river-wind/) | 
| Liam Bowers | New York, NY | [https://github.com/liamondrop](https://github.com/liamondrop) | 
| Scott Pillow | UTC - 5(city?) | [https://github.com/spillow](https://github.com/spillow) |
| Eugen Nekhai | Minsk, Belarus | [https://github.com/eugenn](https://github.com/eugenn) | 
| Roman Kyslyi | Ukraine | [https://github.com/woters](https://github.com/woters) |


-------------------README framework v2-----------


## Scope and Purpose

This project is designed to both teach about the Robot Operating System (ROS), and to allow the students to employ what they have learned during the Udacity Self Driving Car Nano Degree.  Each team member was assigned a portion of the project, and all work together to integrate those subsections into a codebase which would run successfully on the Udacity simulator, and directly on Carla, the real-world Udacity Licoln sedan.

### ROS

The Robot Operating System is employed by many robotics projects in order to facilitate the organized communication between discreet subsystems handling perception, localization, prediction, and control nessisary for an automated vehicle to function.  In this project, we rely on it to pass messages between task-specific "nodes", either streaming them to a "topic" and allowing other nodes to listen for those messages, or by creating a two-way communication via "services".  

ROS acts as the underlying communications structure which our nodes will rely on to control Carla's

### Nodes 

Nodes are specific logical units which handle particular tasks.  In this project, many nodes are used to handle sensor input and control of the vehicle.  As just a sample of the nodes involved, we used the tl\_detector node to identify stoplights in input video frames, the waypoint\_updater nodes to manage path planning, and the dbw and twist\_controller nodes to handle vehicle controls.

### Topics

Topics are one communication route for data to be sent between nodes.  A node publisher posts data to a "topic" which other nodes can subscribe to.  This communication is one-way, and there is no confirmation if a message has been received, much like the UDP internet protocol.

### Services

Using topics vs services depends on the requirements of each task at hand - if a message might trigger actions by other nodes, and the original node does not need to know about that action, a topic can.  Alternately, if a given message needs to trigger and action and provide a response back to the originating node, then a service would be used.


## Functional Project sections:

### Object and Traffic Light Detection

Relying on a pre-trained mobilenet Tensorflow network, the system classifies incoming visual data by if it contains a traffic light or not, and if that traffic light is currently Green, Yellow or Red.  When the light is Red, a message is published to ?? /traffic\_light\_state  ????  which the waypoint\_updater node uses to determine that it needs to stop the car at the stop line waypoint prior to that light.

### Waypoint finding

The waypoint finding subsystem relies on visual input to locate the desired best path for the car.  This must manage the obvious situations, such as remaining on the road itself and not drifting into other lanes, to less obvious, such as avoiding an routing around unexpected obstacles in the road.  The waypoint updater node takes in a list of current desired path nodes, and updates them by first 

### DBW and Twist Controller

The DBW system handles the control output, and the Twist Controller node takes in messages from the Waypoint updater and calculates the best throttle, brake and steering commands needed to reach the planned waypoints.   Further information


## Results

### In Udacity Simulator

<embed image>  For training purposes, the Udacity Term 3 simulator is used as a stand-in for the real vehicle Carla during training.  As of this project's completion, the simulator included a 3 mile long virtual track and a virtual representation of the real-world gravel parking area where the real-world test on Carla will be performed. 

The project code is able to drive the Udacity virtual car around the simulator test track successfully at a range of speeds, identify red stop lights, and stop prior to the stop line.  The code is also able to drive Carla in the Site simulator mode, and is ready to be run on Carla in the real-world churchlot environment.

### On Carla

For the project's initial submition, Carla testing has not yet been completed.  Once this testing is completed, this section will be updated.


## Conclusion and Additional Thoughts

Relying on ROS to communicate asynchronously between ROS nodes allows for each portion of the car's logic pipeline to operate on incoming sesor or internal state data, so that each step in the process is not waiting for a prior step to complete.  Relying on video input for stoplight detection to potentially override the waypoint and throttle commands which would normally be sent to the control system is one example of potentially multiple node which may need to take over from the base control strucure we had built in our waypoint finding exercise.  From traffic lights, signs, traffic cones, other cars, pedestrians, etc, the planning system will need to take into account more than just lane lines and speed limits when driving on a real-world road.  

Accident avoidance would be one such requirement, and ROS's action structure allows for inturrption and preemption, as we are demonstrating in a simpler form with the traffic light handling routine.  Using GPU-enabled Tensorflow and a mobilenet network to identify stoplights and classify lights into Red, Yellow, or Red, we can publish messages which the system can use plan its upcoming route.

*What would be needed to add additional logic, like object detection.  Mention YOLO, Apple's identification additions, and behavior planning based around known behaviors of common objects*


## Setup Prerequisites:

### System environment

To run this project yourself, either set up a native install or use Docker to create a docker container to work in.  Udacity has provided a VM image in case you are not running Linux natively, and also a Docker container for this purpose.  There is no GPU support under either option, however GPU-enabled docker containers for this project can be found online.

### VM Image

Udacity has made available a pre-configured VM image for use with Virtualbox.  You can find a link to the image on the project summary page in the Udacity classroom.  This is the quickest way to start this project - download the VM image and launch it from within Virtualbox per the provided instructions.

### Native Installation

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

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/) and launch the Quickstart Terminal.

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

## Usage

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

## Real world testing

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
5. Confirm that traffic light detection works on real life images, either by outputting log messages on succesfull identification, or by saving output images with rosimage.

Additional information about testing against RosBag data can be found here:
https://discussions.udacity.com/t/cant-find-the-rosbag-which-dbw-test-used-for-testing/
