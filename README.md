# Panasonic-VR-006L-driver
Interface between ROS and a Panasonic VR-006L robot over the PROFIBUS network

## Introduction

This repository contains an experimental ROS driver for a Panasonic VR-006L manipulator 
using a Profibus fieldbus as a physical connection between ROS and the Panasonic robot controller.

## Installation

The first thing you need to do is to create a local ROS environment and clone this repo into your local workspace. 
This can be done as follows:

```
mkdir -p ~/catkin_ws/src
```

After this, clone this repository into your workspace by using the HTTP or SSH of the repo:
```
cd catkin_ws/src
git clone https://github.com/davidDS96/Panasonic-VR-006L
rosdep install -r -y --from-paths src --ignore-src
```

The next thing is to make sure that the Pyprofibus stack is properly installed. To install this, 
please check the [author's website](https://bues.ch/cms/automation/profibus.html), his [GitHub page](https://github.com/mbuesch/pyprofibus/) or the [Python (pypy) webpage](https://pypi.org/project/pyprofibus/#description). 

To check if this stack is properly installed on your system's python, you can verify it by launching the python shell in the terminal and check if there are no import errors:
```python
python
import pyprofibus
```
If no error shows up, the stack is properly installed.

The last thing you need to do is to adjust a few paths in some files. First, go to the ``` vr_driver/config ``` folder and change in ```panaprofi.config``` the specified path of the serial interface. This either can be a USB-port or a RS232-serial port. Secondly, in this file change also the specified path of the GSD-file of the robot to your local path. Lastly, navigate to the file ```interface.py``` located in ```vr_driver/scripts``` and change the local path of the config file to the adjusted path.


## Usage

A first thing is to make sure that a serial to RS485 adapter is used as a proxy between the serial port of the PC and the Profibus connector of the robot. After the installation of the Profibus stack and the setup of the repository, the next thing is to make sure that the used serial port is allowed to read and write data. Then the driver can be used to let the robot move. The user can specifiy a desired Cartesian Pose (position and orientation) by specifying this in the interface that shows up in the terminal. A quick walkthrough is given below:

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
dmesg
sudo chmod a+rw /dev/tty...
```

In the ```dtps``` folder inside ```vr-driver``` the program of the robot is included (```Position_Streaming.prg```). This program needs to be converted to the controller by using the touch pad of the Panasonic robot. Just execute this file on robot first.

## Launching the interface

Finally, the driver can be used by launching the ``` position_streaming_interface.launch``` file that launches the driver node which initializes the communication between ROS and the robot:
```
roslaunch vr_driver position_streaming_interface.launch
```

## Launching MoveIt! and visualise movement in RViz

To visualise the imposed movement of the robot in visualisation, first launch the ```visualisation.launch```file that is located in the same
folder as ``` position_streaming_interface.launch```. Here the robot's movement is visualised in RViz before sending its
Cartesian pose to the robot controller.


## Expected output

The expected output of the interface in the terminal should be something like this:

```
SUMMARY
========

PARAMETERS
 * /rosdistro: indigo
 * /rosversion: 1.11.21

NODES
  /
    interface (vr_driver/interface.py)

auto-starting new master
process[master]: started with pid [20780]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to d06cb56a-59c3-11e8-a736-f81654125b1a
process[rosout-1]: started with pid [20793]
started core service [/rosout]
process[interface-2]: started with pid [20796]
[INFO] [WallTime: 1526555819.348752] Initializing position streaming interface 
[INFO] [WallTime: 1526555819.349098] Setting up communication with Panasonic robot controller 
[INFO] [WallTime: 1526555819.379378] Getting user information from terminal 
Give an integer value for the x-position in mm (between 100 and 1400):900
Give an integer value for the y-position in mm (between -900 and 900):-200
Give an integer value for the z-position in mm (between 0 and 1400):600
Give an integer value for the x-orientation in degrees (between -1800 and 180):30
Give an integer value for the y-orientation in degrees (between -180 and 180):45
Give an integer value for the z-orientation in degrees (between -180 and 180):45
Now give a movetype: Point-to-point (1), Continuous-Linear (2), Continuous-Circular (3) --> 1
[INFO] [WallTime: 1526555829.892654] Sending position and orientation to robot
```


## Acknowledgments

The authors would like to thank [Michael Büsch](https://bues.ch/cms/resources/contact.html) for using the open implementation of the PROFIBUS protocol in python.
