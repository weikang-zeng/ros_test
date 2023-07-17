This is a skill test project at the request of iFollow
====
![](img/iFollow-OFFICIEL-PNG.png)

|Author|zeng|
|---|---
|E-mail|mr.gangge@hotmail.com

***

## Objectif
* [Mise en place de l'environnement de test]
* [Multiplexeur]
* [Teleoperation a distance]
* [Envoi de Goal determine par un tag visuel]

## Mise en place de l'environnement de test
---
![](img/robotis_emanual_logo.png)
**recommended to use this site web to get started**[emanual.robotis](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

Following the instructions you can set up your simulation platform or manipulate the robot

Specify the model of the robot you want to simulate before performin the simulation
`Export TURTLEBOT3_MODEL=burger`

Run an esisting map
`roslaunch turtlebot3_gazebo turtlebot3_world`

Run SLAM Node
`roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping`

Run Teleoperation Node
`roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`

Scanning out and storing environmental maps using keypad controls

![](img/showcase1.gif)

Save the map for the navigation after

`rosrun map_saver map_saver -f ~/map`

Run Navigation Node and load the map

`roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml`

Estimate Initial Pose-Align the robot's inital position in the map

Then we can let the robot navigate to the specified place

![](img/showcase2.gif)


**Configuration and commissioning time: 1h**

## Multiplexeur
---
Regarding multiplexers, it can be achieved using simple topic subscription and publishing, but it is not stable.
Version1 is a simple example：cmd_mux (version1).cpp

Considering the stability of the multiplexer, you can optimize the subscriber and publisher settings, add priorities for different topics, as well as locking mechanisms and diagnostic information.

**recommended to use this github to learn the structure of a multiplexer**[twist_mux](https://github.com/ros-teleop/twist_mux)

The original idea was to add services and yaml files to the multiplexer to implement a prioritization system and handle different control sources. However, there were some unexpected obstacles to adding services. For example, using message_generation as a dependency to generate the corresponding header file for the srv file.

It was eventually decided to change the control source directly in the callback instead of through a service call. Change source_change_srv to type ros::Subscriber and subscribe to the new topic (/control_source)

You need to specify the source priority before using cmd_mux
`rosparam set sources "[{name: 'cmd_local', priority: 1}, {name: 'cmd_web', priority: 2}]"`

and, if necessary, switch the control source
`rostopic pub /control_source std_msgs/String "data: 'cmd_local' ou 'cmd_web"`

Use the following commands to post speed information on different topics, and with option -r 3 to post information continuously

`rostopic pub /cmd_local geometry_msgs/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

rostopic pub /cmd_web geometry_msgs/Twist "linear:
  x: -0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"`

Observe published messages at another console  
`rostopic echo /cmd_vel`

![](img/showcase3.gif)

**Configuration and commissioning time**: **2h for the first version**

**Coding time: 3~5h and commissioning and debugging time**: **10~15h**

## Teleoperation a distance
---
For the mqtt part, I developed a ros node in C++ that uses the mosquitto package to subscribe to mqtt topics. For the publisher side I used python's paho library for the customer side.

Finally it implements the command to publish the linear and angular velocities via mqtt, the ros simulator side receives the information and sends it to the cmd_web topic, and finally publishes it to the cmd_vel topic via the multiplexer cmd_mux.

During debugging, I forgot that python's scripting requirements require the use of keyboard arrow keys to issue speed commands, because at the beginning I tested this by sending a simple message to the subscription side as a test. It's easy to implement though, use the getch library and then define the corresponding speed commands for each arrow key under keypress and then publish them.

**mqtt_subscriber_node.cpp**:The main thing this program does is to receive messages on an MQTT topic, convert them to ROS Twist messages, and post them to the /cmd_web topic. In this process, it uses regular expressions to parse the payload of the received MQTT messages.

**mqtt_publisher.py**:This is a python script for letting users specify linear and  angular velocities and send velocity commands via MQTT

**old_version_test_python_code.py**:This is a release program that was written in the initial phase as required, in the first round of testing there was a problem with the MQTT link, so other program tests were written to debug MQTT. This was to be used after the debugging was complete, but I forgot about it. 

![](img/showcase4.gif)


**Coding time: 8h and commissioning and debugging time: 20h**
## Envoi de Goal determine par un tag visuel
---

Before using the camera to recognize April tags, you need to calibrate the camera and make appropriate changes to the launch file corresponding to apriltag_ros, such as the corresponding PC camera topic, and the image topic

The camera can be initially calibrated using the **usb_cam** package in conjunction with the checkerboard grid.

Further calibration for recognizing apriltag can be done according to the tutorial in Apriltag_ros in conjunction with matlab calibration.

Don't forget to set the type of April tags to be recognized in the **settings.yaml** file, and make the corresponding changes in the **tag_family** item.

After finishing calibration and setup, you can run the following commands to test the results of ARtag detection

`roslaunch usb_cam usb_cam-test.launch`
`roslaunch apriltag_ros continuous_detection.launch`
`rqt_image_view`

The tag I received should be tag36h11, which is recognized in the camera as April tags with IDs 20, 21, and 22 respectively, and contains no additional coordinate information.This proves that apriltag_ros is a proper package.

![](img/showcase5.gif)

Now the progress is to call the functions in AprilTagDetectionArray to recognize and decode the image and get the parsed information in preparation for publishing the navigation information later.

The apriltag_models were created in the models file of turtlebot3_gazebo, and the model file can be used to build the April model target in the simulation world.

**Coding time: 10h and commissioning and debugging time: 25~30h**
