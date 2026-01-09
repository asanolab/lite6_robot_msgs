# lite6_labauto package

This package works as bridge of other packages and Ufactory official packages.

## Prepare Ufactory Lite 6
Launch Ufacotry Lite 6 xarm_bringup (make sure to clone official packages first).
```
$ roslaunch xarm_bringup lite6_server.launch robot_ip:=192. show_rviz:=true add_gripper:=true
```

## Prepare camera
Launch camera nodes (clone official Realsense-ROS packages first).
```
$ roslaunch realsense2_camera rs_camera.launch
```

## Initialize or shut down Lite 6
```
$ rosrun robot_control robot_init.py
$ rosrun robot_control robot_disable.py
```

## Initialize all Serial Ports (pH sensor, gripper, pipetty, pipette tip disposal motor)
```
$ roslaunch robot_control all_serial_nodes.launch
```

Available args:
```
  <arg name="gripper_port"           default="/dev/ttyACM0"/>
  <arg name="gripper_baud"           default="115200"/>

  <arg name="ph_port"                default="/dev/ttyACM2"/>
  <arg name="ph_baud"                default="9600"/>
  <arg name="ph_publish_voltage"     default="false"/>

  <arg name="pipette_motor_port"     default="/dev/ttyACM1"/>
  <arg name="pipette_motor_baud"     default="9600"/>

  <arg name="pipetty_port"           default="/dev/ttyUSB1"/>
  <arg name="pipetty_baud"           default="31520"/>
```


# msg
## LabwareOBB.msg
```
std_msgs/Header header
geometry_msgs/Pose pose
float32 x_length
float32 y_width
float32 z_height
```

## MovePose.msg
```
# For pose in move_line function
# [x,y,z,r,p,y] in unit [mm,mm,mm,rad,rad,rad]

float64[6] pose
```

#srv
## BeakerMani.srv
```
# input 
bool shake
---
# output
bool success
string message
```

## pHMeasure.srv
```
# input

float32 timeout_s
---
# output
bool success
float32 ph
string message
```

## PipetteDo.srv
```
# input 
string liquid # "HCl" or "water"
float32 volume_ul # Volume to aspirate
int32 tip_id
---
# output
bool success
string message
```

## TweezersDraw.srv
```
# input 
bool draw
---
# output
bool success
string message
```

# Hardware of Parallel Gripper
As for the robot arm "Ufactory Lite 6", there is an open-source parallel gripper which can be controlled by position, [OpenParallelGripper](https://github.com/hygradme/OpenParallelGripper).  
Here we used the [XL330_version](https://github.com/hygradme/OpenParallelGripper/blob/main/XL330_version/README.md), and modified the gripper jaws to adapt to different objects, as shown in the figure, ![Modified Gripper](assets/gripper.png "gripper").
