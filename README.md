# Inverse and Forward Kinematics ROS Package 

This code was part of a semester thesis at ETH Zurich for Ophthorobotics AG and is just an excerpt which might be useful for other users. It was tried to change the code to a generic form.

This ROS packages solves the common problem of inverse and forward kinematics in an easy way in C++ by using OrocosKDL library. OrocosKDL library can be used standalone. This solution is much simpler than using for example MoveIt package if we are just interested in coordinate transformations.

This ROS package provides overall kinematic control for a robot defined in the form of coordinate transformations. Generalized joint coordinates or end-effector coordinates can be used. It handles all the commands sent to the different joints and offers itself an API to receive commands. Furthermore, it provides a class, which does all the forward and inverse kinematics calculations by using the Orocos KDL library. A model of a specific robot was implemented by defining coordinate system transformations according to the image below.

![Robot coordinate system](kos.png?raw=true "Robot coordinate system")

This is a catkin workspace ROS node. Tested on Ubuntu 16.04 and ROS Kinetic Kame.

## Important notes

Base frame is defined at the intersection of base plate and alpha axis. Set the distances in config/default.yaml. It is very difficult to find possible end-effector positions due to joint limits. Check error messages. 

Inputs are in mm and degree.

Set robot parameters in config/default.yaml.

## Build

Use "catkin build" for building. Download and install dependencies first.

`cd catkin_ws`

`catkin build serialkinematics`

(Builds all the dependencies. Think of removing them and adding your own.)

## Run

`roslaunch serialkinematics serialkinematics.launch`


## Services and topics

Joint parameters in mm and deg:

`rosservice call /serialkinematics/joint_pos_service "{y_joint: 0.0, x_joint: 0.0, alpha_joint: 0.0, beta_joint: 0.0, d_joint: 0.0}" `

End-effector position in mm and deg:

`rosservice call /serialkinematics/ee_pos_service "{x_ee: 0.0, y_ee: 0.0, z_ee: 0.0, phi_ee: 0.0, theta_ee: 0.0}" `

Generalized joint coordinates topic:

`rostopic echo /serialkinematics/global_state_topic`


## Dependencies

This package handles overall control. Therefore, there are many building and runtime dependencies. At least the following packages should be available. Some of them are also started in the launch file and are necessary for proper use of the kinematics. 

- roscpp
- orocos_kdl
- message_generation
- std_msgs

Think of removing these dependencies and adding your own:
- tdc_controller (needed for message headers)
- custom_communication (message headers)
- epos2_controller (needed for message headers)
- kinematics_gui (started in launch file)
