# xm_arm_manipulation

Create Date: 2015.11.1

Authors: myyerrol

Function: 
This metapackage implements the first generation of our xmbot's arm manipulation.

Package:
1.xm_arm_bringup: Bring up arm or gripper's controllers and other default parameters to initialize states.
2.xm_arm_control: This package implements some important componets about joint's action and service to provide interfaces with state machine.
3.xm_arm_teleop: Control arm with gripper by using keyboard.
4.xm_arm_test: Test arm's manipulation.

Summary:
In this version, though whole arm's pick and place pipeline only uses four joint's dof instead of six, we achieve a simple arm manipulation with our xmbot successfully! In the future version, in lower layer, we plan to rewrite arm's underlying codes to make more fiexible control and encapsulate codes for being called by statemachine more easily. And for upper layer, we will use moveit package to achieve simple motion planning. To sum up, next version can implement complete arm manipulation really.

Use:
First to ensure that the stm32 serial port wire, the servo controller serial wire as well as kinect have been connected to the laptop. Particularly, for the serial wire, it must be connected to the stm32 first, and then connect the servo controller(because stm32: /dev/ttyUSB0, servo controller: /dev/ttyUSB1)
1.Change the serial port of the executable permission:
sudo chmod 777 /dev/ttyUSB*
2.Start the mechanical arm and gripper action server:
roslaunch xm_arm_bringup xm_arm_and_gripper_bringup.launch
3.Start robothw and serial port node:
roslaunch xm_bringup xm_robothw_and_serial_bringup.launch
4.Start kinect's openni driver:
roslaunch openni_launch openni.launch 
5.Start kinect's two-dimensional code recognition:
roslaunch xm_ar_tags ar_indiv_kinect.launch
6.Start kinect's object recognition:
rosrun xm_object_cui main_test
7.Start object grasping's state machine:
rosrun xm_strategy object_smach.py
