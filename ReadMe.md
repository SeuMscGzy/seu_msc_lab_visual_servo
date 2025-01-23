## RAID-AgiVS in PBVS

### Before Simulink
1、Start Manipulator
```shell
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102
```
2、Start the corresponding program in UR instructor

3、Switch the manipulator controller to the joint speed control
```shell
ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_velocity_controller
```
4、Start the visual detection node
⭐⭐⭐Under the condition that the camera hardware and transmission line are normal, if the operation fails and no image appears, you can shut down the node and restart it.
```shell
ros2 run object_detector object_detector
```

5、Start the manipulator servo node
```shell
ros2 run ur5_servo joint_vel_sender
```

6、Start the belt control node
```shell
ros2 run conveyor_belt_control belt_control
```

7、If you don't start matlab from a terminal, remember to type it in the matlab command line
```matlab
setenv("ROS_DOMAIN_ID","15")
```

8、Start the corresponding matlab program

### Simulink Part

The program is located under ias_lab_visual_servo/simulink/ur5_simulink_ros2_pbvs_mpc/ur5_simulink_ros2_pbvs_pid.slx

⭐⭐⭐Initialize the manipulator model by executing ur5_model_init.m before running simulink. 

1、flag_control: stop_flag is used to control whether the manipulator is running. Flag_control: stop_flag is used to control whether the manipulator is running. motor_flag indicates that the motor control 0 indicates the stop, 1 to 4 indicates the forward movement of the absolute position, and -1 to -4 indicates the reverse movement to the absolute position. The greater the absolute value of the number, the faster the speed.
<p align="center">
<img src="doc/pbvs_flag.png">
</p>
2、time record: used to record the current program running time, mainly used for drawing, or own design of different time reference input or motor operation.
<p align="center">
<img src="doc/pbvs_time.png">
</p>
3、refer quat: the position feedback has not been done at the moment, so a reference pose is set.
<p align="center">
<img src="doc/pbvs_refer_quat.png">
</p>
4、subscriber: Track the spatial position of objects from ros2 subscription and obtain the Angle of each joint of the manipulator in real time.

⭐ Note that the joint sequence of the robotic arm subscribed by ros2 is different from the actual one, which has been modified in simulink.

<p align="center">
<img src="doc/pbvs_sub.png">
</p>

5、controller: Example of proportional controller design

<p align="center">
<img src="doc/pbvs_controller.png">
</p>

6、controller: Scale controller design example and publish joint speed into ros2.

<p align="center">
<img src="doc/pbvs_controller.png">
</p>

7、motor: Release the corresponding motor control signal to ros2.

<p align="center">
<img src="doc/pbvs_motor.png">
</p>

8、trans: manipulator coordinate change and Jacobian matrix acquisition.
<p align="center">
<img src="doc/pbvs_trans.png">
</p>

9、data: Records running process information, which is used to draw diagrams in matlab.
<p align="center">
<img src="doc/pbvs_trans.png">
</p>
