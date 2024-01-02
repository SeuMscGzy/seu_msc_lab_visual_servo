### 位置视觉伺服
启动机械臂
```shell
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.101
```
切换机械臂控制器到关节速度控制下
```shell
ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_velocity_controller
```

启动视觉检测节点
```shell
ros2 run object_detector object_detector
```

启动伺服节点
```shell
ros2 run ur5_servo joint_vel_sender
```

启动传送带控制节点
```shell
ros2 run conveyor_belt_control belt_control
```
如果未从终端启动matlab，记得在matlab命令行里输入
```matlab
setenv("ROS_DOMAIN_ID","15")
```