### 位置视觉伺服
启动机械臂
```shell
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.101
```
切换机械臂控制器
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