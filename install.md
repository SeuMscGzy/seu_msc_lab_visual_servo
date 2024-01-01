## ros2 simulink 图像视觉伺服配置过程

### ros2安装(humble版本)

https://docs.ros.org/en/humble/index.html
### 额外的依赖库
sudo apt install libserial-dev
### realsense ros2安装
https://github.com/IntelRealSense/realsense-ros#installation
### visp 安装
保证源码编译，且编译前已完成librealsense的安装

https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-install-ubuntu.html

编译结束后，在对应文件夹执行
```shell
sudo make install
```
### moveit2安装(humble版本)
https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html

不要忘记安装cyclonedds，并进行切换
```shell
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### ur5 ros2驱动安装(humble版本)
https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble
编译前将Universal_Robots_ROS2_Driver/ur_robot_driver/launch/ur_control.launch.py里的
```py
controller_spawner_inactive_names = ["forward_position_controller"]
```
更改为
```py
controller_spawner_inactive_names = ["forward_velocity_controller"]
```


### .bashrc里添加
```
###################################
###########    ROS2   ############
#################################
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export ROS_DOMAIN_ID=15
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /home/ias-lab-701/ibvs_ws/install/setup.bash
```