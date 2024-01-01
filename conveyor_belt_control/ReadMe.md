## ros2 simulink 丝杆电机控制
编译前安装依赖
```shell
sudo apt install libserial-dev
```
确定串口权限被打开,设置完毕后重启计算机
```shell
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER
```
