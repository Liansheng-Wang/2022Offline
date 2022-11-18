# 环境配置

1、更改 detection 中 cmakelist 中 paddle_inference 的文件位置

2、配置好 PX4 固件之后。把 onboard/src/simulation/competition_sim/models 中的 models 文件拷贝到 ~/.gazebo/models中


``` 
cd onboard
catkin_make
source ./devel/setup.zsh
roslaunch competition_sim sim_with_d435i.launch
```

查看是否可以正确打开仿真环境，并且 mavros 正常运行

如图所示：

![仿真环境](pictures/sim.png)


```
roslaunch controller AutoFly.launch
```

## 该分支在搞大动作, 搞呀搞
