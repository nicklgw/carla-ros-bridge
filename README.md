

## 启动流程

### 终端窗口1
```
nick@nick-PC:~/workspaces/carla-0.9.13/CARLA_0.9.13$ ./CarlaUE4.sh -quality-level=Epic -resx=800 -resy=600 -carla-rpc-port=2000
```

### 终端窗口2
```
nick@nick-PC:~/carla-ros-bridge$ source source_env.sh 
nick@nick-PC:~/carla-ros-bridge$ ros2 launch carla_shenlan_bridge_ego_vis carla_bridge_ego_vehilce.launch.py 
```

### 终端窗口3
```
nick@nick-PC:~/carla-ros-bridge$ source source_env.sh 
nick@nick-PC:~/carla-ros-bridge$ ros2 launch carla_shenlan_bridge_ego_vis carla_bridge_ego_vehilce.launch.py 
```

### 终端窗口4
```
nick@nick-PC:~/carla-ros-bridge$ rqt
打开 Plugins -> Visualiztion -> Plot
添加 以下话题
当前速度
/carla/ego_vehicle/vehicle_status/velocity
目标速度
/carla/ego_vehicle/target_velocity/velocity
```

## 要点分析
### 1. 目标速度来自于参考路径最近点的速度
### 2. 使用PID算法，控制油门&刹车，来跟踪目标速度
