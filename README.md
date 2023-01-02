
### 编译整个工程代码
```
nick@nick-hp:~/carla-ros-bridge$ source source_env.sh 
nick@nick-hp:~/carla-ros-bridge$ colcon build
```

### 启动流程

#### 终端窗口1  启动carla仿真服务器
```
nick@nick-PC:~/workspaces/carla-0.9.13/CARLA_0.9.13$ ./CarlaUE4.sh -quality-level=Epic -resx=800 -resy=600 -carla-rpc-port=2000
```

#### 终端窗口2  启动carla-ros-bridge
```
nick@nick-PC:~/carla-ros-bridge$ source source_env.sh 
nick@nick-PC:~/carla-ros-bridge$ ros2 launch carla_shenlan_bridge_ego_vis carla_bridge_ego_vehilce.launch.py 
```

#### 终端窗口3  启动LQR轨迹跟随算法,并使用rviz2可视化轨迹
```
nick@nick-hp:~/carla-ros-bridge$ source source_env.sh 
nick@nick-hp:~/carla-ros-bridge$ ros2 launch carla_shenlan_lqr_pid_controller lqr_launch.py 
```
