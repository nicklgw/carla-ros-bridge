
终端窗口1  启动carla仿真服务器
nick@nick-PC:~/workspaces/carla-0.9.13/CARLA_0.9.13$ ./CarlaUE4.sh -quality-level=Epic -resx=800 -resy=600 -carla-rpc-port=2000

终端窗口2  启动carla-ros-bridge
nick@nick-PC:~/carla-ros-bridge$ source source_env.sh 
nick@nick-PC:~/carla-ros-bridge$ ros2 launch carla_shenlan_bridge_ego_vis carla_bridge_ego_vehilce.launch.py 

终端窗口3  启动Stanley轨迹跟随算法
nick@nick-hp:~/carla-ros-bridge$ source source_env.sh 
nick@nick-hp:~/carla-ros-bridge$ ros2 run carla_shenlan_stanley_pid_controller carla_shenlan_stanley_pid_controller_node 

终端窗口4  使用rviz2可视化轨迹
nick@nick-hp:~/carla-ros-bridge$ source source_env.sh 
nick@nick-hp:~/carla-ros-bridge$ rviz2 
用rviz2加载配置config/visualization.rviz
[File]->[Open Config]->config/visualization.rviz



################################################################################
【自动驾驶】Stanley（前轮反馈）实现轨迹跟踪 | python实现 | c++实现
https://blog.csdn.net/weixin_42301220/article/details/124899547

从这篇文章的贴图，可以看出
横向误差ey 为 【最近路径点】 在 坐标系 O(车体x，车体y，Ψt) 下， 纵坐标y的值。
TF转换



