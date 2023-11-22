# aubo_ros2_driver

遨博机器人ROS2驱动

# 在 rviz 中 查看 aubo 机器人模型（以 aubo_i5 为例）
```bash
ros2 launch aubo_description aubo_viewer.launch.py
```
# 驱动真实机械臂 aubo_i5 (修改机器人对应 robot_ip aubo_type)
```bash 
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5 robot_ip:=192.168.127.128  
 use_fake_hardware:=false
ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=aubo_i5

```
# 驱动真实机械臂 aubo_i5单点轨迹执行demo (修改机器人对应 robot_ip aubo_type)
```bash 
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5 robot_ip:=192.168.127.128  
 use_fake_hardware:=false
ros2 launch ros_joints_plan joints_plan.launch.py aubo_type:=aubo_i5
```
