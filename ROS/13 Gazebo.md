# Gazebo

## 'ros_control'

- ROS为开发者提供的控制中间件
- 包含一系列控制器接口、传动装置接口、硬件接口、控制器工具箱等等
- 可以帮助机器人应用功能包快速落地，提高开发效率

![image-20221227094853332](/home/suyu/.config/Typora/typora-user-images/image-20221227094853332.png)



- 控制器管理器
  提供一种通用的接口来管理不同的控制器。
- 控制器
  读取硬件状态，发布控制命令，完成每个
  joint的控制。
- 硬件资源
  为上下两层提供硬件资源的接口。
- 机器人硬件抽象
  机器人硬件抽象和硬件资源直接打交道，通过
  write和read方法完成硬件操作。
- 真实机器人
  执行接收到的命令。



![image-20221227095103258](/home/suyu/.config/Typora/typora-user-images/image-20221227095103258.png)



控制器(Controllers):

- joint_effort_controller
- joint_state_controller
- joint_position_controller
- joint_velocity_controller



## 配置机器人模型



## 创建仿真环境



## 开始仿真





















