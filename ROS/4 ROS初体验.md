# ROS初体验

## 重要目录

- include
- lib
- share

## roscore

```shell
ROS_MASTER_URI=http://Hangero:11311/
#当前运行ROSMaster的主机的资源地址；因为时分布式软件框架，且只有一个ROSMaster，故需要明确地址
```



![image-20221210233120222](/home/suyu/.config/Typora/typora-user-images/image-20221210233120222.png)



## 常用命令

| 命令              | 作用                       |
| ----------------- | -------------------------- |
| catkin_create_pkg | 创建功能包                 |
| rospack           | 获取功能包信息             |
| catkin_make       | 编译工作空间中的功能包     |
| rosdep            | 自动安装功能包依赖的其他包 |
| roscd             | 功能包目录跳转             |
| roscp             | 拷贝功能包中的文件         |
| rosrun            | 运行功能包中的可执行文件   |
| roslaunch         | 运行启动文件               |
| rosed             | 编辑功能包中的文件         |



## 小海龟

```shell
rosrun turtlesim turtlesim_node

rosrun turtlesim turtle_teleop_key

```

```shell
rosnode info turtlesim_node


contacting node http://Hangero:35143/ ...
                                                         │Pid: 13330
                                                         │Connections:
                                                         │ * topic: /rosout
                                                         │    * to: /rosout
                                                         │    * direction: outbound (42281 - 192.168.5.89:51746) [3
                                                         │0]
                                                         │    * transport: TCPROS
                                                         │ * topic: /turtle1/cmd_vel
                                                         │    * to: /teleop_turtle (http://Hangero:36511/)
                                                         │    * direction: inbound (34870 - Hangero:54419) [32]
                                                         │    * transport: TCPROS #通讯协议

```

```shell
rostopic info /turtle1/cmd_vel
rostopic echo /turtle1/cmd_vel
```



```shell
#发布一次测试数据
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist "linear:
 x: 20
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 0.0" 
 #发布频率每秒10次
 rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist "linear:
```



```shell
rosservice list
rosservice call /spawn 
```



```shell
rqt_plot#绘制图像
```



![image-20221213174912367](/home/suyu/.config/Typora/typora-user-images/image-20221213174912367.png)

## 应用框架

![image-20221213174935078](/home/suyu/.config/Typora/typora-user-images/image-20221213174935078.png)



























