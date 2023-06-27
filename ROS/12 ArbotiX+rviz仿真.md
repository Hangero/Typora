# ArbotiX+rviz仿真

## AbbotiX

- 一款控制电机、舵机的硬件控制板
- 提供了相应的ROS功能包
- 提供了一个**差速控制器**，通过接受速度控制指令，更新机器人的里程计状态

### 安装

```shell
命令行直接调用：
	sudo apt-get install ros-noetic-arbotix
git安装源码：
	git clone https://github.com/vanadiumlabs/arbotix_ros.git
```

下载后调用catkin_make编译完成即可；


### 配置Arbotix控制器

#### 创建launch文件

```xml
<node name="arbotix" pkg="arbotix_python" type="arbotix_driver" output="screen">
        <rosparam file="$(find mrobot_description)/config/fake_mrobot_arbotix.yaml" command="load" />
        <param name="sim" value="true"/>
    </node>
```

基本要求：

- 调用了arbotix_python 功能包下的 arbotix_driver 节点
- 通过 file 加载配置文件获取机器人信息以便驱动运动实现
-  配置 sim 为 true，完成仿真环境要


#### 创建配置文件

```yaml
controllers: {
   base_controller: {
   type: diff_controller, #控制器类型：差速控制器
   base_frame_id: base_footprint, #控制的坐标系
   base_width: 0.26, #两轮间距；因为是差速控制，所以必须要知道间距
   ticks_meter: 4100, #控制频率
   Kp: 12, #PID控制
   Kd: 12, 
   Ki: 0, 
   Ko: 50, 
   accel_limit: 1.0 #加速度限制
   }
}
```



#### 启动仿真器

```shell
roslaunch mbot_description arbotix_mbot_with_camera_xacro.launch
```





## ArbotiX+rviz功能仿真
















