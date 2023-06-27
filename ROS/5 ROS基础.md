

# ROS基础

## 工作空间

工作空间(workspace)是存放工程开发相关文件的文件夹

- src：代码空间(Source Space)
- build：编译空间(Build Space)
- devel：开发空间(Development Space)
- install：安装空间(Install Space)**与devel功能重复**

![image-20221213180329670](/home/suyu/.config/Typora/typora-user-images/image-20221213180329670.png) 

### 创建工作空间

#### 创建工作空间

```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
 catkin_init_workspace#这样就在src文件中创建了一个 CMakeLists.txt 的文件，目的是告诉系统，这个是ROS的工作空间。实际上是自动空配置了Cmake
```

#### 编译工作空间

```shell
cd ~/catkin_ws/
catkin_make#发现catkin_ws中多了两个文件 build 和 devel。实际上，catkin_make是对cmake的改进
```

#### 设置环境变量

```shell
source devel/setup.bash#实际上是启动devel里的脚本文件自动修改环境变量；但是只会在当前终端下生效
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc#把上一行命令写入.bashrc就可以每个终端都有    
```

#### 检查环境变量

```shell
echo $ROS_PACKAGE_PATH 
>>/home/suyu/catkin_ws/src:/opt/ros/noetic/share 
```



### 创建功能包

```shell
catkin_create_pkg  <package_name>   [depend1]  [depend2]
```

#### 创建功能包

```shell
cd   ~/catkin_ws/src
catkin_create_pkg learning_communication std_msgs rospy roscpp
```

#### 编译功能包

```shell
cd ~/catkin_ws 
catkin_make

#编译成功记得重新source一下，不然可能找不到这个功能包。
#source 一下工程路径下devel内的setup.bash文件
source ~/catkin_ws/devel/setup.bash
```

**同一个工作空间下，不允许存在同名功能包；不同工作空间下可以**

#### 工作空间的覆盖：Overlaying机制

```shell
env | grep ros

PKG_CONFIG_PATH=/opt/ros/noetic/lib/pkgconfig
ROS_PACKAGE_PATH=/home/suyu/catkin_ws/src:/opt/ros/noetic/share#功能包
ROS_ETC_DIR=/opt/ros/noetic/etc/ros
CMAKE_PREFIX_PATH=/home/suyu/catkin_ws/devel:/opt/ros/noetic
PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages
LD_LIBRARY_PATH=/home/suyu/catkin_ws/devel/lib:/opt/ros/noetic/lib
PATH=/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin
ROS_ROOT=/opt/ros/noetic/share/ros

```

```shell
ROS_PACKAGE_PATH=/home/suyu/catkin_ws/src:/opt/ros/noetic/share
#寻找功能包时，会首先寻找/home/suyu/catkin_ws/src，如果没有，继续寻找/opt/ros/noetic/share下的
```

- 工作空间的路径依次在`ROS_PACKAGE_PATH`环境变量中记录
- 新设置的路径在`ROS_PACKAGE_PATH`中会被自动放置到最前端
- 运行时，ROS会优先查找最前端的工作空间中是否存在指定的功能包
- 如果不存在，就顺序向后查找其他工作空间

```shell
rospack find <package_name>
```





