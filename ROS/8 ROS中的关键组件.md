# ROS中的关键组件

## Launch文件

通过XML文件实现多节点的配置和启动（**可自启动ROS Master**）

```xml
<launch>
<！--
local machine already has a definition by default.
This tag overrides the default definition with
specific ROS ROOT and ROS PACKAGE PATH values --
<machine name="local alt"address="localhost"default="true"ros-root="/u/user/ros/ros/"ros-package-path="/u/user/ros/ros-pkg"/
<!--a basic listener node -->
<node name="listener-1"pkg="rospy tutorials"type="listener"/>
<!--pass args to the listener node -->
<node name="listener-2"pkg="rospy_tutorials"type="listener"args="-foo arg2"/>
<!--a respawn-able listener node -->
<node name="listener-3"pkg="rospy_tutorials"type="listener"respawn="true"/>
<!--start listener node in the 'wgl'namespace -->
<node ns="wg1"name="listener-wgl"pkg="rospy_tutorials"type="listener"respawn="true"/>
<!--start a group of nodes in the 'wg2'namespace -->
<group ns="wg2">
<!--
remap applies to all future statements in this scope.--
<remap from="chatter"to="hello"/>
<node pkg="rospy_tutorials"type="listener"name="listener"args="--test"respawn="true"/>
<node pkg="rospy_tutorials"type="talker"name="talker">
<!--set a private parameter for the node -->
<param name="talker 1 param"value="a value"/
<!--nodes can have their own remap args -->
<remap from="chatter"to="hello-1"/>
<!--you can set environment variables for a node -->
<env name-"ENV EXAMPLE"value-"some value"/
</node>
</group>
</launch>
```

### `<launch>`

launch文件中的根元素采用`<launch>`标签定义

### `<node>`

启动节点

```xml
<node pkg="package-name" tyoe="executable-name" name="node-name"/>
```

- `pkg`：节点所在功能包名
- `type`：节点的可执行文件
- `name`：节点运行时的名称
- `output  respawn required ns args`

### 参数设置标签

#### `<param>/<rosparam>`

设置ROS系统总运行的参数，存储在参数服务器中。

```xml
<param name="output_frame" value="odom"/>
```

- `name`：参数名
- `value`：参数值

加载参数文件中的多个参数：

```xml
<rosparam file="param.yaml" command="load" ns="param"/>
```

#### `<arg>`

launch文件内的局部变量，仅限于launch文件使用，文件内传参

```xml
<arg name="arg-name" default="arg-value"/>
```

- `name`：参数名
- `value`：参数值

调用：

```xml
<param name="foo" value="$(arg arg name)"/>
<node name="node" pkg="package" type="type" args="$(arg arg-name)"/>
```



### 重映射`<remap>`

修改接口

### 嵌套`<include>`

包含其他launch文件，类似头文件包含

```xml
<include file="$(dirname/other.launch)"/>
```

- `file`：包含其他launch文件路径



## TF坐标变换

- 广播TF变换
- 监听TF变换

```shell
rosrun tf view_frames
rosrun tf tf_echo turtle1 turtle2
```

### 实现TF广播器

- 定义TF广播器
- 创建坐标变化值
- 发布坐标变化

```cpp
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;
void poseCallback(const turtlesim::PoseConstPtr& msg){
    //tf广播器
    static tf::TransformBoradcaster br;
    
    //根据乌龟当前的位姿，设置相对于世界坐标系的坐标变换
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->x,msg->y,0.0));
    tf::Quaternion q;
    q.setRPY(0,0,msg->theta);
    transform.setRotation(q);
    
    //发布坐标变换
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world",turtle_name));
}

int main(int argc,char **argv){
    //初始化节点
    ros::init(argc,argv "my_tf_broadcaster");
    if(argc！=2){
        ROS_ERROR("need turtle name as arguement");
        return -1;
    }
    turtle_name = argv[1];
    
    //订阅乌龟的pose信息
    ros::NodeHandle node;
    ros::Subscriber sub  = node.subscribe(turtle_name+"/pose",10,&poseCallback);
    
    ros::spin();
    
    return 0;
}
```

### 实现TF监听器

- 定义TF监听器
- 查找坐标变换

```cpp
int main(int argc,char**argv){

//初始化Ros节点
ros:init(argc,argv,"my_tf_listener");
//创建节点句柄
ros:NodeHandle node;
//请求产生turtle2通过service的Client,发布/spawn服务来生成一只新海龟
ros:service:wattForservice("/spawn");
ros:Serviceclient add_turtle node.serviceclient<turtlesim:Spawn>("/spawn");
turtlesim:Spawn srv;
add turtle.call(srv);
//创建发布turtle2速度控制指令的发布者创建Publisher,发布速度的数据Twist
ros:Publisher turtle_vel node.advertise<geometry_msgs:Twist>("/turtle2/cmd_vel",10);
//创建tf的监听器
tf:TransformListener
listener;//来监听两个坐标系之间的关系
ros::Rate rate(1o.o);//循环频率
whtle (node.ok()){
    //获取turtle:1与turtle:2坐标系之间的tf数据,先等待变换waitForTransform,看是否存在/代urtle2和
	tf:StampedTransform transform;建立变换矩阵
//代turtle1的坐标系，如果存在跳到lookup。等待的是当前的时间Time(0),等待3秒Duration(3.0)
    try{
    listener.waitForTransform("/turtle2","/turtle1",ros:Time(0),ros:Duration(3.0));
	listener.lookupTransform("/turtle2","/turtle1",ros:Time(0),transform);
	}
//坐标系如果存在，执行lookupTransform:查询这俩坐标系之间的关系。查询当前的时间Time(O),结果保存到transform
	catch (tf:TransformException &ex){
	ROS ERROR("%s",ex.what());
	ros:Duration(1.0).sleep();
	continue;
}

//得到变换矩阵后，准备让turtle2向turtle1移动
///根据turtle1与turtle:2坐标系之间的位置关系，发布turtle2的速度控制指令
	geometry_msgs:Twist vel_msg;
	vel_msg.angular.z=4.0atan2(transform.getOrigin().y()t,transform.getOrigin().x());//定义角速度
	vel_msg.linear,×=0.5*sqrt(pow(transform.getOrigin().x(),2)+turtle_vel.publish(vel_msg);
//定义线速度，乘0.5表示希望在2秒内pow(transform.getorigin().y(),2)i到达turtle1的位置
rate.sleep();
}

return 0;
}                          
```

### 编译



### launch文件启动



```xml
<launch>
<!--海龟仿真-->
<node pkg="turtlesim" type="turtlesim_node" name="sim"/>
 
<!--键盘控制-->
<node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen"/>
    
<!--两只海龟的tf广播-->
<node pkg="learning_tf" type="turtle_tf_broadcaster" args="/turtles1" name="turtle1_tf_broadcaster"/>
<node pkg="learning_tf" type="turtle_tf_broadcaster" args="/turtles2" name="turtle2_tf_broadcaster"/>

<!--监听tf广播，并且控制turtle2移动-->
 <node pkg+"learning_tf" type="turtle_tf_listener" name="listener"/>
    
</launch>
```



## Qt工具箱

### 日志输出工具`rqt_console`



### 计算图可视化`rqt_gtaph`

![image-20221215193959349](/home/suyu/.config/Typora/typora-user-images/image-20221215193959349.png)



### 数据绘图工具`rqt_plot`



### 参数动态配置工具`rqt_reconfigure`

![image-20221215194634072](/home/suyu/.config/Typora/typora-user-images/image-20221215194634072.png)



## Rviz可视化平台

![image-20221215200642305](/home/suyu/.config/Typora/typora-user-images/image-20221215200642305.png)

![image-20221215200707251](/home/suyu/.config/Typora/typora-user-images/image-20221215200707251.png)



## Gazebo物理仿真环境

![image-20221215201049255](/home/suyu/.config/Typora/typora-user-images/image-20221215201049255.png)

![image-20221215201122522](/home/suyu/.config/Typora/typora-user-images/image-20221215201122522.png)

![image-20221215201244126](/home/suyu/.config/Typora/typora-user-images/image-20221215201244126.png)

























