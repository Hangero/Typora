# URDF机器人建模

## `<robot>`

完整机器人模型的**最顶层标签**，< link > 和 < joint >标签必须包含在 < robot > 标签内

![image-20221216175131727](/home/suyu/.config/Typora/typora-user-images/image-20221216175131727.png)

## `<link>`

用于描述机器人**某个刚体部分的外观和物理属性**（比如尺寸（size）、颜色（color）、形状（shape）、惯性矩阵（inertial matrix）、碰撞属性（collision properties）等）

- `<visual>`：描述机器人link部分的外观参数
- `<inertial>`：描述link的惯性参数 
- `<collision>`：描述link的碰撞属性

![image-20221216174357962](/home/suyu/.config/Typora/typora-user-images/image-20221216174357962.png)

```xml
<link name="<link name>">
<!-- 描述机器人link部分的外观参数, 尺寸、颜色、形状等的外观信息 -->
	<inertial>......</inertial>
	<!-- 描述link的惯性参数，主要用到机器人动力学的运算部分 -->
	<visual>......</visual>
	<!-- 描述link的碰撞属性 -->
	<collision>......</collision>
</link>
```



## `<joint>`

描述机器人关节的**运动学和动力学属性**，包括关节运动的位置和速度限制。根据关节的运动形式，可以将其分为六种类型

| 关节类型   | 描述                                             |
| ---------- | ------------------------------------------------ |
| continuous | 旋转关节，可以围绕单轴无限旋转                   |
| revolute   | 旋转关节，类似于continuous，但是有旋转的角度限制 |
| prismatic  | 滑动关节，沿某一轴线移动的关节，带有位置极限     |
| planar     | 平面关节，允许在平面正交方向上平移或者旋转       |
| floating   | 浮动关节，允许进行平移、旋转运动                 |
| fixed      | 固定关节，不允许运动的特殊关节                   |

- `<calibration>`
- `<dynamics>`
- `<limit>`
- `<mimic>`：描述该关节与已有关节的关系
- `<safety_controller>`：描述安全控制器参数

![image-20221216175103041](/home/suyu/.config/Typora/typora-user-images/image-20221216175103041.png)

```xml
<joint name="<name of the joint>" type="<joint type>">
	<parent link="parent_link"/>
	<child link="child_link"/>
	<!--关节的参考位置，用来校准关节的绝对位置-->
	<calibration ....../>
	<!--描述关节的物理属性，例如物理静摩擦力-->
	<dynamics damping ....../>
	<!--描述运动的一些极限值，包括关节运动的上下限位置，速度限制-->
	<limit effort ....../>
	......
</joint>
```



## 创建URDF模型

### 创建功能包

```shell
#进入工作空间
cd ./catkin_ws/src/
#创建功能包
catkin_create_pkg mbot_description urdf xacro

cd ./catkin_ws/src/mbot_description
mkdir urdf
mkdir meshes
mkdir launch
mkdir config
```

- urdf：存放URDF或xacro文件
- meshes：放置URDF中的模型渲染文件
- launch：保存相关启动文件
- config：保存rviz的配置文件、功能包的配置文件

### 编辑launch文件

```xml
<launch>
<!-- 加载的参数名字叫robot_description，具体内容是urdf相关模型的路径 -->
<param name="robot_descrption" textfile="$(find mbot_description)/urdf/mbot_base.urdf"/>
<!-- 设置GUI参数，显示关节控制插件 -->
<param name="use_gui" value="true"/>
<!-- 运行joint_state_publisher节点，发布机器人的关节状态  -->
<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher"/>
<!-- 运行robot_state_publisher节点，根据joint_state的状态，创建并 发布tf  -->
<node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher"/>
<!-- 运行rviz可视化界面，保存每次打开之后的相关插件，保存到config文件夹下面 -->
<node name="rviz" pkg="rviz" type="rviz" args="-d $(find mbot_description)/config/mbot_urdf.rviz" required="true"/>
</launch>
```

- `joint_state_publisher`：发布每个joint的状态，而且可以通过UI界面对joint进行控制
- `robot_state_publisher`：

### 编辑URDF文件

```xml
<?xml version="1.0" ?>
<robot name="mbot">


    <link name="base_link">
        <visual>
            <origin xyz=" 0 0 0" rpy="0 0 0" />
            <!-- link坐标位置，放在最中央 ，xyz单位是米，rpy单位是弧度-->
            <geometry>
                <cylinder length="0.16" radius="0.20"/>
            </geometry>
            <!-- 机器人的外观效果，使用圆柱体，高0.16，半径0.20 -->
            <material name="yellow">
                <color rgba="1 0.4 0 1"/>
            </material>
             <!--  通过rgba来描述颜色，颜色命名为yellow,a为1是不透明-->
        </visual>
        <!--visual指的是一些物理属性的标签 -->
    </link>

        <joint name="left_wheel_joint" type="continuous">
    <!--   joint名字为left_wheel_joint，属性为continuous，即无限旋转-->
        <origin xyz="0 0.19 -0.05" rpy="0 0 0"/>
        <!--  坐标位置，在base_link基础上，Y偏移0.19，z偏移-0.05-->
        <parent link="base_link"/>
        <child link="left_wheel_link"/>
        <!--  主关节：base_link，上面的圆柱形车体-->
        <!--  子关节：left_wheel_link，左轮-->
        <!--  主关节最后是一个，本代码是base_link-->
        <axis xyz="0 1 0"/>
        <!--left_wheel_link绕某一个轴做旋转，指定为Y轴  -->
    </joint>
     <!--joint用来连接两个link-->
    <link name="left_wheel_link">
        <visual>
            <origin xyz="0 0 0" rpy="1.5707 0 0" />
            <geometry>
                <cylinder radius="0.06" length = "0.025"/>
            </geometry>
            <material name="white">
                <color rgba="1 1 1 0.9"/>
            </material>
        </visual>
    </link>
  <!--左轮信息-->

</robot>

```

### 加载纹理信息

```xml
    <!--Kinect配置-->
    <link name="kinect_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 1.5708"/>
            <geometry>
                <mesh filename="package://mbot_description/meshes/kinect.dae" />
                 <!--直接加载Kinect的外观纹理描述文件-->
            </geometry>
        </visual>
    </link>
    <joint name="laser_joint" type="fixed">
        <origin xyz="0.15 0 0.11" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="kinect_link"/>
    </joint>

```

在meshes文件中创建纹理描述信息。
新建文件名称： `kinect.dae`

### 检查URDF模型整体结构

```shell
urdf_to_graphiz mbot_with_camera.urdf
```

