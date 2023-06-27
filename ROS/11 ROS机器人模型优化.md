# ROS机器人模型优化

## URDF建模问题

- 模型冗长，重复内容多
- 参数修改麻烦，不利于二次开发
- 没有参数计算功能

## xacro文件

- 精简模型代码
  - 创建宏文件
  - 文件包含
- 提供可编程接口
  - 常量
  - 变量
  - 数学计算
  - 条件语句

## URDF模型优化

### 常量定义

```xml
<!--name是定义的常量名，value是常量的值-->
<xacro:property name="M_PI" value="3.14159"/>
```

### 常量使用

```xml
<!--常量 ${ } 在括号里使用常量 在括号里面可以进行运算-->
<origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
```

### 数学计算

```xml
<!--括号里面可以进行运算-->
<origin xyz="0 ${(motor_length+wheel_length)/2} 0" rpy="0 0 0"/> 
```

> 所有数学运算都会转换成浮点数进行，以保证运算精度

### 宏定义

```cpp
<!--name是宏定义的名字类似函数名，params是类似函数参数，可以是字符串-->
<xacro:macro name="name" params="A B C">
   ......具体模型定义（类似函数内容）
</xacro:macro> 
```

### 宏调用

```cpp
<!--A_value，B_value，C_value是宏的名称-->
<name A="A_value" B="B_value" C="C_value" />
```

==`noetic`版本要使用`<xacro：mrobot_body/>`调用==

### 文件包含

```cpp
<!--$(find+功能包)=包的具体路径-->
<xacro:include filename="$(find mbot_descripiton)/urdf/xacro/mbot_base.xacro" />
```



## 用xacro创建小车机器人

将xacro文件转化成URDF文件后显示

```shell
rosrun xacro xacro.py mbot.xacro>
```

直接调用xacro文件解析器

```xml
<arg name-="model" default=$(find xacro)/xacro--inorder"'$(find mbot_description)/urdf/xacro/mbot.xacro'"/>

<param name="robot_description" commmand="$(arg model)"/>
```





































































