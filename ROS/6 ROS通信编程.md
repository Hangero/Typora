# ROS通信编程

## 话题通信

### 话题编程流程

- 创建发布者
- 创建订阅者
- 添加编译选项
- 运行可执行程序

### 实现发布者

- 初始化ROS节点
- 向ROS Master注册节点消息，包括发布的话题名和话题中的消息类型
- 按照一定的频率循环发布消息

```cpp

#include "sstream"
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc ,char **argv)
{
    ///ROS节点初始化
    ros::init(argc,argv,"talker");

    ///创建节点句柄
    ros::NodeHandle n;

    ///创建一个Publisher,发布名为chatter的topic，消息类型为std_msgs::String
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",1000);//1000是队列长度

    ///设置循环频率
    ros::Rate loop_rate(10);

    int count=0;
    while(ros::ok()){
        ///初始化std_msgs::String类型的消息
        std_msg::String msg;
        std::stringstream ss;
        ss<<"hello world"<<count;
        msg.data = ss.str();

        ///发布消息
        ROS_INFO("%s",msg.data.c_str());
        chatter_pub.publish(msg);

        ///循环等待回调函数
        ros::spinOnce();

        ///按照循环频率延时，一定要加上sleep，不然会大量的占用cpu资源
        loop_rate.sleep();
        ++count;
    }
	return 0
}
```

### 实现订阅者

- 初始化ROS节点
- 订阅需要的话题
- 循环等待话题消息，接受到消息后进入回调函数
- 在回调函数值中完成消息处理

```cpp
#include "ros/ros.h"
#include "std_msg/String.h"

void chatterCallBack(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I head: [%s]",msg->data.c_str());
}


int main(int argc ,char **argv)
{
    ///ROS节点初始化
    ros::init(argc,argv,"listerner");

    ///创建节点句柄
    ros::NodeHandle n;

    ///创建一个Subscribe,发布名为chatter的topic，注册回调函数为chatterCallBack
    ros::Subscriber sub= n.subscribe("chatter",1000, chatterCallBack);//1000是队列长度

    ///循环等待回调函数
    ros::spin();

    return 0;
}
```

### 编译代码

- 设置需要的编译代码和生成的可执行文件
- 设置链接库
- 设置依赖

### 自定义话题消息

- 定义msg文件

- 在`package.xml`中添加功能包依赖

  ```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
  ```

- 在`CMakeLists.txt`添加编译选项

  ```cmake
  find_package(... message_generation)
  
  catkin_package(CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime)
  
  add_message_files(FILES Person.msg)
  generate_messages(DEPENDENCIES std_msgs)
  ```

- 编译后，查看

  ```shell
  rosmsg show Person
  >>[learning_communication/Person]:
  uint8 unknown=0
  uint8 male=1
  uint8 female=2
  string name
  uint8 sex
  uint8 age
  
  rosmsg show learning_communication/Person
  >>uint8 unknown=0
  uint8 male=1
  uint8 female=2
  string name
  uint8 sex
  uint8 age
  
  
  ```

  

## 服务通信

- 创建服务器
- 创建客户端
- 添加编译选项
- 运行可执行程序

###  自定义服务请求与应答

- 定义srv文件

- 在`package.xml`文件中添加功能包依赖

  ```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
  ```

- 在`CMakeLists.txt`添加编译选项

  ```cmake
  find_package(... message_generation)
  
  catkin_package(CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime)
  
  add_mservice_files(FILES AddTwoInts.srv)
  ```

  

```srv
int64 a
int64 b
---
int64 sum
```

上面两个是服务的请求数据，下面是服务的应答数据

### 实现服务器

- 初始化ROS节点
- 创建Serer实例
- 循环等待服务请求，进入回调函数
- 在回调函数中完成服务功能的处理，并反馈应答和数据

### 编译代码

- 设置需要编译的代码和生成的可执行文件
- 设置链接库
- 设置依赖

```cmake
add_executable(server src/server.cpp)
target_link_libraries(server ${catkin_LIBRARIES})
add_dependencies(server ${PROJECT_NAME}_gencpp)

add_executable(client src/client.cpp)
target_link_libraries(client ${catkin_LIBRARIES})
add_dependencies(client ${PROJECT_NAME}_gencpp)
```



## 动作编程

### 动作（action）

- 一种问答机制
- 带有连续反馈
- 可以在任务过程中止运行
- 基于ROS的消息机制实现

### Action接口

- goal：发布任务目标；
- cancel：请求取消任务
- status：通知客户端当前的状态
- feedback：周期反馈任务运行的监控数据
- result：向客户端发送任务的执行结果，只发布一次

![image-20221215103754858](/home/suyu/.config/Typora/typora-user-images/image-20221215103754858.png)



![image-20221215103806914](/home/suyu/.config/Typora/typora-user-images/image-20221215103806914.png)



### 自定义动作消息

- 定义action文件

- 在`package.xml`中添加功能包依赖

  ```xml
  <build_depend>actionlib</build_depend>
  <build_depend>actionlib_msgs</build_depend>
  <exec_depend>actionlib</exec_depend>
  <exec_depend>action_msgs</exec_depend>
  ```

- 在`CMakeLists.txt`添加编译选项

  ```cmake
  find_package(catkin REQUIRED actionlib_msgs actionlib)
  
  add_action_files(DIRECTORY action FILES DoDishes.action)
  
  generate_messages(DEPENDENCIES actionlib_msgs)
  ```

  

### 实现动作服务器

- 初始化ROS节点
- 创建动作服务器实例
- 启动服务器，等待动作请求
- 在回调函数中完成动作服务功能的处理，并反馈进度信息
- 动作完成，发送结束信息





### 实现动作客户端

- 初始化ROS节点
- 创建动作客户端实例
- 连接动作服务器
- 发送动作目标
- 根据不同类型的服务段反馈处理回调函数



### 添加编译选项

```cmake
add_executable(DoDishes_client src/DoDishes_client.cpp)
target_link_libraries(DoDishes_client ${catkin_LIBRARIES})
add_dependencies(DoDishes_client ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(DoDishes_server src/DoDishes_server.cpp)
target_link_libraries(DoDishes_server ${catkin_LIBRARIES})
add_dependencies(DoDishes_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

```





















