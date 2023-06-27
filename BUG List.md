#  BUG List



> 人类从历史中学到的唯一教训，就是人类无法从历史中学到任何教训。
>
> ​                                                                                                                                                 ——黑格尔

##  Linux

### 常见

1. gedit   ~/.bashrc 

   `.bashrc`是home目录下的一个shell文件，用于储存用户的个性化设置。在bash每次启动时都会加载`.bashrc`文件中的内容，并根据内容定制当前bash的配置和环境。[.bashrc介绍](https://blog.csdn.net/Heyyellman/article/details/111565781?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165555441916781435475651%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165555441916781435475651&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-111565781-null-null.142^v17^pc_rank_34,157^v15^new_3&utm_term=.bashrc+&spm=1018.2226.3001.4187)。`gedit   ~/.bashrc `可以读取`.bashrc`并进入修改模式。

### shell

1. ```shell
   first value:100
   second value:200
   ./test.sh: 第 5 行: test: 100: 需要一元表达式
   相等:2
   ./test.sh: 第 8 行: test: 100: 需要一元表达式
   greater:2
   ./test.sh: 行 11: [100: 未找到命令
   less:127
   
   ```

   把`data2`写成了`date2`

2. ```shell'
   ./test.sh
   1's value :100
   2's value :200
   和的最大值为:0
   
   ```

   ```shell
   #!/bin/bash
   
   function my_max()
   {
           if [ $1 -gt $2 ]; then
                   return $1
           else
                   return $2
           fi
   }
   read -p "1's value :" data1     #键盘读入，这里不用`$data1`
   read -p "2's value :" data2
   my_max $data1 $data2
   echo "$data1和$data2的最大值为:$?"
   
   ```

3. ` ps -ef |grep helloworld.cpp`搜索到PID，但是`kill`和`top`显示无进程。

   ```shell
    $ ps -ef | grep helloworld.cpp
    suyu     27031 25827  0 09:05   pts/0    00:00:00 grep --color=auto helloworld.cpp
    
    $ ps -ef | grep helloworld
   suyu     26961 26704  0 09:04    pts/3    00:00:00 /home/suyu/week01/helloworld
   suyu     27173 25827  0 09:08    pts/0    00:00:00 grep --color=auto helloworld
    #UID    PID     PPID     C STIME  TTY       TIME         CMD
   ```

   |       |                                                              |
   | ----- | ------------------------------------------------------------ |
   | UID   | 该进程执行的用户id                                           |
   | PID   | 进程id                                                       |
   | PPID  | 该进程的父进程id；若无进程，则称为僵尸进程（parent process ID） |
   | C     | CPU的占用率，其形式是百分数                                  |
   | STIME | 进程的启动时间                                               |
   | TTY   | 发起该进程的设备识别符号（本次中，该终端为pts/0，VS Code中的终端为pts/3） |
   | TIME  | 进程执行时间                                                 |
   | CMD   | 进程名称或路径                                               |

问题一：文件名与进程名

![](/home/suyu/桌面/Typora/image/BUG List/20201208211600568.png)

`hello.c`是文件，`hello`是进程。

本次中，helloworld.cpp的路径是`/home/suyu/week01/helloworld.cpp`

​                 helloworld进程的路径是`/home/suyu/week01/helloworld`

问题二：

两次查找均出现了一个相似的东西

```shell
 $ ps -ef | grep helloworld.cpp
 suyu     27031 25827  0 09:05   pts/0    00:00:00 grep --color=auto helloworld.cpp
 
  $ ps -ef | grep helloworld
suyu     27173 25827  0 09:08    pts/0    00:00:00 grep --color=auto helloworld
```

这里找到的，应该都是“本次搜索”（TTY均为pts/0）。我的理解是，像清点人数不要忘了自己，在查找含`helloworld.cpp`或`helloworld`的进程时，不要忘了本次搜索**“本身”**也含有`helloworld.cpp`或`helloworld`。其中`--color=auto`的意思是：对匹配内容高亮显示。

4. pthread不是Linux下默认的库，cmake要自己链接

##   ROS

1. RLExceplition:[xx.launch] is neither a ...

   需要设置环境变量。

   ```shell
   gedit ~/.bashrc 
   source ~/catkin_ws/devel/setup.bash
   ```

2. ```shell
   Could not find the GUI 
   ```

   升级`joint_state_publish_gui`把launch文件中的`joint_state_publisher`修改为`joint_state_publish_gui`。

3. 一定要在相关目录下运行`check_urdf`才能正确的检查urdf文件。

4. 编译空间时，确保工作变量可以找到功能包。可以用`rospack`去寻找。[rospack+roscd+rosls](https://blog.csdn.net/qq_42910179/article/details/106868560?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165555531216782184620676%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165555531216782184620676&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-106868560-null-null.142^v17^pc_rank_34,157^v15^new_3&utm_term=rospack&spm=1018.2226.3001.4187) 

```shell
$ rospack find [pack_name]
```

5. ```shell
   Invoking "cmake" failed
   ```

    

6. ROSModel报错 

   改`Frame`为`base_link`

7. ```shell
   Invoking "make cmake_check_build_system" failed
   ```

   不同功能包里不有重名的节点。

8. ```cpp
   Make sure file exists in package path and permission is set to excutable (chmod + x)
   ```

   注意文件的属性 ，本次是"scripts  >> mrobot_teleop.py"，“属性>>可执行”

9. 创建功能包

   ```shell
   cd   ~/catkin_ws/src
   catkin_create_pkg  <package_name>   [depend1]  [depend2]
   cd  ~/catkin_ws/src
   catkin_make
   ```

10. rviz中不能设置碰撞参数

11. urdf第一行不能修改，尽量去掉注释

12. ```cpp
    Error  reading  end  tag 
    ```

    检查标签是否成对

13. ```cpp
    Couldn't  save  project  < invalid  path >
    ```

    设置路径

14. 注意`oarm6.xacro`本身就是宏，引用时要加上`</xacro:macro`

15. ```cpp
    maximum recursion depth exceeded
    ```

    

16. - ```cpp
      Resource not found: The following package was not found in <arg default="$(find oarm6_moveit_config)/default_warehouse_mongo_db" name="db_path"/>: oarm6_moveit_config
      ROS path [0]=/opt/ros/melodic/share/ros
      ROS path [1]=/opt/ros/melodic/share
      ```
      
      

    [Building Common MoveIt Dependencies from Source in Catkin](https://blog.csdn.net/lwq123free/article/details/97007370?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165564144116782425159299%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165564144116782425159299&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~baidu_landing_v2~default-3-97007370-null-null.142^v17^pc_rank_34,157^v15^new_3&utm_term=default_warehouse_mongo_db&spm=1018.2226.3001.4187)

    当使用`rospack`去寻找`oarm6_moveit_config`时，发现找不到这个功能包，原因是没有写入路径，用`gedit ~/.bashrc`打开`.bashrc`,设置ROS功能包路径，然后检查，找到。

    ```shell
    rospack oarm6_description   //寻找功能包
    gedit ~/.bashrc   //打开.bashrc,设置ROS功能包路径
    source ~/.bashrc   //终端输入，让配置在当前终端生效
    echo $ROS_PACKAGE_PATH   //检查环境变量
    ```

    - 通过`rospack list`  ,发现找不到`oarm6_descrption`,` rospack list`显示所有的功能包，发现

      只有改动之前的`oarm6`。

      **记得要在`package.xml`和`CMakeList.txt`中同步修改名称**

      [更改ROS功能包的名称](https://www.guyuehome.com/35250)

17. ```cpp
    Invalid <arg> tag: moveit_config
    ROS path [0]=/opt/ros/indigo/share/ros
    ROS path [1]=/home/spark/my_ws/src
    ROS path [2]=/home/spark/catkin_ws/src
    ROS path [3]=/opt/ros/indigo/share
    ROS path [4]=/opt/ros/indigo/stacks. 
    
    Arg xml is <arg default="$(find moveit_config)/default_warehouse_mongo_db" name="db_path"/>
    The traceback for the exception was written to the log file
    ```

    moveit_config是一个类似元功能包，不能递归的编译，需要在src 根目录下编译

18. ```cpp
    <arg default="$(find oarm6_moveit_config)/default_warehouse_mongo_db" name="db_path"/>: oarm6_moveit_config
    ```

    `$(find oarm6_moveit_config)`是该包的路径，从/home开始写（不加$）就算绝对路径。

19. `env | grep ros`

    查看和ros相关的环境变量，关注 ROS_PACKAGE_PATH的日志信息。

20. - [ROS创建工作空间、功能包，编译示例程序](https://blog.csdn.net/qq_41667348/article/details/113484488?ops_request_misc=&request_id=&biz_id=102&utm_term=env%7Cgrep%20ros&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-0-113484488.142^v20^huaweicloudv1,157^v15^new_3&spm=1018.2226.3001.4187)
    - [关于ROS中找不到工作空间的功能包解决](https://blog.csdn.net/ypk138/article/details/120523830?spm=1001.2101.3001.6661.1&utm_medium=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-1-120523830-blog-123138693.pc_relevant_default&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-1-120523830-blog-123138693.pc_relevant_default&utm_relevant_index=1)

21. ```cpp
    RLException: error loading tag:
    file does not exist [3]
    XML is <rosparam command="load" file="3" ns="manipulator"/ >
    ```

    报错意思是没有kinematics.yaml文件，因此在设置组的时候填上3是不对的，点击后面的浏览文件按钮，选择在solidworks插件生成的config文件`（oarm6_descrption/config）`，而不是填3。古月书里moveit setup assistant助手设置 planning groups中，设置的是`Kin.Solver Attempts`,而现在是`Kin.parameters file`。

22. ROS目录用途解释

    [Linux各目录及每个目录的详细介绍](https://blog.csdn.net/xiao_yi_xiao/article/details/120492997?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165590571816782388062866%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165590571816782388062866&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-120492997-null-null.142^v20^huaweicloudv1,157^v15^new_3&utm_term=linux%E4%B8%AD%E7%9B%AE%E5%BD%95&spm=1018.2226.3001.4187)

    [Linux中的目录详解](https://blog.csdn.net/z17788055263/article/details/106629288?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165590571816782388062866%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165590571816782388062866&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-106629288-null-null.142^v20^huaweicloudv1,157^v15^new_3&utm_term=linux%E4%B8%AD%E7%9B%AE%E5%BD%95&spm=1018.2226.3001.4187)

23. [Moveit!碰撞检测添加模型](https://blog.csdn.net/anyingdaozhimi/article/details/109253898?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165590702916780366590112%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165590702916780366590112&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-3-109253898-null-null.142^v20^huaweicloudv1,157^v15^new_3&utm_term=moveit%E7%A2%B0%E6%92%9E%E6%A3%80%E6%B5%8B&spm=1018.2226.3001.4187)

24. ```cpp
    RLException: Invalid roslaunch XML syntax: no element found: line 42, column 0 The traceback for the exception was written to the log file
    ```

    检查标签是非完整对称。

    ```xml
    <launch>
    ...
    </launch>
    ```

25. ```cpp
    RLException: error loading <rosparam> tag: 
    	file does not exist [/home/suyu/catkin_ws/src/oarm6_description/config/oarm.yaml]
    XML is <rosparam command="load" file="$(find oarm6_description)/config/oarm.yaml"/>
    ```

    [Wiki](https://answers.ros.org/question/287342/launch-move_base-error-loading-rosparam-tag/)

    `oarm`错写成`oram`

26. roscd

27. RLException: error loading <rosparam> tag: 
    	file /home/suyu/catkin_ws/src/oarm6_description/config/oarm.yaml contains invalid YAML:
    while parsing a flow mapping
      in "<string>", line 13, column 14:
        controllers: {
                     ^
    expected ',' or '}', but got '<stream end>'
      in "<string>", line 15, column 1:
        

        ^

    XML is <rosparam command="load" file="$(find oarm6_description)/config/oarm.yaml"/>
    The traceback for the exception was written to the log file

    删去`{`  

28. No tf data.  Actual error: Fixed Frame [map] does not exist

    ```shell
    rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map xxx 100  //把xxx换掉
    ```

29. [ERROR] [1656077235.365596]: Unrecognized controller: follow_controller

30. 先执行订阅者，后执行发布者，发现不是从第一条数据开始接受，原因是发布者在发布前几条数据时，也在向ROS Master注册，注册后，才可以订阅

    ```cpp
    ros::Duration(3.0).sleep()//加入休眠，延迟第一条数据的发送
    ```

31. 注意头文件`std_msgs/String.h`是`msgs`

32. ```shell
    find_package(catkin) failed.  catkin was neither found in the workspace nor   in the CMAKE_PREFIX_PATH.  One reason may be that no ROS setup.sh was   sourced before.
    ```

    ***直接原因就是***： CMAKE_PREFIX_PATH中未找到 .catkin 

    因此，我们只需将 .catkin 的路径添加到CMAKE_PREFIX_PATH中即可。

    ```shell
    locate .catkin
    >> /home/suyu/catkin_ws/devel/.catkin
    
    echo " export CMAKE_PREFIX_PATH=/home/suyu/catkin_ws/devel/:$CMAKE_PREFIX_PATH">>~/.bashrc
    ```

    [解决](https://blog.csdn.net/Alex_wise/article/details/105201687?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522167100963416782412572282%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=167100963416782412572282&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-105201687-null-null.142^v68^control,201^v4^add_ask,213^v2^t3_esquery_v1&utm_term=%20find_package%28catkin%29%20failed.%20%20catkin%20was%20neither%20found%20in%20the%20workspace%20nor%20%20%20in%20the%20CMAKE_PREFIX_PATH.%20%20One%20reason%20may%20be%20that%20no%20ROS%20setup.sh%20was%20%20%20sourced%20before.&spm=1018.2226.3001.4187)

33. 编写srv，msg，action时**`uint`不要写错**

34. ```cpp
    [ERROR] [1671445442.740098899]: No link elements found in urdf file
    [robot_state_publisher-3] process has died [pid 8213, exit code 1, cmd /opt/ros/noetic/lib/robot_state_publisher/robot_state_publisher __name:=robot_state_publisher __log:=/home/suyu/.ros/log/41df14ac-7f87-11ed-85a2-2314f5e6c7f7/robot_state_publisher-3.log].
    ```

    - 可能性一：

      在`noetic`版本中xacro文件的宏调用是<xacro：mrobot_body/>，原老版本为<mrobot_body/>

    - 可能性二：

      [No link elements found in urdf file](https://blog.csdn.net/weixin_42314494/article/details/123989003?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522167144465416800192233542%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=167144465416800192233542&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-1-123989003-null-null.142^v68^control,201^v4^add_ask,213^v2^t3_esquery_v1&utm_term=%5BERROR%5D%20%5B1671444610.084140078%5D%3A%20No%20link%20elements%20found%20in%20urdf%20file&spm=1018.2226.3001.4187)

      ```xml
      The create.urdf.xacro file just contains a macro for instantiating a create model, but does not call it. The intended use is to include the create.urdf.xacro file in your model and then instantiate it like so:
      
      <xacro:include filename="$(find create_description)/urdf/create.urdf.xacro"/>
      <create />
      ```

35. ```shell
    [ WARN] [1671454932.937121505]: Joint state with name: "base_l_wheel_joint" was received but not found in URDF
    ```

    修改成对应名称即可，可能是ROS默认的命名方式



## 比赛

1. opencv

2. python

3. Realsense d435i

4. yolo(知道怎么调用就行)

5. 机器人学基础 正逆运动学

## C++

1. VSCode中先生成了testFunc1.c，后生成了testFunc1.h（testFunc1.c中引用了testFunc1.h），报错无源文件。

   重新写一遍testFunc1.c即可。

2. ```shell
   /home/suyu/week03/src/copy_image.cpp:1:9: warning: #pragma once in main file
    #pragma once
            ^~~~
   ```

   `.cpp`文件中不能加` #pragma once`

3. 段错误 (核心已转储)

   一旦一个程序发生了越界访问，cpu 就会产生相应的保护，于是 segmentation fault 就出现了，通过上面的解释，段错误应该就是访问了不可访问的内存，这个内存区要么是不存在的，要么是受到系统保护的，还有可能是缺少文件或者文件损坏。

   [段错误 (核心已转储)](https://gitee.com/hitcrt2022_vision_edu/lqm120-l021920/blob/master/week03/demo1_c++version.cpp)

4. ```cpp
   Warning: 'typedef' was ignored in this declaration.
   ```

   Delete `typedef`. It's the C way of declaring structs, C++ does it automatically for you.

5. <font color = SandyBrown>**不要函数名和变量名重名！！！**</font>

6. ***#include <unistd.h>***

   ubuntu里没有<Windows.h>头文件，在<unistd.h>里有usleep()

7. <font color = SandyBrown>**对于分文件编写**</font>

   - 把次级目录编写成动态库
   - 把`.cpp`文件链接到源文件下


8. `boost::this_thread‘ has not been declared； undefined reference to ‘boost::this_thread...‘`

   [Debug](https://blog.csdn.net/qq_41035283/article/details/124746050?ops_request_misc=&request_id=&biz_id=102&utm_term=%20undefined%20reference%20to%20%60boost&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-1-124746050.142^v73^control,201^v4^add_ask,239^v2^insert_chatgpt&spm=1018.2226.3001.4187)

9. `forming pointer to reference type`

   标准容器中是“按值”存放并操纵存放于其中的实例的，不允许在标准容器中存放“引用”，因为容器中很可能有`T* t`之类的底层存储指针，那么如果T被实例化为引用类型（例如 int&），则`T* t`就变成了`int&* t`（即`pointer to reference`），所以非法。与“指向引用的指针是非法的”类似，C++中同样不允许“引用数组”：

   ```cpp
   int& *p;  //pointer to reference，非法
   int& a[3]; //引用数组，非法
   ```

## CMake

1. `learncmake`根目录已经`cmake`过，在`build`中`make`时，显示无`Makefile`

   删去根目录中的除`CMakeList.txt`的其他文件

2. ```cmake
   $ cmake ..
   CMake Warning:
     Ignoring extra path from command line:
   
      ".."
   
   
   CMake Error: The source directory "/home/suyu/learncmake" does not appear to contain CMakeLists.txt.
   Specify --help for usage, or press the help button on the CMake GUI.
   
   ```

   

3. 必须有`cmake_minimum_required`

4. 引用变量`${SRC_LIST}`

5. ```cmake
   CMake Warning (dev) at CMakeLists.txt:5:
     Syntax Warning in cmake code at column 37
   
     Argument not separated from preceding token by whitespace.
   This warning is for project developers.  Use -Wno-dev to suppress it.
   
   ```

   MESSAGE(STATUS "This is BINARY dir " ${HELLO_BINARY_DIR})

   引用参数加一个空格
   
6. ```cmake
   CMake Error at CMakeLists.txt:3 (ADD_SUBDIRECTORY):
     ADD_SUBDIRECTORY given source "~/learncmake/t3/lib" which is not an
    existing directory.
   
   ```

   cmake时，想让`libhello.so`生成在`lib`目录中，而不是在`build`下再生成一个`lib`

   两种处理

   - 在主工程文件 CMakeLists.txt 中修
     改 ADD_SUBDIRECTORY(lib)指令来指定一个编译输出位置

     ```cmake
     ADD_SUBDIRECTORY(lib ~/learncmake/t3/lib)
     ```

     含义是定义了将 lib子目录加入工程,并指定编译输出(包含编译中间结果)路径为lib目录

   - 在 lib/CMakeLists.txt 中添加
     SET(LIBRARY_OUTPUT_PATH <路径>)来指定一个新的位置(**推荐**)

7. 像生成两个同名的动态库和静态库

   ```cmake
   ADD_LIBRARY(hello SHARED ${LIBHELLO_SRC})#生成libhello.so
   ADD_LIBRARY(hello_static STATIC ${LIBHELLO_SRC})#生成libhello_static.a，与libhello.so来自同一个源文件。
   SET_TARGET_PROPERTIES(hello_static PROPERTIES OUTPUT_NAME "hello")#将libhello_static.a改为libhello.a
   ```


8. ```cpp
   cpp.cpp:5:18: error: cast from ‘int*’ to ‘int’ loses precision [-fpermissive]
        cout << (int)arr << endl;
   ```

   ```cpp
   (int)arr //显示会丢失精度
   
   (long)arr//OK
   ```


9. ```shell
   warning: ‘typedef’ was ignored in this declaration
    typedef struct Mat {
    ^~~~~~~
   
   ```

   ```cpp
   //给Mat类加上别名
   typedef struct Mat {
       int width;
       int length;
       float image_point[1023][1023];
   } Mat;
   ```

10. ```cmake
    # cmake version
    CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
    
    # project name
    PROJECT(opencvcourse)
    
    # find OpenCV
    FIND_PACKAGE(OpenCV REQUIRED)
    
    # show the message of OpenCV
    MESSAGE(STATUS "OpenCV library status:")
    MESSAGE(STATUS "    version: 	${OpenCV_VERSION}")
    MESSAGE(STATUS "    headers: 	${OpenCV_INCLUDE_DIRS}")
    MESSAGE(STATUS "    libraries: 	${OpenCV_LIBS}")
    
    # link headers
    INCLUDE_DIRECTORIES({OpenCV_INCLUDE_DIRS}  ./include)
    
    #设置输出路径
    SET (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
    
    
    #SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    
    # 添加源代码文件到SRC_LIST变量中
     AUX_SOURCE_DIRECTORY(src SRC_LIST)
    
    # 生成可执行文件
    ADD_EXECUTABLE(opencv ${SRC_LIST})
    
    
    
    # after ADD_EXECUTABLE，为生成文件target添加库
    TARGET_LINK_LIBRARIES(opencv ${OpenCV_LIBS})
    
    ```

    按完整工程格式编译模板

11. 删去build中的所有文件，再打开VSCode,选择第二个(Let CMake guess what compilers and environment to use)

    ![](/home/suyu/2022-10-11 20-01-33 的屏幕截图.png)

12. ```cmake
    # cmake version
    CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
    
    # project name
    PROJECT(program)
    set(CMAKE_CXX_STANDARD 14)
    
    # find OpenCV PCL
    FIND_PACKAGE(OpenCV REQUIRED)
    find_package(PCL  REQUIRED )
    
    # show the message of OpenCV
    MESSAGE(STATUS "OpenCV library status:")
    # MESSAGE(STATUS "    version: 	${OpenCV_VERSION}")
    # MESSAGE(STATUS "    headers: 	${OpenCV_INCLUDE_DIRS}")
    # MESSAGE(STATUS "    libraries: 	${OpenCV_LIBS}")
    
    # show the message of PCL
    MESSAGE(STATUS "PCL library status:")
    # MESSAGE(STATUS "    version: 	${PCL_VERSION}")
    # MESSAGE(STATUS "    headers: 	${PCL_INCLUDE_DIRS}")
    # MESSAGE(STATUS "    libraries: 	${PCL_LIBS}")
    
    # link headers
    INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS} )
    INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS} ) 
    INCLUDE_DIRECTORIES(./include)
    
    #设置输出路径
    SET (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
    
    
    LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})   
    
    ADD_DEFINITIONS(${PCL_DEFINITIONS})  
    
    # 添加源代码文件到SRC_LIST变量中
    AUX_SOURCE_DIRECTORY(src SRC_LIST)
    
    # 生成可执行文件
    ADD_EXECUTABLE(program ${SRC_LIST})
    
    
    
    # after ADD_EXECUTABLE，为生成文件target添加库
    TARGET_LINK_LIBRARIES(program ${OpenCV_LIBS})
    TARGET_LINK_LIBRARIES(program ${PCL_LIBRARIES})
    TARGET_LINK_LIBRARIES(program pthread)
    
    
    
    
    ```

13. ```cmake
    #在test目录下
    aux_source_directory(test SRC_LIST)
    #这个指令的含义是在当前目录下，找到test目录，并将里面的所有文件一起设置成SRC_LIST
    No SOURCES given to target: demo
    [cmake] 
    
    #所以要把test改成.(当前目录)
    aux_source_directory(. SRC_LIST)
    ```

14. ` error: no matching function for call to ‘MD_1TaskNN::MD_1TaskNN(std::shared_ptr<ThreadWatcher>)’
      146 |  { ::new((void *)__p) _Up(std::forward<_Args>(__args)...); }
          |    ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    `



## OpenCV

1. ```
   命名空间 "cv" 没有成员 "imshow"
   ```

   ```cpp
   #include<opencv2/opencv.hpp>
   ```

2. ```shell
   terminate called after throwing an instance of 'cv::Exception'
     what():  OpenCV(4.5.5) /home/suyu/opencv-4.5.5/modules/core/src/array.cpp:2494: error: (-206:Bad flag (parameter or structure field)) Unrecognized or unsupported array type in function 'cvGetMat'
   
   ```
   
   每次运行完成后都会报错，因为没有更多图像可以显示

3. ```cpp
     what():  OpenCV(4.5.5) /home/suyu/opencv-4.5.5/modules/imgproc/src/color.cpp:182: error: (-215:Assertion failed) !_src.empty() in function 'cvtColor'
      
   已放弃 (核心已转储)
   ```

   `imread`未正常读取图片
   
4. Opencv中Mat数据类型定义为指向uchar的指针

```cpp
typedef Vec<uchar, 3> Vec3b;//像素的数据值由三个数组成，这三个数的数据类型为uchar
```

5. ，at方法取图像中的点的用法：

   **image.at<uchar>(i,j)**：取出灰度图像中i行j列的点。

   **image.at<Vec3b>(i,j)[k]**：取出彩色图像中i行j列第k通道的颜色点。其中uchar,Vec3b都是图像像素值的类型，不要对Vec3b这种类型感觉害怕，其实在core里它是通过typedef Vec<T,N>来定义的，N代表元素的个数，T代表类型。

<font color=sandybrown>注意是先行后列，先y坐标后先坐标</font>



6. opencv中，由于使用Mat.at访问数据时，必须正确填写相应的数据类型，因此必须弄清楚opencv中的数据类型与我们常用 
   数据类型一一对应关系。
   `Mat_<uchar>`---------CV_8U
   `Mat_<char>`-----------CV_8S
   `Mat_<short>`---------CV_16S
   `Mat_<ushort>-`-------CV_16U
   `Mat_<int>`-----------CV_32S
   `Mat_<float>`----------CV_32F

   `Mat_<double>`--------CV_64F

| C1     | C2       | C3          | C4          | C6          |             |
| :----- | :------- | :---------- | :---------- | :---------- | ----------- |
| uchar  | `uchar`  | `cv::Vec2b` | `cv::Vec3b` | `cv::Vec4b` |             |
| short  | `short`  | `cv::Vec2s` | `cv::Vec3s` | `cv::Vec4s` |             |
| int    | `int`    | `cv::Vec2i` | `cv::Vec3i` | `cv::Vec4i` |             |
| float  | `float`  | `cv::Vec2f` | `cv::Vec3f` | `cv::Vec4f` | `cv::Vec6f` |
| double | `double` | `cv::Vec2d` | `cv::Vec3d` | `cv::Vec4d` | `cv::Vec6d` |

7. OpenCV中的at函数

```cpp
srcImage.at<uchar>(x, y) //表示的是 x 行 y 列 的这个像素
srcImage.at<uchar>(Point(x, y)) //表示的是 坐标（x,y）的像素
```

at<typename>(i,j) 是opencv中图像[遍历](https://so.csdn.net/so/search?q=遍历&spm=1001.2101.3001.7020)函数，它是一个模板函数，可以取到任何类型的图像上的点。

image.at<Vec3b>(i,j)[k]：是指取出彩色图像中i行j列第k通道的颜色点。

其中Vec3b是图像像素值的类型，如果还不太明白，可以选中右键转到其定义： typedef Vec<uchar, 3> Vec3b; 是通过typedef Vec<T,N>来定义的，N代表元素的个数，T代表类型，unchar是类型，3是元素的个数。

8. 

```cpp
.at<>()返回一个值，.ptr<>()返回一个地址
```

9. `cannot declare member function ‘static GoalInfo Task::getTaskResult()’ to have static linkage `

   - 问题分析：
     出现这种情况通常.h文件中声明类的成员函数用static修饰。此时static的作用是让一个类只有一个static成员函数实例化。
     在.cpp文件中实现函数的时候，也加了static修饰符，此时static修饰符的作用是当前函数只能作用在当前的cpp文件。

   - 解决方法：
     删除.cpp文件的static修饰符。或者在.h文件中实现。
     Git

1. ```cpp
   fatal: Not possible to fast-forward, aborting.
   ```

   git pull origin master --rebase
