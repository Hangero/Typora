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

##   ROS

1. RLExceplition:[xx.launch] is neither a ...

   需要设置环境变量。

   ```shell
   gedit ~/.bashrc 
   source ~/catkin_ws/devel/setup.bash
   ```

2. Could not find the GUI 

   升级`joint_state_publish_gui`把launch文件中的`joint_state_publisher`修改为`joint_state_publish_gui`。

3. 一定要在相关目录下运行`check_urdf`才能正确的检查urdf文件。

4. 编译空间时，确保工作变量可以找到功能包。可以用`rospack`去寻找。[rospack+roscd+rosls](https://blog.csdn.net/qq_42910179/article/details/106868560?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165555531216782184620676%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165555531216782184620676&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-106868560-null-null.142^v17^pc_rank_34,157^v15^new_3&utm_term=rospack&spm=1018.2226.3001.4187) 

```shell
$ rospack find [pack_name]
```

5. Invoking "cmake" failed 

6. ROSModel报错 

   改`Frame`为`base_link`

7. Invoking "make cmake_check_build_system" failed

   不同功能包里不有重名的节点。

8. Make sure file exists in package path and permission is set to excutable (chmod + x)

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

12. Error  reading  end  tag 

    检查标签是否成对

13. Couldn't  save  project  < invalid  path >

    设置路径

14. 注意`oarm6.xacro`本身就是宏，引用时要加上`</xacro:macro`

15. maximum recursion depth exceeded

16. - Resource not found: The following package was not found in <arg default="$(find oarm6_moveit_config)/default_warehouse_mongo_db" name="db_path"/>: oarm6_moveit_config
      ROS path [0]=/opt/ros/melodic/share/ros
      ROS path [1]=/opt/ros/melodic/share

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

      记得要在`package.xml`和`CMakeList.txt`中同步修改名称

      [更改ROS功能包的名称](https://www.guyuehome.com/35250)

17. Invalid <arg> tag: moveit_config
    ROS path [0]=/opt/ros/indigo/share/ros
    ROS path [1]=/home/spark/my_ws/src
    ROS path [2]=/home/spark/catkin_ws/src
    ROS path [3]=/opt/ros/indigo/share
    ROS path [4]=/opt/ros/indigo/stacks. 

    Arg xml is <arg default="$(find moveit_config)/default_warehouse_mongo_db" name="db_path"/>
    The traceback for the exception was written to the log file
    moveit_config是一个类似元功能包，不能递归的编译，需要在src 根目录下编译

18. <arg default="$(find oarm6_moveit_config)/default_warehouse_mongo_db" name="db_path"/>: oarm6_moveit_config

    `$(find oarm6_moveit_config)`是该包的路径，从/home开始写（不加$）就算绝对路径。

19. `env | grep ros`

    查看和ros相关的环境变量，关注 ROS_PACKAGE_PATH的日志信息。

20. - [ROS创建工作空间、功能包，编译示例程序](https://blog.csdn.net/qq_41667348/article/details/113484488?ops_request_misc=&request_id=&biz_id=102&utm_term=env%7Cgrep%20ros&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-0-113484488.142^v20^huaweicloudv1,157^v15^new_3&spm=1018.2226.3001.4187)
    - [关于ROS中找不到工作空间的功能包解决](https://blog.csdn.net/ypk138/article/details/120523830?spm=1001.2101.3001.6661.1&utm_medium=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-1-120523830-blog-123138693.pc_relevant_default&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-1-120523830-blog-123138693.pc_relevant_default&utm_relevant_index=1)

21. RLException: error loading tag:
    file does not exist [3]
    XML is <rosparam command="load" file="3" ns="manipulator"/ >

    报错意思是没有kinematics.yaml文件，因此在设置组的时候填上3是不对的，点击后面的浏览文件按钮，选择在solidworks插件生成的config文件`（oarm6_descrption/config）`，而不是填3。古月书里moveit setup assistant助手设置 planning groups中，设置的是`Kin.Solver Attempts`,而现在是`Kin.parameters file`。

22. ROS目录用途解释

    [Linux各目录及每个目录的详细介绍](https://blog.csdn.net/xiao_yi_xiao/article/details/120492997?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165590571816782388062866%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165590571816782388062866&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-120492997-null-null.142^v20^huaweicloudv1,157^v15^new_3&utm_term=linux%E4%B8%AD%E7%9B%AE%E5%BD%95&spm=1018.2226.3001.4187)

    [Linux中的目录详解](https://blog.csdn.net/z17788055263/article/details/106629288?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165590571816782388062866%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165590571816782388062866&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-106629288-null-null.142^v20^huaweicloudv1,157^v15^new_3&utm_term=linux%E4%B8%AD%E7%9B%AE%E5%BD%95&spm=1018.2226.3001.4187)

23. [Moveit!碰撞检测添加模型](https://blog.csdn.net/anyingdaozhimi/article/details/109253898?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165590702916780366590112%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165590702916780366590112&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-3-109253898-null-null.142^v20^huaweicloudv1,157^v15^new_3&utm_term=moveit%E7%A2%B0%E6%92%9E%E6%A3%80%E6%B5%8B&spm=1018.2226.3001.4187)

24. RLException: Invalid roslaunch XML syntax: no element found: line 42, column 0 The traceback for the exception was written to the log file

    检查标签是非完整对称。

    ```xml
    <launch>
    ...
    </launch>
    ```

25. RLException: error loading <rosparam> tag: 
    	file does not exist [/home/suyu/catkin_ws/src/oarm6_description/config/oarm.yaml]
    XML is <rosparam command="load" file="$(find oarm6_description)/config/oarm.yaml"/>

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

## 比赛

1. opencv

2. python

3. Realsense d435i

4. yolo(知道怎么调用就行)

5. 机器人学基础 正逆运动学

## C++

1. VSCode中先生成了testFunc1.c，后生成了testFunc1.h（testFunc1.c中引用了testFunc1.h），报错无源文件。

   重新写一遍testFunc1.c即可。

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

   

