# Linux

## Lℹnux命令行

### awk

#### 内置变量

| 内置变量 | 说明                                                         |
| -------- | ------------------------------------------------------------ |
| FS       | 列分割符。指定每行文本的字段分隔符，默认为空格或制表位。与"-F"作用相同 |
| NF       | 当前处理的行的字段个数                                       |
| NR       | 当前处理的行的行号（序数）                                   |
| $0       | 当前处理的行的整行内容                                       |
| $n       | 当前处理行的第n个字段（第n列）                               |
| FILENAME | 被处理的文件名                                               |
| RS       | 行分隔符。awk从文件上读取资料时,将根据RS的定义把资料切割成许多条记录,而awk一次仅读入一条记录,以进行处理。预设值是’\n’ |

#### 按行输出文本

```shell
awk    ' {print}  '  123.txt #输出所有内容
awk    ' {print $0} ' 123.txt #$0代表整行
awk    ' NR==1,NR==5{print }' 123.txt#输出1-3行
awk    ' NR==1||NR==4{print}' 123.txt#输出第一和第四行
awk    ' (NR%2)==1{print }' 123.txt#输出奇数行
awk    ' /^0/{print }' 123.txt#输出以0开头的行
awk    ' BEGIN{X=0};/^1/{x++};END {print }' 123.txt#统计以1开头的行数
```

#### 按字段输出文本

```shell
-F #指定分隔符
awk -F  ":"  '{print $2}'#以冒号分割，输出第二个字段 
```

### cat

| 参数 | 含义                             |
| ---- | -------------------------------- |
| -b   | 显示文件中的行号，空行不编号     |
| -E   | 在文件的每一行行尾加上“$”字符    |
| -T   | 将文件的Tab键用字符“^I”来显示    |
| -n   | 在文件的每行前面显示行号         |
| -s   | 将连续的多个空行用一个空行来显示 |
| -v   | 显示除Tab和Enter之外的所有字符   |



###  chattr

chattr命令用于设置文件的隐藏权限，英文全称为change attributes，语法格式为“chattr [参数] 文件名称”。

如果想要把某个隐藏功能添加到文件上，则需要在命令后面追加“+参数”，如果想要把某个隐藏功能移出文件，则需要追加“-参数”。chattr命令中可供选择的隐藏权限参数非常丰富，具体如表5-8所示。

​              chattr命令中的参数及其作用

| 参数 | 作用                                                         |
| ---- | ------------------------------------------------------------ |
| i    | 无法对文件进行修改；若对目录设置了该参数，则仅能修改其中的子文件内容而不能新建或删除文件 |
| a    | 仅允许补充（追加）内容，无法覆盖/删除内容（Append Only）     |
| S    | 文件内容在变更后立即同步到硬盘（sync）                       |
| s    | 彻底从硬盘中删除，不可恢复（用0填充原文件所在硬盘区域）      |
| A    | 不再修改这个文件或目录的最后访问时间（atime）                |
| b    | 不再修改文件或目录的存取时间                                 |
| D    | 检查压缩文件中的错误                                         |
| d    | 使用dump命令备份时忽略本文件/目录                            |
| c    | 默认将文件或目录进行压缩                                     |
| u    | 当删除该文件后依然保留其在硬盘中的数据，方便日后恢复         |
| t    | 让文件系统支持尾部合并（tail-merging）                       |
| x    | 可以直接访问压缩文件中的内容                                 |

### cp

| 参数 | 含义                                                         |
| ---- | ------------------------------------------------------------ |
| -a   | 该选项通常在复制目录时使用，它保留链接、文件属性，并递归地复制目录 |
| -f   | 删除已经存在的目标文件而不提示                               |
| -i   | 交互式复制，在覆盖目标文件之前将给出提示要求用户确认         |
| -p   | 此时cp命令除复制源文件的内容外，还将把其修改时间和访问权限也复制到新文件中 |
| -r   | 若给出的源文件是目录文件，则cp将递归复制该目录下的所有子目录和文件，目标文件必须为一个目录名 |
| -l   | 不作复制，只是链接文件                                       |



### df

`df -h`查看磁盘空间

### date

`date ` `cal`

### free

`free -m/-g`查看内存

### find

[find命令详解](https://blog.csdn.net/m0_46674735/article/details/112390027?ops_request_misc=&request_id=&biz_id=102&utm_term=find&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-0-112390027.142^v33^experiment_28w_v1,185^v2^control&spm=1018.2226.3001.4187)

通常用来在特定的目录下搜索符合条件的文件，也可以用来搜索特定用户属主的文件。

```shell
find  目录  -选项  动作[-print -exec -ok ...]
```



| 参数                       | 含 义                                          |
| -------------------------- | ---------------------------------------------- |
| -name \<filename>          | 指定搜索的文件名，输出搜索结果                 |
| -iname                     | 按照文件名查找文件(忽略大小写)                 |
| -user \<username>          | 搜索指定用户搜索所属的文件                     |
| -atim \<time>              | 搜索在指定的时间内读取过的文件                 |
| -ctim \<time>              | 搜索在指定的时间内修改过的文件                 |
| -a                         | and 必须满足两个条件才显示                     |
| -o                         | or 只要满足一个条件就显示                      |
| -type                      | 根据文件类型进行搜索                           |
| -perm                      | 按照文件权限来查找文件                         |
| -fprint \<filename>        | 将匹配的文件输出到文件                         |
| -newer file1 ! newer file2 | 查找更改时间比文件file1新但比文件file2旧的文件 |

| 参数    | 含义                                                         |
| ------- | ------------------------------------------------------------ |
| -print  | 默认动作，将匹配的文件输出到标准输出                         |
| -exec   | 对匹配的文件执行该参数所给出的命令。相应命令的形式为 'command' { } \;，注意{ }和\；之间的空格。 |
| -ok     | 和-exec的作用相同，只不过以一种更为安全的模式来执行该参数所给出的命令，在执行每一个命令之前，都会给出提示，让用户来确定是否执行。 |
| -delete | 将匹配到的文件删除                                           |

```shell
find /home/ -name "*.txt" #在/home目录下查找以.txt结尾的文件名
find /home -iname "*.txt"
find /home/ -name "*.txt" -o -name "*.pdf"#查找 /home/ 下所有以.txt或.pdf结尾的文件
find /home/ -type f -name "*.txt" -fprint /tmp/re.txt #搜索/home目录下txt结尾的文件，并将输出到指定文件中(re.txt)
find /usr/local/ -maxdepth 3 -type d#向下最大深度限制为3
```

```shell
 f   #普通文件
 l   #符号连接（软连接）
 d  #目录
 b  #块设备
 s  #套接字
```

#### **根据时间戳**

- 访问时间（-atime/天，-amin/分钟）：用户最近一次访问时间。 

- 修改时间（-mtime/天，-mmin/分钟）：文件最后一次修改时间 

- 变化时间（-ctime/天，-cmin/分钟）：文件数据元（例如权限等）最后一次修改时间。

```shell
find /etc/ -type f -atime -7#搜索最近七天内被访问过的所有文件
find /etc -type f -atime +7#搜索超过七天内(7天外)被访问过的所有文件
```

#### **根据文件大小**

```cpp
   b —— 块（512字节）
    c —— 字节
    w —— 字（2字节）
    k —— 千字节
    M —— 兆字节
    G —— 吉字节
```

```shell
find /etc/ -type f -size +10k#搜索大于10KB的文件
find /etc/ -type f -size -10k#搜索小于10KB的文件
find /etc/ -type f -size   10k#搜索等于10KB的文件
find /var/log -type f -name "*.log" -size +10G -delete#搜索大于10G的日志文件，并删除
```

#### **根据文件权限**

```shell
find / -type f -perm 644#指定目录下搜索出权限为644的文件
find / -type f -name "*.txt" ! -perm 644#找出指定目录下权限不是644的txt文件
find /home/ -type f -user frank#找出/home目录用户frank拥有的所有文件
find /home/ -type f -group frank#找出/home目录用户组frank拥有的所有文件
```

**借助-exec**

找出/tmp目录下所有root的文件，并把所有权更改为用户frank

```shell
find /tmp/ -type f -user root -exec chown frank {} \;
#使用占位符{}来表示find到的文件名
```

找出家目录下所有的.sh文件并删除

```shell
[root@localhost home]# find $HOME -name "*.sh" -ok rm {} \;
< rm ... /root/install_lnmp.sh > ? y
#-ok和-exec行为一样，不过它会给出提示，是否执行相应的操作。
```

查找/home目录下所有.txt文件并把他们拼接到all.txt文件中

```shell
find /home/ -type f -name "*.txt" -exec cat {} \;>all.txt
```

查找home目录下所有.txt文件并把他们复制到/opt/backup文件中

```shell
find /home/ -type f -name "*.txt" -exec cp {} /opt/backup/ \;
```

在/var/log目录中查找更改时间在5日以前的文件并删除它们

```shell
find /var/log -type f -mtime +5 -exec rm {} \;
```



### head & tail

`head -n 文件路径`查看前n行

`tail -n 文件路径`查看后n行，`tail -f 文件路径`查看一个文件的动态变化内容

### ifconfig

`ifconfig`用于操作网卡相关的指令

### kill

### less

### ls

| 参数 | 含义                                                         |
| ---- | ------------------------------------------------------------ |
| -a   | 显示指定目录下所有子目录与文件，包括隐藏文件                 |
| -c   | 按文件的修改时间排序                                         |
| -F   | 在列出的文件名后以符号表示文件类型：目录文件后加“/”，可执行文件后加“*”，符号链接文件后加“@”，管道文件后加“ |
| h    | -以用户习惯的单位表示文件的大小，K表示千，M表示兆。通常与-l选项搭配使用 |
| -l   | 以长格式显示文件的详细信息。每行列出的信息依次是：文件类型与权限、链接数、文件属主、文件属组、文件大小、文件建立或修改的时间、文件名。对于符号链接文件，显示的文件名后有“—>”和引用文件路径名；对于设备文件，其“文件大小”字段显示主、次设备号，而不是文件大小。目录中总块数显示在长格式列表的开头，其中包含间接块 |
| -r   | 从后向前地列举目录中的内容                                   |
| -s   | 按文件大小排序                                               |
| -t   | 按文件建立的时间排序，越新修改的越排在前面                   |
| -u   | 按文件上次存取时间排序                                       |



### man

`man 命令`查询命令，退出按q

### more

和cat命令类似，more可将文件内容显示在屏幕上，

1.每次只显示一页，

2.按下空格键可以显示下一页，

3.按下q键退出显示. 文件中搜索指定的字符串。

其格式如下：

### ps

- `ps -ef `查看服务器的信息
- -e：等价于“-A”，表示列出全部的进程
- -f：显示全部的列（显示全字段）
- `ps -ef  |grep 进程名称`在ps的结果中过滤出想要查看的进程状态

`kill 进程的PID`配合`ps -ef`使用

### rm

| 参数 | 含 义                                |
| ---- | ------------------------------------ |
| -i   | 以进行交互式方式执行                 |
| -f   | 强制删除，忽略不存在的文件，无需提示 |
| -r   | 递归地删除目录下的内容               |



### service

`service 服务名 start/stop/restart`

### top

进入：top      推出：q

| 名称        | 含义                                                       |
| ----------- | ---------------------------------------------------------- |
| **PID**     | 进程id                                                     |
| **USER**    | 该进程对应的用户                                           |
| **PR**      | 优先级                                                     |
| **VIRT**    | 虚拟内存；                                                 |
| **RES**     | 常驻内存；                                                 |
| **SHR**     | 共享内存；                                                 |
| **S**       | 表示进程的状态status（sleeping，其中S表示睡眠，R表示运行） |
| **%CPU**    | 表示CPU的占用百分比                                        |
| **%MEM**    | 表示内存的占用百分比                                       |
| **TIME+**   | 执行的时间                                                 |
| **COMMAND** | 进程的名称或者路径                                         |

在运行**top**的时候，可以按下方便的快捷键：

M：表示将结果按照内存（MEM）从高到低进行降序排列；

P：表示将结果按照CPU使用率从高到低进行降序排列；

*当服务器拥有多个cpu的时候可以使用“1”快捷键来切换是否展示显示各个cpu的详细信息*



### tree

以[树形结构](https://so.csdn.net/so/search?q=树形结构&spm=1001.2101.3001.7020)列出指定目录下的所有内容，包括所有文件、子目录及子目录里的目录和文件。

```shell
-a                            #显示所有文件
-d                           #只显示目录
-f                            #显示每个文件的全路径
-i                            #不显示树枝
-L level                #遍历目录的最大层数             
-F                           #在不同文件类型的结尾，各自加上不同符号
-I                           #使用正则匹配不想看到的文件夹
```

| 文件类型 | 符号 |
| -------- | ---- |
| 执行文件 | *    |
| 目录     | \    |
| Socket   | =    |
| 符号连接 | @    |
| 管道名称 | \|   |

```shell
tree -L 1 -F /boot/|grep /$      #<==过滤以斜线结尾的所有内容 
#$在Linux正则表达式中标识以什么什么结尾，^表示以什么什么开头，^$表示空行

  tree -L 2 > /home/luke/tree.txt  #把目录结构信息保存到文本中
```

### wc

`wc -lwd 需要统计的文件路径`

- -l lines
- -w words 
- -c bytes   



### |

管道符 |   作用：管道一般可以用于“**过滤**”，“特殊”，“扩展处理”。

语法：管道不能单独使用，必须需要配合前面所讲的一些指令来一起使用，其作用**主要是辅助作用**。

- 过滤：需要通过管道查询出根目录下包含“y”字母的文档名称。

  ```shell 
  ls / | grep y  #①以管道作为分界线，前面的命令有个输出，后面需要先输入，然后再过滤，最后再输出，通俗的讲就是管道前面的输出就是后面指令的输入；②grep指令：主要用于过滤
  ```

- cat 路径 | less 路径

### --prefix

源码的安装一般由3个步骤组成：配置(configure)、编译(make)、安装( make install ).

Configure是一个可执行的脚本，它有很多选项，在待安装的源码路径下使用命令./configure--help 输出详细的选项列表。其中–prefix选项是配置安装的路径.

如果不配置该选项，安装后可执行文件默认放在/usr/local/bin中，库文件默认放在/usr/local/lib中，配置文件默认放在/usr/local/etc中，其它的资源文件放在/usr/local/share中，比较凌乱。

如果配置–prefix，如：./configure --prefix=/usr/lcoal/test

可以把所有的资源文件放在/user/local/test路径中，不会杂乱。

### ./configure

### 快捷键

`ctl +L`清屏

## Linux系统基础

### [用户身份与文件权限](https://www.linuxprobe.com/basic-learning-05.html)

### Linux系统中常见的目录名称以及相应内容

| 目录名称       | 应放置文件的内容                                             |
| -------------- | ------------------------------------------------------------ |
| /boot          | 开机所需文件—内核、开机菜单以及所需配置文件等                |
| /dev           | 以文件形式存放任何设备与接口                                 |
| **/etc**       | **配置文件**                                                 |
| /home          | 用户主目录                                                   |
| /bin           | 存放单用户模式下还可以操作的[命令](https://www.linuxcool.com/) |
| /lib           | 开机时用到的函数库，以及/bin与/sbin下面的命令要调用的函数    |
| /sbin          | 开机过程中需要的命令                                         |
| /media         | 用于挂载设备文件的目录                                       |
| **/opt**       | **放置第三方的软件**                                         |
| /root          | 系统管理员的家目录                                           |
| /srv           | 一些网络服务的数据文件目录                                   |
| /tmp           | 任何人均可使用的“共享”临时目录                               |
| /proc          | 虚拟文件系统，例如系统内核、进程、外部设备及网络状态等       |
| **/usr/local** | **用户自行安装的软件**                                       |
| /usr/sbin      | Linux系统开机时不会使用到的软件/命令/[脚本](https://www.linuxcool.com/) |
| /usr/share     | 帮助与说明文件，也可放置共享文件                             |
| **/var**       | **主要存放经常变化的文件，如日志**                           |
| /lost+found    | 当文件系统发生错误时，将一些丢失的文件片段存放在这里         |

### GDB调试

#### 启动GDB

```shell
gdb ./filename #elf程序名称	
```

#### 察看源码

```shell
l							  #程序未启动，从头开始显示
list                        #默认显示断点的上下四行
l -5
```

#### 运行程序

```shell
r							 #运行程序
```

#### 设置断点

```shell
b main				  #打断点  b+函数名
b mian.cpp:36
```

#### 查看断点

```bash
info breakpoints
delet  #删除所有断点
delet  #删除第二个断点
```

#### 单步执行

```bash
n  #不进入函数  next
recode #打标记
s#单步源码进行，遇见函数进入函数 step 
```

#### 查看变量

```shell
p
```

#### 退出

```shell
q
```



gdb



### 正常调试

1. 取指定位置的堆栈操作

```Bash
(gdb) x /200aw $sp//sp指针指向的位置
```

1. 单步执行step(进入函数内，源码层面)/stepi(机器指令层面)

```Bash
(gdb) s
```

1. 单步执行(不进入函数内)

```Bash
(gdb) n
```

1. 设置动态库相对搜索路径

```Bash
(gdb) set solib-search-path + path
```

1. 设置动态库绝对搜索路径

```Bash
(gdb) set solib-absolute-prefix + path
```

1. 继续执行到下一个断点(恢复所有线程的执行)

```Bash
(gdb) c
```

1. 选择进入对应的栈

```Bash
f + num
```

1. 连接上对应的进程

```Bash
gdb -p pid
```

1. 查看所有线程的堆栈

```Bash
thread apply all bt
```

1. 多线程调试，启停其他线程

```Bash
set scheduler-locking [ off| on | step]
#off 不锁定任何线程， on表示只有当前被调试的线程继续执行, step 表示单步执行的时候，只有当前线程执行
```

1. 设置命令行参数

```Bash
set args 命令号参数
```

1. 按照字符的形式打印sp附近的指定字节

```Bash
x /200aw $sp
```

1. gdb调试守护进程，设置了如下两个参数之后在gdb环境下可以在父进程退出之后跟随子进程

```Bash
set follow-fork-mode child
set unwindonsignal on
```

follow-fork-mode有两个参数可以使用，parent 是在调用fork之后继续跟踪父进程，child是在执行fork之后继续跟踪子进程

> Set debugger response to a program call of fork or vfork.

> A fork or vfork creates a new process.  follow-fork-mode can be:

>   parent  - the original process is debugged after a fork

>   child   - the new process is debugged after a fork

> The unfollowed process will continue to run.

> By default, the debugger will follow the parent process.

unwindonsignal 当gdb调用函数的时候，接收到信号的处理方式

> Set unwinding of stack if a signal is received while in a call dummy.

> The unwindonsignal lets the user determine what gdb should do if a signal

> is received while in a function called from gdb (call dummy).  If set, gdb

> unwinds the stack and restore the context to what as it was before the call.

> The default is to stop in the frame where the signal was received.

1. thread命令

```Bash
thread name   [name]#为当前线程设置命名
thread applay [command] # 为当前线程应用对应的命令
thread find # 查找符合描述的线程，查找条件一般按照线程命名查找，Will display thread ids whose name, target ID, or extra info matches REGEXP
```

1. 查看本地变量命令

```Bash
i locals 
```

1. 使用watch监控变量

```Bash
watch variable if variable == 66
```

1. 运行完一个函数，跳到函数调用的地方finish

该命令还可以用于跳出循环使用

```Bash
finish
```

1. 跳出函数，指定函数返回值return

```Bash
return 119
```

1. 跳出循环finish 或者until

```Bash
finish # 跳出当前循环(跳出当前栈)
util num # 运行到指定行结束运行
```

1. 打印指向子类对象的父类指针

打印父类指针的时候经常使用p *lpWidget，因为父类指针只能按照父类的信息提取对象的内容，父类一般都是虚基类，因此打印出来的信息可能就一张虚函数表。这个时候要想打印全部的信息，需要在代码中找到子类的定义，在打印信息中将指针强制转换成子类类型指针之后再进行取值打印，一般一般如下：

```Bash
p *(Derive *)lpWidget
```

1. 当gdb打印的字符补全时

当使用gdb调试程序打印的字符不全时，可以可以通过设置打印元素限制为0来放开限制

```Bash
set print element 0
```

1. gdb打印含有0的数组或者字符数组

```Bash
# 使用gdb打印m开始n个字符(即使中间遇到'\o'(0))也不会停止
p arr[m]@n
```

#### 反向调试

GDb7.0以上的平台开始支持反向调试

反向调试需要开启记录，调试结束关闭记录，只有在开启记录之后才能完全正常的进行反向调试。

1. 开启记录关闭记录

```Bash
# 开启记录
record
# 关闭记录
record stop
```

1. 向上走一步`reverse-step/reverse-stepi`进入函数内部

```Bash
reverse-step/reverse-stepi
```

1. 向上走一步源码层面reverse-next/reverse-nexti

```Bash
reverse-next/reverse-nexti
```

1. 反向运行到调用当前函数的地方

```Bash
reverse-finish
```

1. 设置程序运行方向，能够像正常调试方式一样反向调试

```Bash
set exec-direction [forward | reverse]
```













## gdb执行需赋权的程序



程序执行的信息受到其父进程影响，在GDB中真正启动调试进程的是/bin/bash 进程，gdb只是调试进程的grand father，因此如果想通过gdb给调试的进程赋权，需要将gdb，bash和对应的进程都进行赋权

```Bash
sudo setcap cap_net_raw,cap_net_admin=eip ./a.out
sudo setcap cap_net_raw,cap_net_admin=eip /bin/bash
sudo setcap cap_net_raw,cap_net_admin=eip /bin/gdb
```

或者关闭gdb启动过程的shell子进程调用过程

```Bash

```



[gdb appears to ignore executable capabilities](https://stackoverflow.com/questions/4357057/gdb-appears-to-ignore-executable-capabilities)





































