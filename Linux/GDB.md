# GDB

[GDB调试-从入门实践到原理](https://mp.weixin.qq.com/s/XxPIfrQ3E0GR88UsmQNggg)

GDB是一个由GNU开源组织发布的、UNIX/LINUX操作系统下的、**「基于命令行的、功能强大的程序调试工具」**。

> 在Linux.md下也有简单教程

## 断点

断点是我们在调试中经常用的一个功能，我们在指定位置设置断点之后，程序运行到该位置将会`暂停`，这个时候我们就可以对程序进行更多的操作，比如查看`变量内容，堆栈情况`等等，以帮助我们调试程序。

以设置断点的命令分为以下几类：

- breakpoint
- watchpoint
- catchpoint

### breakpoint

可以根据行号、函数、条件生成断点，下面是相关命令以及对应的作用说明：

| 命令                      | 作用                                       |
| :------------------------ | :----------------------------------------- |
| **break [file]:function** | **在文件file的function函数入口设置断点**   |
| **break [file]:line**     | **在文件file的第line行设置断点**           |
| info breakpoints          | 查看断点列表                               |
| **break [+-]offset**      | **在当前位置偏移量为[+-]offset处设置断点** |
| break *addr               | 在地址addr处设置断点                       |
| break ... if expr         | 设置条件断点，仅仅在条件满足时             |
| ignore n count            | 接下来对于编号为n的断点忽略count次         |
| clear                     | 删除所有断点                               |
| clear function            | 删除所有位于function内的断点               |
| delete n                  | 删除指定编号的断点                         |
| enable n                  | 启用指定编号的断点                         |
| disable n                 | 禁用指定编号的断点                         |
| save breakpoints file     | 保存断点信息到指定文件                     |
| source file               | 导入文件中保存的断点信息                   |
| break                     | 在下一个指令处设置断点                     |
| clear [file:]line         | 删除第line行的断点                         |



### watchpoint 

watchpoint是一种特殊类型的断点，类似于正常断点，是要求GDB暂停程序执行的命令。区别在于watchpoint`没有驻留`某一行源代码中，而是指示GDB每当某个表达式改变了值就`暂停执行`的命令。

watchpoint分为`硬件实现和软件实现`两种。前者需要硬件系统的支持；后者的原理就是每步执行后都检查变量的值是否改变。GDB在新建数据断点时会优先尝试硬件方式，如果失败再尝试软件实现。

| 命令                         | 作用                         |
| :--------------------------- | :--------------------------- |
| watch variable               | 设置变量数据断点             |
| watch var1 + var2            | 设置表达式数据断点           |
| rwatch variable              | 设置读断点，仅支持硬件实现   |
| awatch variable              | 设置读写断点，仅支持硬件实现 |
| info watchpoints             | 查看数据断点列表             |
| set can-use-hw-watchpoints 0 | 强制基于软件方式实现         |

使用数据断点时，需要注意：

- 当监控变量为局部变量时，一旦局部变量失效，数据断点也会失效
- **如果监控的是指针变量`p`，则`watch *p`监控的是`p`所指内存数据的变化情况，而`watch p`监控的是`p`指针本身有没有改变指向**

最常见的数据断点应用场景：**「定位堆上的结构体内部成员何时被修改」**。由于指针一般为局部变量，为了解决断点失效，一般有两种方法。

| 命令                    | 作用                                 |
| :---------------------- | :----------------------------------- |
| print &variable         | 查看变量的内存地址                   |
| watch *(type *)address  | 通过内存地址间接设置断点             |
| watch -l variable       | 指定location参数                     |
| watch variable thread 1 | 仅编号为1的线程修改变量var值时会中断 |

最常见的数据断点应用场景：**「定位堆上的结构体内部成员何时被修改」**。由于指针一般为局部变量，为了解决断点失效，一般有两种方法。

| 命令                    | 作用                                 |
| :---------------------- | :----------------------------------- |
| print &variable         | 查看变量的内存地址                   |
| watch *(type *)address  | 通过内存地址间接设置断点             |
| watch -l variable       | 指定location参数                     |
| watch variable thread 1 | 仅编号为1的线程修改变量var值时会中断 |



### catchpoint

从字面意思理解，是捕获断点，其主要监测信号的产生。例如c++的throw，或者加载库的时候，产生断点行为。

| 命令                 | 含义                                 |
| -------------------- | ------------------------------------ |
| catch fork           | 程序调用fork时中断                   |
| tcatch fork          | 设置的断点只触发一次，之后被自动删除 |
| catch syscall ptrace | 为ptrace系统调用设置断点             |



## 命令行

| 命令             | 作用                        |
| :--------------- | :-------------------------- |
| run arglist      | 以arglist为参数列表运行程序 |
| set args arglist | 指定启动命令行参数          |
| set args         | 指定空的参数列表            |
| show args        | 打印命令行列表              |



## 程序栈

| 命令              | 作用                                      |
| :---------------- | :---------------------------------------- |
| backtrace [n]     | 打印栈帧                                  |
| frame [n]         | 选择第n个栈帧，如果不存在，则打印当前栈帧 |
| up n              | 选择当前栈帧编号+n的栈帧                  |
| down n            | 选择当前栈帧编号-n的栈帧                  |
| info frame [addr] | 描述当前选择的栈帧                        |
| info args         | 当前栈帧的参数列表                        |
| info locals       | 当前栈帧的局部变量                        |



## 多进程、多线程

### 多进程

GDB在调试多进程程序（程序含`fork`调用）时，默认只追踪父进程。可以通过命令设置，实现只追踪父进程或子进程，或者同时调试父进程和子进程。





### 多线程

多线程开发在日常开发工作中很常见，所以多线程的调试技巧非常有必要掌握。

默认调试多线程时，一旦程序中断，所有线程都将暂停。如果此时再继续执行当前线程，其他线程也会同时执行。

| 命令                       | 作用                                                         |
| :------------------------- | :----------------------------------------------------------- |
| info threads               | 查看线程列表                                                 |
| print $_thread             | 显示当前正在调试的线程编号                                   |
| set scheduler-locking on   | 调试一个线程时，其他线程暂停执行                             |
| set scheduler-locking off  | 调试一个线程时，其他线程同步执行                             |
| set scheduler-locking step | 仅用step调试线程时其他线程不执行，用其他命令如next调试时仍执行 |

如果只关心当前线程，建议临时设置 `scheduler-locking` 为 `on`，避免其他线程同时运行，导致命中其他断点分散注意力。

## 打印输出

通常情况下，在调试的过程中，我们需要查看某个变量的值，以分析其是否符合预期，这个时候就需要打印输出变量值。

| 命令               | 作用                                 |
| ------------------ | ------------------------------------ |
| whatis variable    | 查看变量的类型                       |
| ptype variable     | 查看变量详细的类型信息               |
| info variables var | 查看定义该变量的文件，不支持局部变量 |

### 打印字符串

使用`x/s`命令打印`ASCII`字符串，如果是宽字符字符串，需要先看宽字符的长度 `print sizeof(str)`。

如果长度为`2`，则使用`x/hs`打印；如果长度为`4`，则使用`x/ws`打印

| 命令                    | 作用                                   |
| ----------------------- | -------------------------------------- |
| x/s str                 | 打印字符串                             |
| set print elements 0    | 打印不限制字符串长度/或不限制数组长度  |
| call printf("%s\n",xxx) | 这时打印出的字符串不会含有多余的转义符 |
| printf "%s\n",xxx       | 同上                                   |

### 打印数组

| 命令                       | 作用                                                 |
| :------------------------- | :--------------------------------------------------- |
| print *array@10            | 打印从数组开头连续10个元素的值                       |
| print array[60]@10         | 打印array数组下标从60开始的10个元素，即第60~69个元素 |
| set print array-indexes on | 打印数组元素时，同时打印数组的下标                   |

### 打印指针

| 命令                     | 作用                           |
| :----------------------- | :----------------------------- |
| print ptr                | 查看该指针指向的类型及指针地址 |
| print *(struct xxx *)ptr | 查看指向的结构体的内容         |



## 函数跳转

| 命令                     | 作用                                                      |
| :----------------------- | :-------------------------------------------------------- |
| set step-mode on         | 不跳过不含调试信息的函数，可以显示和调试汇编代码          |
| finish                   | 执行完当前函数并打印返回值，然后触发中断                  |
| return 0                 | 不再执行后面的指令，直接返回，可以指定返回值              |
| call printf("%s\n", str) | 调用printf函数，打印字符串(可以使用call或者print调用函数) |
| print func()             | 调用func函数(可以使用call或者print调用函数)               |
| set var variable=xxx     | 设置变量variable的值为xxx                                 |
| set {type}address = xxx  | 给存储地址为address，类型为type的变量赋值                 |
| info frame               | 显示函数堆栈的信息（堆栈帧地址、指令寄存器的值等）        |



## 其它

### 图形化

tui为`terminal user interface`的缩写，在启动时候指定`-tui`参数，或者调试时使用`ctrl+x+a`组合键，可进入或退出图形化界面。

| 命令              | 含义                        |
| ----------------- | --------------------------- |
| layout src        | 显示源码窗口                |
| layout asm        | 显示汇编窗口                |
| layout split      | 显示源码 + 汇编窗口         |
| layout regs       | 显示寄存器 + 源码或汇编窗口 |
| winheight src +5  | 源码窗口高度增加5行         |
| winheight asm -5  | 汇编窗口高度减小5行         |
| winheight cmd +5  | 控制台窗口高度增加5行       |
| winheight regs -5 | 寄存器窗口高度减小5行       |



## 启动方式

使用gdb调试，一般有以下几种启动方式：

- gdb filename: 调试可执行程序
- gdb attach pid: 通过”绑定“进程ID来调试正在运行的进程
- gdb filename -c coredump_file: 调试可执行文件



## 调试

### 可执行文件

#### 单线程

```cpp
#include<stdio.h>

void print(int xx, int *xxptr) {
  printf("In print():\n");
  printf("   xx is %d and is stored at %p.\n", xx, &xx);
  printf("   ptr points to %p which holds %d.\n", xxptr, *xxptr);
}

int main(void) {
  int x = 10;
  int *ptr = &x;
  printf("In main():\n");
  printf("   x is %d and is stored at %p.\n", x, &x);
  printf("   ptr points to %p which holds %d.\n", ptr, *ptr);
  print(x, ptr);
  return 0;
}
```



```Cpp
(gdb) b 15
Breakpoint 1 at 0x400601: file test_main.cc, line 15.
(gdb) info b
Num     Type           Disp Enb Address            What
1       breakpoint     keep y   0x0000000000400601 in main() at test_main.cc:15
(gdb) r
Starting program: /root/./test_main
In main():
   x is 10 and is stored at 0x7fffffffe424.
   ptr points to 0x7fffffffe424 which holds 10.

Breakpoint 1, main () at test_main.cc:15
15   print(xx, xxptr);
Missing separate debuginfos, use: debuginfo-install glibc-2.17-260.el7.x86_64
(gdb) backtrace	//backtrace命令是列出当前堆栈中的所有帧。例子中，栈上只有一帧，编号为0，属于main函数。
#0  main () at test_main.cc:15 
(gdb) step //执行了step命令，即进入函数内。
print (xx=10, xxptr=0x7fffffffe424) at test_main.cc:4
4   printf("In print():\n");
(gdb) backtrace	//有两个栈帧，第1帧属于main函数，第0帧属于print函数。
#0  print (xx=10, xxptr=0x7fffffffe424) at test_main.cc:4
#1  0x0000000000400612 in main () at test_main.cc:15
```

##### frame 

栈帧用来存储函数的变量值等信息，默认情况下，GDB总是位于当前正在执行函数对应栈帧的上下文中。

在前面的例子中，由于当前正在print()函数中执行，GDB位于第0帧的上下文中。**可以通过frame命令来获取当前正在执行的上下文所在的帧**。

```cpp
(gdb) frame
#0  print (xx=10, xxptr=0x7fffffffe424) at test_main.cc:4
4   printf("In print():\n");
(gdb) print xx
$1 = 10	//尝试使用print命令打印下当前栈帧的值
(gdb) print xxptr
$2 = (int *) 0x7fffffffe424
//如果我们想看其他栈帧的内容呢？比如main函数中x和ptr的信息呢？假如直接打印这俩值的话，那么就会得到如下：
(gdb) print x
No symbol "x" in current context.
(gdb) print xxptr
No symbol "ptr" in current context.

(gdb) frame 1  //可以通过_frame num_来切换栈帧
#1  0x0000000000400612 in main () at test_main.cc:15
15   print(x, ptr);
(gdb) print x
$3 = 10
(gdb) print ptr
$4 = (int *) 0x7fffffffe424
(gdb)
```

#### 多线程

```cpp
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

int fun_int(int n) {
  std::this_thread::sleep_for(std::chrono::seconds(10));
  std::cout << "in fun_int n = " << n << std::endl;
  
  return 0;
}

int fun_string(const std::string &s) {
  std::this_thread::sleep_for(std::chrono::seconds(10));
  std::cout << "in fun_string s = " << s << std::endl;
  
  return 0;
}

int main() {
  std::vector<int> v;
  v.emplace_back(1);
  v.emplace_back(2);
  v.emplace_back(3);

  std::cout << v.size() << std::endl;

  std::thread t1(fun_int, 1);
  std::thread t2(fun_string, "test");

  std::cout << "after thread create" << std::endl;
  t1.join();
  t2.join();
  return 0;
}
```



```cpp
(gdb) b 27 //在第27行加上断点
Breakpoint 1 at 0x4013d5: file test.cc, line 27.
(gdb) b test.cc:32 //在第32行加上断点(效果与b 32一致)
Breakpoint 2 at 0x40142d: file test.cc, line 32.
(gdb) info b  //输出所有的断点信息
Num     Type           Disp Enb Address            What
1       breakpoint     keep y   0x00000000004013d5 in main() at test.cc:27
2       breakpoint     keep y   0x000000000040142d in main() at test.cc:32
(gdb) r	//程序开始运行，并在第一个断点处暂停
Starting program: /root/test
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/lib64/libthread_db.so.1".

Breakpoint 1, main () at test.cc:27
(gdb) c	//执行c命令，在第二个断点处暂停，在第一个断点和第二个断点之间，创建了两个线程t1和t2
Continuing.
3
[New Thread 0x7ffff6fd2700 (LWP 44996)]
in fun_int n = 1
[New Thread 0x7ffff67d1700 (LWP 44997)]

Breakpoint 2, main () at test.cc:32
32   std::cout << "after thread create" << std::endl;
(gdb) info threads	//输出所有的线程信息，从输出上可以看出，总共有3个线程，分别为main线程、t1和t2
  Id   Target Id         Frame
  3    Thread 0x7ffff67d1700 (LWP 44997) "test" 0x00007ffff7051fc3 in new_heap () from /lib64/libc.so.6
  2    Thread 0x7ffff6fd2700 (LWP 44996) "test" 0x00007ffff7097e2d in nanosleep () from /lib64/libc.so.6
* 1    Thread 0x7ffff7fe7740 (LWP 44987) "test" main () at test.cc:32
(gdb) thread 2	//切换至线程2
[Switching to thread 2 (Thread 0x7ffff6fd2700 (LWP 44996))]
#0  0x00007ffff7097e2d in nanosleep () from /lib64/libc.so.6
(gdb) bt	//输出线程2的堆栈信息
#0  0x00007ffff7097e2d in nanosleep () from /lib64/libc.so.6
#1  0x00007ffff7097cc4 in sleep () from /lib64/libc.so.6
#2  0x00007ffff796ceb9 in std::this_thread::__sleep_for(std::chrono::duration<long, std::ratio<1l, 1l> >, std::chrono::duration<long, std::ratio<1l, 1000000000l> >) () from /lib64/libstdc++.so.6
#3  0x00000000004018cc in std::this_thread::sleep_for<long, std::ratio<1l, 1l> > (__rtime=...) at /usr/include/c++/4.8.2/thread:281
#4  0x0000000000401307 in fun_int (n=1) at test.cc:9
#5  0x0000000000404696 in std::_Bind_simple<int (*(int))(int)>::_M_invoke<0ul>(std::_Index_tuple<0ul>) (this=0x609080)
    at /usr/include/c++/4.8.2/functional:1732
#6  0x000000000040443d in std::_Bind_simple<int (*(int))(int)>::operator()() (this=0x609080) at /usr/include/c++/4.8.2/functional:1720
#7  0x000000000040436e in std::thread::_Impl<std::_Bind_simple<int (*(int))(int)> >::_M_run() (this=0x609068) at /usr/include/c++/4.8.2/thread:115
#8  0x00007ffff796d070 in ?? () from /lib64/libstdc++.so.6
#9  0x00007ffff7bc6dd5 in start_thread () from /lib64/libpthread.so.0
#10 0x00007ffff70d0ead in clone () from /lib64/libc.so.6
(gdb) c	//直至程序结束
Continuing.
after thread create
in fun_int n = 1
[Thread 0x7ffff6fd2700 (LWP 45234) exited]
in fun_string s = test
[Thread 0x7ffff67d1700 (LWP 45235) exited]
[Inferior 1 (process 45230) exited normally]
(gdb) q
```

#### 多进程

### coredump

当我们开发或者使用一个程序时候，最怕的莫过于程序莫名其妙崩溃。为了分析崩溃产生的原因，操作系统的内存内容（包括程序崩溃时候的堆栈等信息）会在程序崩溃的时候dump出来（默认情况下，这个文件名为core.pid，其中pid为进程id），这个dump操作叫做coredump(核心转储)，然后我们可以用调试器调试此文件，以还原程序崩溃时候的场景

在系统默认情况下，coredump生成是关闭的，所以需要设置对应的选项以打开coredump生成。

[Ubuntu C++项目coredump的快速定位](https://www.bilibili.com/video/BV1kK411S7MB/?spm_id_from=333.337.search-card.all.click&vd_source=c5bac967ea2f91153247432d9c1e767d)



# GDB常用操作 





# 其他命令行工具

## pstack

此命令可显示每个进程的栈跟踪。pstack 命令必须由相应进程的属主或 root 运行。可以使用 pstack 来确定进程挂起的位置。此命令允许使用的唯一选项是要检查的进程的 PID。

这个命令在排查进程问题时非常有用，比如我们发现一个服务一直处于work状态（如假死状态，好似死循环），使用这个命令就能轻松定位问题所在；可以在一段时间内，多执行几次pstack，若发现代码栈总是停在同一个位置，那个位置就需要重点关注，很可能就是出问题的地方；

以前面的多线程代码为例，其进程ID是4507(在笔者本地)，那么通过

pstack 4507输出结果如下：

```bash
Thread 3 (Thread 0x7f07aaa69700 (LWP 45708)):
#0  0x00007f07aab2ee2d in nanosleep () from /lib64/libc.so.6
#1  0x00007f07aab2ecc4 in sleep () from /lib64/libc.so.6
#2  0x00007f07ab403eb9 in std::this_thread::__sleep_for(std::chrono::duration<long, std::ratio<1l, 1l> >, std::chrono::duration<long, std::ratio<1l, 1000000000l> >) () from /lib64/libstdc++.so.6
#3  0x00000000004018cc in void std::this_thread::sleep_for<long, std::ratio<1l, 1l> >(std::chrono::duration<long, std::ratio<1l, 1l> > const&) ()
#4  0x00000000004012de in fun_int(int) ()
#5  0x0000000000404696 in int std::_Bind_simple<int (*(int))(int)>::_M_invoke<0ul>(std::_Index_tuple<0ul>) ()
#6  0x000000000040443d in std::_Bind_simple<int (*(int))(int)>::operator()() ()
#7  0x000000000040436e in std::thread::_Impl<std::_Bind_simple<int (*(int))(int)> >::_M_run() ()
#8  0x00007f07ab404070 in ?? () from /lib64/libstdc++.so.6
#9  0x00007f07ab65ddd5 in start_thread () from /lib64/libpthread.so.0
#10 0x00007f07aab67ead in clone () from /lib64/libc.so.6
Thread 2 (Thread 0x7f07aa268700 (LWP 45709)):
#0  0x00007f07aab2ee2d in nanosleep () from /lib64/libc.so.6
#1  0x00007f07aab2ecc4 in sleep () from /lib64/libc.so.6
#2  0x00007f07ab403eb9 in std::this_thread::__sleep_for(std::chrono::duration<long, std::ratio<1l, 1l> >, std::chrono::duration<long, std::ratio<1l, 1000000000l> >) () from /lib64/libstdc++.so.6
#3  0x00000000004018cc in void std::this_thread::sleep_for<long, std::ratio<1l, 1l> >(std::chrono::duration<long, std::ratio<1l, 1l> > const&) ()
#4  0x0000000000401340 in fun_string(std::string const&) ()
#5  0x000000000040459f in int std::_Bind_simple<int (*(char const*))(std::string const&)>::_M_invoke<0ul>(std::_Index_tuple<0ul>) ()
#6  0x000000000040441f in std::_Bind_simple<int (*(char const*))(std::string const&)>::operator()() ()
#7  0x0000000000404350 in std::thread::_Impl<std::_Bind_simple<int (*(char const*))(std::string const&)> >::_M_run() ()
#8  0x00007f07ab404070 in ?? () from /lib64/libstdc++.so.6
#9  0x00007f07ab65ddd5 in start_thread () from /lib64/libpthread.so.0
#10 0x00007f07aab67ead in clone () from /lib64/libc.so.6
Thread 1 (Thread 0x7f07aba80740 (LWP 45707)):
#0  0x00007f07ab65ef47 in pthread_join () from /lib64/libpthread.so.0
#1  0x00007f07ab403e37 in std::thread::join() () from /lib64/libstdc++.so.6
#2  0x0000000000401455 in main ()
```



## ldd

在我们编译过程中通常会提示编译失败，通过输出错误信息发现是找不到函数定义，再或者编译成功了，但是运行时候失败(往往是因为依赖了非正常版本的lib库导致)，这个时候，我们就可以**通过ldd来分析该可执行文件依赖了哪些库以及这些库所在的路径。**

用来查看程式运行所需的共享库,常用来解决程式因缺少某个库文件而不能运行的一些问题。

仍然查看可执行程序test_thread的依赖库，输出如下：

```cpp
ldd -r ./test_thread
 linux-vdso.so.1 =>  (0x00007ffde43bc000)
 libpthread.so.0 => /lib64/libpthread.so.0 (0x00007f8c5e310000)
 libstdc++.so.6 => /lib64/libstdc++.so.6 (0x00007f8c5e009000)
 libm.so.6 => /lib64/libm.so.6 (0x00007f8c5dd07000)
 libgcc_s.so.1 => /lib64/libgcc_s.so.1 (0x00007f8c5daf1000)
 libc.so.6 => /lib64/libc.so.6 (0x00007f8c5d724000)
 /lib64/ld-linux-x86-64.so.2 (0x00007f8c5e52c000)
```

在上述输出中：

- 第一列：程序需要依赖什么库
- 第二列：系统提供的与程序需要的库所对应的库
- 第三列：库加载的开始地址

在有时候，我们通过ldd查看依赖库的时候，会提示找不到库，如下：

```cpp
ldd -r test_process
 linux-vdso.so.1 =>  (0x00007ffc71b80000)
 libstdc++.so.6 => /lib64/libstdc++.so.6 (0x00007fe4badd5000)
 libm.so.6 => /lib64/libm.so.6 (0x00007fe4baad3000)
 libgcc_s.so.1 => /lib64/libgcc_s.so.1 (0x00007fe4ba8bd000)
 libc.so.6 => /lib64/libc.so.6 (0x00007fe4ba4f0000)
 /lib64/ld-linux-x86-64.so.2 (0x00007fe4bb0dc000)
  liba.so => not found
```

比如上面最后一句提示，liba.so找不到，这个时候，需要我们知道liba.so的路径，比如在/path/to/liba.so，那么可以有下面两种方式：

```shell
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/
```

这样在通过ldd查看，就能找到对应的lib库，但是这个缺点是临时的，即退出终端后，再执行ldd，仍然会提示找不到该库，所以就有了另外一种方式，即通过修改/etc/ld.so.conf，在该文件的后面加上需要的路径，即

```shell
include ld.so.conf.d/*.conf
/path/to/
```

然后通过如下命令，即可永久生效

```shell
 /sbin/ldconfig
```

## C++filt

[c++filt(1) — Linux manual page](https://man7.org/linux/man-pages/man1/c++filt.1.html)

因为c++支持重载，也就引出了编译器的`name mangling`机制，对函数进行重命名。

我们通过strings命令查看test_thread中的函数信息(仅输出fun等相关)

```shell
strings test_thread | grep fun_
in fun_int n =
in fun_string s =
_GLOBAL__sub_I__Z7fun_inti
_Z10fun_stringRKSs
```

可以看到_Z10fun_stringRKSs这个函数，如果想知道这个函数定义的话，可以使用c++filt命令，如下：

```shell
 c++filt _Z10fun_stringRKSs
fun_string(std::basic_string<char, std::char_traits<char>, std::allocator<char> > const&)
```

通过上述输出，我们可以将编译器生成的函数名还原到我们代码中的函数名即fun_string。

### 命令格式

```shell
c++filt [-_|--strip-underscore]
		[-n|--no-strip-underscore]
		[-p|--no-params]
		[-t|--types]
		[-i|--no-verbose]
		[-s format|--format=format]
		[--help]  [--version]  [symbol...]

```

注意，如果没有给出符号参数 [symbol…]，c++filt 将从标准输入中读取符号名称。

### 选项说明

```cpp
-_, --strip-underscore
	在某些系统中，C和C++编译器都在每个名字前面加下划线。例如，C 名称 foo 获得低级名称为 _foo。此选项用于删除初始下划线，c++filt 是否默认删除下划线是依赖于目标的
-n, --no-strip-underscore
	不删除初始下划线
-p, --no-params
	当解析函数名时，不显示函数参数的类型
-t, --types
	试图解析类型与函数名
-i, --no-verbose
	输出结果中不包括任何实现细节
-s, --format=FORMAT
	c++filt 可以解析不同编译器修饰的符号，此选项用于指明符号修饰所采用的方法：
	"auto"：根据可执行文件自动选择符号解析方法，此为默认选项
	"gnu"： GNU C++ compiler （g++）的符号修饰方法
	"lucid"： Lucid compiler （lcc）的符号修饰方法
	"arm"：C++ Annotated Reference Manual 指明的方法
	"hp"：HP compiler （aCC）的符号修饰方法
	"edg"：EDG compiler 的符号修饰方法
	"gnu-v3"：GNU C++ compiler (g++) with the V3 ABI 的符号修饰方法
	"java"：GNU Java compiler （gcj）的符号修饰方法
	"gnat"：GNU Ada compiler (GNAT) 的符号修饰方法
--help
	显示帮助信息
--version
	显示版本信息
@FILE
	从文件 FILE 中读取命令行选项，读取的选项将插入到 @FILE 选项的位置。如果文件不存在，或者无法读取，那么选项 @FILE 将被按照字面意义处理，而不是被忽略

```

