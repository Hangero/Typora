# Cmake Practice

​                                                  --Cjacker

## 前言

cmake 已经开发了 5,6 年的时间，如果没有 KDE4，也许不会有人或者 Linux 发行版本重视 cmake，因为除了 Kitware 似乎没有人使用它。通过 KDE4 的选型和开发，cmake逐渐进入了人们的视线，在实际的使用过程中，cmake 的优势也逐渐的被大家所认识，至少 KDE 的开发者们给予了 cmake 极高的评价，同时庞大的 KDE 项目使用 cmake 来作为构建工具也证明了 cmake 的可用性和大项目管理能力。

所以，cmake 应该感谢 KDE，也正因为如此，cmake 的开发者投入了 KDE 从autotools 到 cmake 的迁移过程中，并相当快速和顺利的完成了迁移，现在整个 KDE4 开发版本全部使用 cmake 构建。

这也是促使我们学习 cmake 的原因，首先 cmake 被接受并成功应用，其次，cmake的优势在实际使用中不断的体现出来。

我们为什么不来认识一下这款优秀的工程构建工具呢？

在 2006 年 KDE 大会，听 cmake 开发者当面介绍了 cmake 之后，我就开始关注cmake，并将 cmake 纳入了 Everest 发行版，作为系统默认组件。最近 QT-4.3 也正式进入了 Everest 系统，为 KDE4 构建完成了准备工作。

但是，在学习 cmake 的过程中，发现官方的文档非常的少，而且错误也较多，比如:在介绍 Find<Name>模块编写的文档中，模块名称为 FOO，但是后面却出现了Foo_FIND_QUIETLY 的定义，这显然是错误的，这样的定义永远不可能有效，正确的定义是 FOO_FIND_QUIETLY 。种种原因，促使我开始写一份 “ 面向使用和实用 ” 的 cmake 文档，也就是本教程《cmake 实践》(Cmake Practice)

本文档是边学习边编写的成果，更像是一个学习笔记和 Tutorial，因此难免有失误或者理解不够透彻的地方，比如，我仍然不能理解为什么绝大部分使用变量的情况要通过`${}`引用，而在 IF 语句中却必须直接使用变量名。也希望能够有 cmake 的高手来指点迷津。

> 补：从 cmake 的 maillist,我找到了一些答案，原文是：The `IF(var)` or `IF(NOT var)` command expects `var` to be thename of a variable. This is stated in CMake's manual. So, for your situation `IF(${libX})` is the same as `IF(/usr/lib/xorg)` and then CMake will check the value of the variable named `/usr/lib/xorg`.**也就是说 IF 需要的是变量名而不是变量值**

这个文档是开放的，开放的目的是为了让更多的人能够读到并且能够修改，任何人都可以对它作出修改和补充，但是，为了大家都能够获得你关于 cmake 的经验和积累，如果你现错误或者添加了新内容后，请务必 CC 给我一份，让我们共同把 cmake 掌握的更好。

## 一.	初识cmake

> Cmake 不再使你在构建项目时郁闷地想自杀了.
> 																			--一位 KDE 开发者

### 1.	背景知识

cmake 是 kitware 公司以及一些开源开发者在开发几个工具套件(VTK)的过程中衍生品，最终形成体系，成为一个独立的开放源代码项目。项目的诞生时间是 2001 年。其官方网站是 www.cmake.org，可以通过访问官方网站获得更多关于 cmake 的信息。cmake的流行其实要归功于 KDE4 的开发(似乎跟当年的 svn 一样，KDE 将代码仓库从 CVS 迁移到SVN，同时证明了 SVN 管理大型项目的可用性)，在 KDE 开发者使用了近 10 年 autotools之后，他们终于决定为 KDE4 选择一个新的工程构建工具，其根本原因用 KDE 开发者的话来说就是：只有少数几个 “ 编译专家 ” 能够掌握 KDE 现在的构建体系(admin/Makefile.common)，在经历了 unsermake, scons 以及 cmake 的选型和尝试之后，KDE4 决定使用 cmake 作为自己的构建系统。在迁移过程中，进展异常的顺利，并获得了 cmake 开发者的支持。所以，目前的 KDE4 开发版本已经完全使用 cmake 来进行构建。像 kdesvn,rosegarden 等项目也开始使用 cmake，这也注定了 cmake 必然会成为一个主流的构建体系。

### 2.	特点

cmake 的特点主要有：

1. 开放源代码，使用类 BSD 许可发布。http://cmake.org/HTML/Copyright.html
2. 跨平台，并可生成 native 编译配置文件，在 Linux/Unix 平台，生成 makefile，在苹果平台，可以生成 xcode，在 Windows 平台，可以生成 MSVC 的工程文件。
3. 能够管理大型项目，KDE4 就是最好的证明。
4. 简化编译构建过程和编译过程。Cmake 的工具链非常简单：cmake+make。
5. 高效虑，按照 KDE 官方说法，CMake 构建 KDE4 的 kdelibs 要比使用 autotools 来
   构建 KDE3.5.6 的 kdelibs 快 40% ，主要是因为 Cmake 在工具链中没有 libtool。
6. 可扩展，可以为 cmake 编写特定功能的模块，扩充 cmake 功能。



### 3.	问题

1. cmake 很简单，但绝对没有听起来或者想象中那么简单。
2. **cmake 编写的过程实际上是编程的过程**，跟以前使用 autotools 一样，不过你需要编写的是 CMakeLists.txt(每个目录一个) ，使用的是 ” cmake 语言和语法 ”。
3. cmake 跟已有体系的配合并不是特别理想，比如 pkgconfig，您在实际使用中会有所体会，虽然有一些扩展可以使用，但并不理想。

### 4.	个人建议

1. 如果你没有实际的项目需求，那么看到这里就可以停下来了，因为 cmake 的学习过程就是实践过程，没有实践，读的再多几天后也会忘记。
2. 如果你的工程只有几个文件，直接编写 Makefile 是最好的选择。
3. 如果使用的是 C/C++/Java 之外的语言，请不要使用 cmake(至少目前是这样)
4. 如果你使用的语言有非常完备的构建体系，比如 java 的 ant，也不需要学习 cmake，虽然有成功的例子，比如 QT4.3 的 csharp 绑定 qyoto。
5. 如果项目已经采用了非常完备的工程管理工具，并且不存在维护问题，没有必要迁移到cmake
6. 如果仅仅使用 qt 编程，没有必要使用 cmake，因为 qmake 管理 Qt 工程的专业性和自动化程度比 cmake 要高很多。



## 二.	安装cmake

cmake 目前已经成为各大 Linux 发行版提供的组件，比如 Everest 直接在系统中包含，Fedora 在 extra 仓库中提供，所以，需要自己动手安装的可能性很小。如果你使用的操作系统(比如 Windows 或者某些 Linux 版本)没有提供 cmake 或者包含的版本较旧，建议你直接从 cmake 官方网站下载安装。http://www.cmake.org/HTML/Download.html在这个页面，提供了源代码的下载以及针对各种不同操作系统的二进制下载，可以选择适合自己操作系统的版本下载安装。因为各个系统的安装方式和包管理格式有所不同，在此就不再赘述了，相信一定能够顺利安装 cmake。

## 三.	初试cmake -- cmake的helloworld

> Hello world ，世界 你好

本节选择了一个最简单的例子 Helloworld 来演练一下 cmake 的完整构建过程，本节并不会深入的探讨 cmake，仅仅展示一个简单的例子，并加以粗略的解释。我们选择了Everest Linux 作为基本开发平台，因为这个只有一张 CD 的发行版本，包含了 gcc-4.2/gtk/qt3/qt4 等完整的开发环境，同时，系统默认集成了 cmake 最新版本 2.4.6。

### 1.	准备工作

首先，在`/backup `目录建立一个 cmake 目录，用来放置我们学习过程中的所有练习。

```shell
mkdir -p /backup/cmake
```

以后我们所有的 cmake 练习都会放在`/backup/cmake`的子目录下(你也可以自行安排目录，这个并不是限制，仅仅是为了叙述的方便)

然后在 cmake 建立第一个练习目录 t1

```shell
cd /backup/cmake
mkdir t1
cd t1
```

在 t1 目录建立 main.c 和 CMakeLists.txt(注意文件名大小写)：

main.c 文件内容：

```cpp

//main.c
#include <stdio.h>
int main()
{
printf(“Hello World from t1 Main!\n”);
return 0;
}

```

CmakeLists.txt 文件内容：

```cmake

PROJECT (HELLO)
SET(SRC_LIST main.c)
MESSAGE(STATUS "This is BINARY dir " ${HELLO_BINARY_DIR})
MESSAGE(STATUS "This is SOURCE dir "${HELLO_SOURCE_DIR})
ADD_EXECUTABLE(hello SRC_LIST)
```













































​              /