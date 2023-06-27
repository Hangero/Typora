# CMake Practice

## 目录用途

| 目录名  | 用途   |
| ------- | ------ |
| include | 头文件 |
|         |        |
|         |        |
|         |        |
|         |        |
|         |        |
|         |        |



*参考*

*[Linux下CMake简明教程](https://blog.csdn.net/whahu1989/article/details/82078563?spm=1001.2014.3001.5506)*

*CMake Practice*

## 特点

1. 开放源代码,使用类 BSD 许可发布

2. 跨平台,并可生成 native 编译配置文件,在 Linux/Unix 平台,生成 makefile,在苹果平台,可以生成 xcode,在 Windows 平台,可以生成 MSVC 的工程文件。
   
3. 能够管理大型项目

4. 简化编译构建过程和编译过程。Cmake 的工具链非常简单:cmake+make

5. 高效率
6. 可扩展,可以为 cmake 编写特定功能的模块,扩充 cmake 功能

## 参数

1. ```cmake
   -D  #设置控制变量
   -G  #设置编译器
   ```

   

## 1

```cmake
PROJECT (HELLO)
SET(SRC_LIST main.c)
MESSAGE(STATUS "This is BINARY dir " ${HELLO_BINARY_DIR})
MESSAGE(STATUS "This is SOURCE dir "${HELLO_SOURCE_DIR})
ADD_EXECUTABLE(hello ${SRC_LIST})
```

### progect

```cmake
PROJECT(projectname [CXX] [C] [Java])
```

这个指令定义工程名称,并可指定工程支持的语言,支持的语言列表是可以忽略的,默认情况表示支持所有语言。

这个指令隐式的定义了两个 cmake 变量:

- \<projectname>_BINARY_DIR
- \<projectname>_SOURCE_DIR

因为采用的是内部编译,两个变量目前指的都是工程所
在路径./t1

同时 cmake 系统也帮助我们预定义了 PROJECT_BINARY_DIR 和 PROJECT_SOURCE_DIR变量,他们的值分别跟 HELLO_BINARY_DIR 与 HELLO_SOURCE_DIR 一致。
为了统一起见,**建议以后直接使用PROJECT_BINARY_DIR,PROJECT_SOURCE_DIR**,即使修改了工程名称,也不会影响这两个变量。如果使用了\<projectname>_SOURCE_DIR ,修改工程名称后,需要同时修改这些变量。

### set

```cmake
SET(VAR [VALUE] [CACHE TYPE DOCSTRING [FORCE]])

SET(SRC_LIST main.c t1.c t2.c)。
```

SET 显式地定义变量

### message

```cmake
MESSAGE([SEND_ERROR | STATUS | FATAL_ERROR] "message to display"...)
```

这个指令用于向终端输出用户定义的信息,包含了三种类型:

- `SEND_ERROR`：产生错误,生成过程被跳过。
- `SATUS `：输出前缀为 — 的信息。
- `FATAL_ERROR`：立即终止所有 cmake 过程。

我们在这里使用的是 STATUS 信息输出,演示了由 PROJECT 指令定义的两个隐式变量`HELLO_BINARY_DIR` 和 `HELLO_SOURCE_DIR`。

### add_executable

```CMAKE
ADD_EXECUTABLE(hello ${SRC_LIST})
```

表示最终要生成的elf文件的名字叫hello，使用的源文件是${SRC_LIST}

> **在本例我们使用了`${}`来引用变量，这是 cmake 的变量应用方式，但是，有一些例外，比如在 IF 控制语句，变量是直接使用变量名引用，而不需要`${}`。如果使用了`${}`去应用变量，其实 IF 会去判断名为`${}`所代表的值的变量，那当然是不存在的了。**

### 基本语法规则1

1. 变量使用`${}`方式取值,但是在 IF 控制语句中是直接使用变量名
2. 指令(参数 1 参数 2...)
   参数使用括弧括起,参数之间使用空格或分号分开。
3. SET(SRC_LIST main.c)也可以写成 SET(SRC_LIST “main.c”)

### make clean

可以将之前产生的可执行档及其他档案删除

### 内部构建与外部构建

两者的区别仅仅是前者将生成的编译文件和源代码、CMakeLists.txt混杂在一起；后者是将所有动作全部发生在编译目录，更好删除编译生成的文件而已（直接删文件夹）

外部编译例子

1. 首先,请清除 t1 目录中除 main.c CmakeLists.txt 之外的所有中间文件,最关键的是 CMakeCache.txt。
   
2. 在 t1 目录中建立 build 目录,当然你也可以在任何地方建立 build 目录,不一定必须在工程目录中。
   
3. 进入 build 目录,运行 cmake ..

   (..代表父目录，因为父目录存在我们需要的CMakeLists.txt；<font color="yellow">**如果你在其他地方建立了 build 目录，需要运行 `cmake <CMakeList_path>`，即去寻找CMakeLists.txt**)</font>，查看一下 build 目录，就会发现了生成了编译需要的 Makefile 以及其他的中间文件。

4. 运行 make 构建工程,就会在当前目录(build 目录)中获得目标文件 hello。

## 2

```cmake
#为工程添加一个子目录 src,用来放置工程源代码;
#添加一个子目录 doc,用来放置这个工程的文档 hello.txt
#在工程目录添加文本文件 COPYRIGHT, README;
#在工程目录添加一个 runhello.sh 脚本,用来调用 hello 二进制
#将构建后的目标文件放入构建目录的 bin 子目录;
```

**最外层的CMakeLists.txt用于掌控全局，使用add_subdirectory来控制其它目录下的CMakeLists.txt的运行。**

src中

```
ADD_EXECUTABLE(hello main.c)
```

根目录中

```cmake
cmake_minimum_required(VERSION 2.8)
PROJECT(HELLO)
ADD_SUBDIRECTORY(src ~/learncmake/t2/bin)
```

### add_subdirectory

```cmake
ADD_SUBDIRECTORY(source_dir [binary_dir] [EXCLUDE_FROM_ALL])
```

这个指令**用于向当前工程添加存放源文件的子目录,并可以指定中间二进制和目标二进制存放的位置**。`EXCLUDE_FROM_ALL `参数的含义是将这个目录从编译过程中排除。上面的例子定义了将 src 子目录加入工程,并指定编译输出(包含编译中间结果)路径为bin 目录。如果不进行 bin 目录的指定,那么编译结果(包括中间结果)都将存放在build/src 目录。

### set

```cmake
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
```

通过 SET 指令重新定义 EXECUTABLE_OUTPUT_PATH 和 LIBRARY_OUTPUT_PATH 变量([CMake自带的预定义变量](https://blog.csdn.net/qq_33726635/article/details/121896681?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165830949216782395396860%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=165830949216782395396860&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-2-121896681-null-null.142^v32^experiment_2_v1,185^v2^control&utm_term=LIBRARY_OUTPUT_PATH%20%E5%8F%98%E9%87%8F&spm=1018.2226.3001.4187))
来指定最终的目标二进制的位置(指最终生成的 hello 或者最终的共享库,不包含编译生成
的中间文件)

- EXECUTABLE_OUTPUT_PATH ：目标二进制可执行文件的存放位置
- PROJECT_SOURCE_DIR：工程的根目录
- LIBRARY_OUTPUT_PATH：库文件的输出目录

我应该把这两条指令写在工程的 CMakeLists.txt 还是 src 目录下的CMakeLists.txt？

把握一个简单的原则：

<table><tr><td bgcolor=grey>  在哪里`ADD_EXECUTABLE` 或 `ADD_LIBRARY`，如果需要改变目标存放路径,就在哪里加入上述的定义。</td></tr></table>

### 源码安装 install

源码的安装一般由3个步骤组成：配置(configure)、编译(make)、安装( make install ).

#### CMAKE_INSTALL_PREFIX

用于书写相对路径

```cmake
cmake -D CMAKE_INSTALL_PREFIX=<INSTALL_PATH>
```

#### INSTALL

##### 目标文件

INSTALL 指令用于定义安装规则,安装的内容可以包括目标二进制、动态库、静态库以及文件、目录、脚本等。

```CMAKE
INSTALL(TARGETS <targets> ...
         [[ARCHIVE|LIBRARY|RUNTIME]
         #   静态库  |  动态库  |  可执行目标二进制
                                   [DESTINATION <dir>]
                                   [PERMISSIONS permissions...]
                                   [CONFIGURATIONS
         [Debug|Release|...]]
                                   [COMPONENT <component>]
                                   [OPTIONAL]
                                   ] [...])
```

| 参数        | 用法                                                         |
| ----------- | ------------------------------------------------------------ |
| TARGETS     | 后跟通过 ADD_EXECUTABLE 或者 ADD_LIBRARY 定义的目标文件,可能是可执行二进制、动态库、静态库 |
| DESTINATION | 定义安装路径（**目标目录**），安装后的路径就是${CMAKE_INSTALL_PREFIX}/<DESTINATION 定义的路径> |

```cmake
INSTALL(TARGETS myrun mylib mystaticlib
            RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
      		ARCHIVE DESTINATION libstatic
			)
```

- 可执行二进制 myrun 安装到`${CMAKE_INSTALL_PREFIX}/bin `目录
- 动态库 libmylib 安装到`\${CMAKE_INSTALL_PREFIX}/lib `目录
- 静态库 libmystaticlib 安装到`${CMAKE_INSTALL_PREFIX}/libstatic` 目录

##### 普通文件

```cmake
INSTALL(FILES files... DESTINATION <dir>
			[PERMISSIONS permissions...]
			[CONFIGURATIONS [Debug|Release|...]]
			[COMPONENT <component>]
			[RENAME <name>] [OPTIONAL])
```

安装一般文件,并可以指定访问权限,文件名是此指令所在路径下的相对路径。如果
默认不定义权限 PERMISSIONS,安装后的权限为:

 OWNER_READ(4)	OWNER_WRITE(2)		GROUP_READ(4)		 WORLD_READ(4)    即 644 权限。

##### 非目标文件的可执行程序

```cmake
INSTALL(PROGRAMS files... DESTINATION <dir>
			[PERMISSIONS permissions...]
			[CONFIGURATIONS [Debug|Release|...]]
			[COMPONENT <component>]
			[RENAME <name>] [OPTIONAL])
```

OWNER_EXECUTE	GROUP_EXECUTE	WORLD_EXECUTE	即 755 权限

##### 目录

```cmake
INSTALL(DIRECTORY dirs... DESTINATION <dir>
			[FILE_PERMISSIONS permissions...]
			[DIRECTORY_PERMISSIONS permissions...]
			[USE_SOURCE_PERMISSIONS]
			[CONFIGURATIONS [Debug|Release|...]]
			[COMPONENT <component>]
			[[PATTERN <pattern> | REGEX <regex>]
			[EXCLUDE] [PERMISSIONS permissions...]] [...])
```

**DIRECTORY **后面连接的是所在 Source 目录的相对路径，记得加`/`

> 但务必注意:
> **abc 和 abc/有很大的区别:如果目录名不以/结尾,那么这个目录将被安装为目标路径下的 abc,如果目录名以/结尾,代表将这个目录中的内容安装到目标路径,但不包括这个目录本身。**

**PATTERN **用于使用正则表达式进行过滤

**PERMISSIONS** 用于指定 PATTERN 过滤后的文件权限

```cmake
INSTALL(DIRECTORY icons scripts/ DESTINATION share/myproj
			PATTERN "CVS" EXCLUDE
			PATTERN "scripts/*"
			PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
			GROUP_EXECUTE GROUP_READ)
```

这条指令的执行结果是:
将 icons 目录安装到 \<prefix>/share/myproj,将 scripts/中的内容安装到
\<prefix>/share/myproj
不包含目录名为 CVS 的目录,对于 scripts/* 文件指定权限为 OWNER_EXECUTE
OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ.

##### 安装时 CMAKE 脚本的执行

```cmake
INSTALL([[SCRIPT <file>] [CODE <code>]] [...])
```

- SCRIPT 参数用于在安装时调用 cmake 脚本文件(也就是\<abc>.cmake 文件)
- CODE 参数用于执行 CMAKE 指令,必须以双引号括起来。比如:
  INSTALL(CODE "MESSAGE(\"Sample install message.\")")

## 3. 静态库与动态库构建

```cmake
#主目录
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(HELLOLIB)
ADD_SUBDIRECTORY(lib )

#./lib目录
SET(LIBHELLO_SRC hello.c)
SET(LIBRARY_OUTPUT_PATH ~/learncmake/t3/lib)
ADD_LIBRARY(hello SHARED ${LIBHELLO_SRC})
```

### add_library

```cmake
ADD_LIBRARY(libname         [SHARED|STATIC|MODULE]
		[EXCLUDE_FROM_ALL]
						source1 source2 ... sourceN)
```

你不需要写全 libhello.so,只需要填写 hello 即可,cmake 系统会自动为你生成libhello.X
类型有三种:

- SHARED：动态库  .so
- STATIC：静态库     .a
- MODULE：在使用 dyld 的系统有效,如果不支持 dyld,则被当作 SHARED 对待。

`EXCLUDE_FROM_ALL `参数的意思是这个库不会被默认构建,除非有其他的组件依赖或者手工构建。

### set_target_properties

```cmake
SET_TARGET_PROPERTIES(target1 target2 ...
										PROPERTIES prop1 value1
										prop2 value2 ...)
```

用来设置输出的名称,对于动态库,还可以用来指定动态库版本和 API 版本

## 4.  使用外部共享库和头文件

### include_directories

引入头文件搜索路径

```cmake
INCLUDE_DIRECTORIES([AFTER|BEFORE] [SYSTEM] dir1 dir2 ...)
```

这条指令可以用来向工程添加多个特定的头文件搜索路径,路径之间用空格分割,如果路径中包含了空格,可以使用双引号将它括起来,默认的行为是追加到当前的头文件搜索路径的后面,你可以通过两种方式来进行控制搜索路径添加的方式:

1. CMAKE_INCLUDE_DIRECTORIES_BEFORE,通过 SET 这个 cmake 变量为 on,可以
   将添加的头文件搜索路径放在已有路径的前面。
2. 通过 AFTER 或者 BEFORE 参数,也可以控制是追加还是置前。

### LINK_DIRECTORIES

```cmake
LINK_DIRECTORIES(directory1 directory2 ...)
```

添加非标准的共享库搜索路径,比如,在工程内部同时存在共享库和可执行二进制,在编译时就需要指定一下这些共享库的路径

### TARGET_LINK_LIBRARIES

```cmake
TARGET_LINK_LIBRARIES(target library1
													<debug | optimized> library2
													...)
```

这个指令可以用来为 target 添加需要链接的共享库,本例中是一个可执行文件,但是同样可以用于为自己编写的共享库添加共享库链接。











+









# CMake

## 常用变量和常用环境变量

### 变量引用的方式

使用`${}`进行变量的引用。在 IF 等语句中,是直接使用变量名

### 自定义变量的方式

主要有隐式定义和显式定义两种

- 隐式定义的例子：就是 PROJECT 指令，在定义工程名称的同时，他会隐式的定义\<projectname>\_BINARY_DIR 和\<projectname>_SOURCE_DIR 两个变量。
- 显式定义的例子：使用 SET 指令,就可以构建一个自定义变量了。比如:
  SET(HELLO_SRC main.SOURCE_PATHc),就 PROJECT_BINARY_DIR 可以通过
  ${HELLO_SRC}来引用这个自定义变量了.

### 常用变量

#### CMAKE_BINARY_DIR

CMAKE_BINARY_DIR；PROJECT_BINARY_DIR ； \<projectname>_BINARY_DIR这三个变量指代的内容是一致的,如果是 in source 编译,指得就是工程顶层目录**如是 out-of-source 编译,指的是工程编译发生的目录。**PROJECT_BINARY_DIR 跟其他指令稍有区别,现在,你可以理解为他们是一致的。

#### CMAKE_SOURCE_DIR

CMAKE_SOURCE_DIR  ；PROJECT_SOURCE_DIR  ；\<projectname>_SOURCE_DIR这三个变量指代的内容是一致的,**不论采用何种编译方式,都是工程顶层目录**。也就是在 in source 编译时,他跟 CMAKE_BINARY_DIR 等变量一致。PROJECT_SOURCE_DIR 跟其他指令稍有区别,现在,你可以理解为他们是一致的。

#### CMAKE_CURRENT_SOURCE_DIR

指的是当前处理的 CMakeLists.txt 所在的路径。

#### CMAKE_CURRRENT_BINARY_DIR

如果是 in-source 编译,它跟 CMAKE_CURRENT_SOURCE_DIR 一致,如果是 out-of-
source 编译,他指的是 target 编译目录。

**使用我们上面提到的 ADD_SUBDIRECTORY(src bin)可以更改这个变量的值。
使用 SET(EXECUTABLE_OUTPUT_PATH <新路径>)并不会对这个变量造成影响,它仅仅
修改了最终目标文件存放的路径。**

#### CMAKE_CURRENT_LIST_FILE

输出调用这个变量的 CMakeLists.txt 的完整路径

#### CMAKE_CURRENT_LIST_LINE

输出这个变量所在的行

#### CMAKE_MODULE_PATH

用来定义自己的 cmake 模块所在的路径。如果你的工程比较复杂,有可能会自己
编写一些 cmake 模块,这些 cmake 模块是随你的工程发布的,为了让 cmake 在处理
CMakeLists.txt 时找到这些模块,你需要通过 SET 指令,将自己的 cmake 模块路径设
置一下。

#### EXECUTABLE_OUTPUT_PATH 和 LIBRARY_OUTPUT_PATH

分别用来重新定义最终结果的存放目录

#### PROJECT_NAME

返回通过 PROJECT 指令定义的项目名称。

### 调用环境变量

使用`$ENV{NAME}`指令可以调用系统的环境变量

#### CMAKE_INCLUDE_CURRENT_DIR

自动添加 CMAKE_CURRENT_BINARY_DIR 和CMAKE_CURRENT_SOURCE_DIR 到当前处理
的 CMakeLists.txt。相当于在每个 CMakeLists.txt 加入:
INCLUDE_DIRECTORIES(\${CMAKE_CURRENT_BINARY_DIR}
\${CMAKE_CURRENT_SOURCE_DIR})

#### CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE

将工程提供的头文件目录始终至于系统头文件目录的前面,当你定义的头文件确实跟系统发生冲突时可以提供一些帮助。

#### CMAKE_INCLUDE_PATH 和 CMAKE_LIBRARY_PATH

### 系统信息

| 变量                   | 含义                             |
| ---------------------- | -------------------------------- |
| CMAKE_MAJOR_VERSION    | CMAKE 主版本号,比如 2.4.6 中的 2 |
| CMAKE_MINOR_VERSION    | CMAKE 次版本号,比如 2.4.6 中的 4 |
| CMAKE_PATCH_VERSION    | CMAKE 补丁等级,比如 2.4.6 中的 6 |
| CMAKE_SYSTEM           | 系统名称,比如 Linux-2.6.22       |
| CMAKE_SYSTEM_NAME      | 不包含版本的系统名,比如 Linux    |
| CMAKE_SYSTEM_VERSION   | 系统版本,比如 2.6.22             |
| CMAKE_SYSTEM_PROCESSOR | 处理器名称,比如 i686             |
| UNIX                   | 在所有的类 UNIX 平台为 TRUE      |
| WIN32                  | 在所有的 win32 平台为 TRUE       |

### 开关选项

```cmake
CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS,
#用来控制 IF ELSE 语句的书写方式
BUILD_SHARED_LIBS
#这个开关用来控制默认的库编译方式,如果不进行设置,使用 ADD_LIBRARY 并没有指定库
#类型的情况下,默认编译生成的库都是静态库。
#如果 SET(BUILD_SHARED_LIBS ON)后,默认生成的为动态库。
CMAKE_C_FLAGS
#设置 C 编译选项,也可以通过指令 ADD_DEFINITIONS()添加。
CMAKE_CXX_FLAGS
#设置 C++编译选项,也可以通过指令 ADD_DEFINITIONS()添加。
```

## 常用指令

### 基本指令

#### ADD_DEFINITIONS

向 C/C++编译器添加-D 定义,比如:
ADD_DEFINITIONS(-DENABLE_DEBUG -DABC),参数之间用空格分割。如果你的代码中定义了#ifdef ENABLE_DEBUG #endif,这个代码块就会生效。如果要添加其他的编译器开关,可以通过 CMAKE_C_FLAGS 变量和 CMAKE_CXX_FLAGS 变量设置

#### ADD_DEPENDENCIES

定义 target 依赖的其他 target,确保在编译本 target 之前,其他的 target 已经被构建。

```cmake
ADD_DEPENDENCIES(target-name depend-target1
											depend-target2 ...)
```

#### ADD_TEST 与 ENABLE_TESTING

ENABLE_TESTING 指令用来控制 Makefile 是否构建 test 目标,涉及工程所有目录。语
法很简单,没有任何参数,ENABLE_TESTING(),一般情况这个指令放在工程的主CMakeLists.txt 中.

```cmake
ADD_TEST(testname Exename arg1 arg2 ...)
```

testname 是自定义的 test 名称,Exename 可以是构建的目标文件也可以是外部脚本等等。后面连接传递给可执行文件的参数。如果没有在同一个 CMakeLists.txt 中打开ENABLE_TESTING()指令,任何 ADD_TEST 都是无效的。

#### AUX_SOURCE_DIRECTORY

```cmake
AUX_SOURCE_DIRECTORY(dir VARIABLE)
```

作用是发现一个目录下所有的源代码文件并将列表存储在一个变量中,这个指令临时被用来自动构建源文件列表。因为目前 cmake 还不能自动发现新添加的源文件。

#### ADD_COMPILE_OPTIONS

添加编译选项

#### CMAKE_MINIMUM_REQUIRED

```cmake
CMAKE_MINIMUM_REQUIRED(VERSION versionNumber [FATAL_ERROR])
```

#### EXEC_PROGRAM

在 CMakeLists.txt 处理过程中执行命令,并不会在生成的 Makefile 中执行。

```cmake
EXEC_PROGRAM(Executable [directory in which to run]
											[ARGS <arguments to executable>]
											[OUTPUT_VARIABLE <var>]
											[RETURN_VALUE <var>])
```

用于在指定的目录运行某个程序,通过 ARGS 添加参数,如果要获取输出和返回值,可通过
OUTPUT_VARIABLE 和 RETURN_VALUE 分别定义两个变量。这个指令可以帮助你在 CMakeLists.txt 处理过程中支持任何命令,比如根据系统情况去修改代码文件等等。

#### FILE

文件操作指令

```cmake
ILE(WRITE filename "message to write"... )
FILE(APPEND filename "message to write"... )
FILE(READ filename variable)
FILE(GLOB   variable [RELATIVE path] [globbing    
expressions]...)
FILE(GLOB_RECURSE variable [RELATIVE path]
			[globbing expressions]...)
FILE(REMOVE [directory]...)
FILE(REMOVE_RECURSE [directory]...)
FILE(MAKE_DIRECTORY [directory]...)
FILE(RELATIVE_PATH variable directory file)
FILE(TO_CMAKE_PATH path result)
FILE(TO_NATIVE_PATH path result)
```

#### INCLUDE

用来载入 CMakeLists.txt 文件,也用于载入预定义的 cmake 模块

```cmake
INCLUDE(file1 [OPTIONAL])
INCLUDE(module [OPTIONAL])
```

OPTIONAL 参数的作用是文件不存在也不会产生错误。
你可以指定载入一个文件,如果定义的是一个模块,那么将在 CMAKE_MODULE_PATH 中搜
索这个模块并载入。
载入的内容将在处理到 INCLUDE 语句是直接执行

#### INCLUDE_DIRECTORIES

向工程添加多个指定头文件的搜索路径，路径之间用空格分隔。

#### OPTION

添加控制选项，主要遇到的情况分为2种：

1. 本来要生成多个bin或库文件，现在只想生成部分指定的bin或库文件
2. 对于同一个bin文件，只想编译其中部分代码（使用宏来控制）

#### 前文已介绍

ADD_EXECUTABLE、ADD_LIBRARY、ADD_SUBDIRECTORY

### INSTALL

### FIND

```cmake
FIND_FILE(<VAR> name1 path1 path2 ...)
#VAR 变量代表找到的文件全路径,包含文件名
FIND_LIBRARY(<VAR> name1 path1 path2 ...)
#VAR 变量表示找到的库全路径,包含库文件名
FIND_PATH(<VAR> name1 path1 path2 ...)
#VAR 变量代表包含这个文件的路径。
FIND_PROGRAM(<VAR> name1 path1 path2 ...)
#VAR 变量代表包含这个程序的全路径。
FIND_PACKAGE(<name> [major.minor] [QUIET] [NO_MODULE]
[[REQUIRED|COMPONENTS] [componets...]])
#用来调用预定义在 CMAKE_MODULE_PATH 下的 Find<name>.cmake 模块,你也可以自己定义 Find<name>模块,通过 SET(CMAKE_MODULE_PATH dir)将其放入工程的某个目录中供工程使用,我们在后面的章节会详细介绍 FIND_PACKAGE 的使用方法和 Find 模块的编写。
```

##### FIND_LIBRARY

```CMAKE
FIND_LIBRARY(<VAR> name1 path1 path2 ...)
```

find_library: 在指定目录下查找指定库，并把**库的绝对路径**存放到变量里。使用find_library的好处是在执行`cmake ..`时就会去查找库是否存在，这样可以提前发现错误，不用等到链时。

对于name

- xxx：默认是查找动态库
- xxx.so：查找动态库
- xxx.a：查找静态库

##### target_link_libraries

把目标文件与库文件进行链接



### 控制指令

#### IF

```cmake
IF(expression)
	# THEN section.
	COMMAND1(ARGS ...)
	COMMAND2(ARGS ...)
	...
ELSE(expression)
	# ELSE section.
	COMMAND1(ARGS ...)
	COMMAND2(ARGS ...)
	...
ENDIF(expression)
```

另外一个指令是 ELSEIF,总体把握一个原则,凡是出现 IF 的地方一定要有对应的ENDIF.出现 ELSEIF 的地方,ENDIF 是可选的。

```cmake
IF(var)#如果变量不是:空,0,N, NO, OFF, FALSE, NOTFOUND 或<var>_NOTFOUND 时,表达式为真。
IF(NOT var )#与上述条件相反。
IF(var1 AND var2)#当两个变量都为真是为真。
IF(var1 OR var2)#当两个变量其中一个为真时为真。
IF(COMMAND cmd)#当给定的 cmd 确实是命令并可以调用是为真。
IF(EXISTS dir)或者 IF(EXISTS file)#当目录名或者文件名存在时为真。
IF(file1 IS_NEWER_THAN file2)#当 file1 比 file2 新,或者 file1/file2 其中有一个不存在时为真,文件名请使用完整路径。
IF(IS_DIRECTORY dirname)#当 dirname 是目录时,为真。
IF(variable MATCHES regex)
IF(string MATCHES regex)#当给定的变量或者字符串能够匹配正则表达式 regex 时为真。比如:
IF("hello" MATCHES "ell")
MESSAGE("true")
ENDIF("hello" MATCHES "ell")IF(variable LESS number)
IF(string LESS number)
IF(variable GREATER number)
IF(string GREATER number)
IF(variable EQUAL number)
IF(string EQUAL number)
```

#### WHILE

```cmake
WHILE(condition)
	COMMAND1(ARGS ...)
	COMMAND2(ARGS ...)
	...
ENDWHILE(condition)
```

#### FOREACH





## 自定义编译选项

[CMake教程](https://zhuanlan.zhihu.com/p/534439206)

CMake 允许为项目增加编译选项，从而可以根据用户的环境和需求选择最合适的编译方案。例如，可以将 MathFunctions 库设为一个可选的库，如果该选项为 `ON` ，就使用该库定义的数学函数来进行运算。否则就调用标准库中的数学函数库。

### 修改CMakeLists文件

```cmake
# CMake 最低版本号要求
cmake_minimum_required (VERSION 2.8)

# 项目信息
project (Demo4)

# 加入一个配置头文件，用于处理 CMake 对源码的设置
configure_file (
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "${PROJECT_BINARY_DIR}/config.h"
  )

# 是否使用自己的 MathFunctions 库
option (USE_MYMATH
       "Use provided math implementation" ON)

# 是否加入 MathFunctions 库
if (USE_MYMATH)
  include_directories ("${PROJECT_SOURCE_DIR}/math")
  add_subdirectory (math)  
  set (EXTRA_LIBS ${EXTRA_LIBS} MathFunctions)
endif (USE_MYMATH)

# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量
aux_source_directory(. DIR_SRCS)

# 指定生成目标
add_executable(Demo ${DIR_SRCS})
target_link_libraries (Demo  ${EXTRA_LIBS})
```

其中：

1. 第7行的 `configure_file` 命令用于加入一个配置头文件 config.h ，这个文件由 CMake 从 config.h.in 生成，通过这样的机制，将可以通过预定义一些参数和变量来控制代码的生成。
2. 第13行的 `option` 命令添加了一个 `USE_MYMATH` 选项，并且默认值为 `ON` 。
3. 第17行根据 `USE_MYMATH` 变量的值来决定是否使用我们自己编写的 MathFunctions 库。

### 修改main.cpp

之后修改 `main.cc` 文件，让其根据 `USE_MYMATH` 的预定义值来决定是否调用标准库还是 MathFunctions 库：

```cmake
#include <stdio.h>
#include <stdlib.h>
#include "config.h"

#ifdef USE_MYMATH
  #include "math/MathFunctions.h"
#else
  #include <math.h>
#endif


int main(int argc, char *argv[])
{
    if (argc < 3){
        printf("Usage: %s base exponent \n", argv[0]);
        return 1;
    }
    double base = atof(argv[1]);
    int exponent = atoi(argv[2]);
    
#ifdef USE_MYMATH
    printf("Now we use our own Math library. \n");
    double result = power(base, exponent);
#else
    printf("Now we use the standard library. \n");
    double result = pow(base, exponent);
#endif
    printf("%g ^ %d is %g\n", base, exponent, result);
    return 0;
}
```

### 编写`config.h.in`文件

```cmake
#cmakedefine USE_MYMATH
```

这样 CMake 会自动根据 CMakeLists 配置文件中的设置自动生成 config.h 文件。

### 编译项目

现在编译一下这个项目，为了便于交互式的选择该变量的值，可以使用 `ccmake` 命令（也可以使用 `cmake -i` 命令，该命令会提供一个会话式的交互式配置界面）：（没有成功）

![img](https://pic2.zhimg.com/80/v2-d796be171c8fd08b017403f9df999455_720w.webp)

CMake的交互式配置界面从中可以找到刚刚定义的 `USE_MYMATH` 选项，按键盘的方向键可以在不同的选项窗口间跳转，按下 `enter` 键可以修改该选项。修改完成后可以按下 `c` 选项完成配置，之后再按 `g` 键确认生成 Makefile 。ccmake 的其他操作可以参考窗口下方给出的指令提示。我们可以试试分别将 `USE_MYMATH` 设为 `ON` 和 `OFF` 得到的结果：

#### `USE_MYMATH` 为` ON`

```shell
[ehome@xman Demo4]$ ./Demo
Now we use our own MathFunctions library. 
 7 ^ 3 = 343.000000
 10 ^ 5 = 100000.000000
 2 ^ 10 = 1024.000000
```

此时 config.h 的内容为：

```cmake
#define USE_MYMATH
```

### `USE_MYMATH` 为 `OFF`

运行结果：

```cpp
[ehome@xman Demo4]$ ./Demo
Now we use the standard library. 
 7 ^ 3 = 343.000000
 10 ^ 5 = 100000.000000
 2 ^ 10 = 1024.000000
```

此时 config.h 的内容为：

```cpp
/* #undef USE_MYMATH */
```

## 安装和测试

CMake 也可以指定安装规则，以及添加测试。这两个功能分别可以通过在产生 Makefile 后使用 `make install` 和 `make test` 来执行。在以前的 GNU Makefile 里，你可能需要为此编写 `install` 和 `test` 两个伪目标和相应的规则，但在 CMake 里，这样的工作同样只需要简单的调用几条命令。

### 定制安装规则

首先先在 math/CMakeLists.txt 文件里添加下面两行：

```cmake
# 指定 MathFunctions 库的安装路径
install (TARGETS MathFunctions DESTINATION bin)
install (FILES MathFunctions.h DESTINATION include)
```

指明 MathFunctions 库的安装路径。之后同样修改根目录的 CMakeLists 文件，在末尾添加下面几行：

```cmake
# 指定安装路径
install (TARGETS Demo DESTINATION bin)
install (FILES "${PROJECT_BINARY_DIR}/config.h"
         DESTINATION include)
```

> DESTINATION  定义安装路径（**目标目录**），安装后的路径就是${CMAKE_INSTALL_PREFIX}/<DESTINATION 定义的路径>

通过上面的定制，生成的 Demo 文件和 MathFunctions 函数库 libMathFunctions.o 文件将会被复制到 `/usr/local/bin` 中，而 MathFunctions.h 和生成的 config.h 文件则会被复制到 `/usr/local/include` 中。我们可以验证一下（顺带一提的是，这里的 `/usr/local/` 是默认安装到的根目录，可以通过修改 `CMAKE_INSTALL_PREFIX` 变量的值来指定这些文件应该拷贝到哪个根目录）：

```shell
[ehome@xman Demo5]$ sudo make install
[ 50%] Built target MathFunctions
[100%] Built target Demo
Install the project...
-- Install configuration: ""
-- Installing: /usr/local/bin/Demo
-- Installing: /usr/local/include/config.h
-- Installing: /usr/local/bin/libMathFunctions.a
-- Up-to-date: /usr/local/include/MathFunctions.h
[ehome@xman Demo5]$ ls /usr/local/bin
Demo  libMathFunctions.a
[ehome@xman Demo5]$ ls /usr/local/include
config.h  MathFunctions.h
```

### 为工程添加测试

添加测试同样很简单。CMake 提供了一个称为 CTest 的测试工具。我们要做的只是在项目根目录的 CMakeLists 文件中调用一系列的 `add_test` 命令。

```cmake
# 启用测试
enable_testing()

# 测试程序是否成功运行
add_test (test_run Demo 5 2)

# 测试帮助信息是否可以正常提示
add_test (test_usage Demo)
set_tests_properties (test_usage
  PROPERTIES PASS_REGULAR_EXPRESSION "Usage: .* base exponent")

# 测试 5 的平方
add_test (test_5_2 Demo 5 2)

set_tests_properties (test_5_2
 PROPERTIES PASS_REGULAR_EXPRESSION "is 25")

# 测试 10 的 5 次方
add_test (test_10_5 Demo 10 5)

set_tests_properties (test_10_5
 PROPERTIES PASS_REGULAR_EXPRESSION "is 100000")

# 测试 2 的 10 次方
add_test (test_2_10 Demo 2 10)

set_tests_properties (test_2_10
 PROPERTIES PASS_REGULAR_EXPRESSION "is 1024")
```

上面的代码包含了四个测试。第一个测试 `test_run` 用来测试程序是否成功运行并返回 0 值。剩下的三个测试分别用来测试 5 的 平方、10 的 5 次方、2 的 10 次方是否都能得到正确的结果。其中 `PASS_REGULAR_EXPRESSION` 用来测试输出是否包含后面跟着的字符串。让我们看看测试的结果：

```shell
[ehome@xman Demo5]$ make test
Running tests...
Test project /home/ehome/Documents/programming/C/power/Demo5
    Start 1: test_run
1/4 Test #1: test_run .........................   Passed    0.00 sec
    Start 2: test_5_2
2/4 Test #2: test_5_2 .........................   Passed    0.00 sec
    Start 3: test_10_5
3/4 Test #3: test_10_5 ........................   Passed    0.00 sec
    Start 4: test_2_10
4/4 Test #4: test_2_10 ........................   Passed    0.00 sec

100% tests passed, 0 tests failed out of 4

Total Test time (real) =   0.01 sec
```



## 支持gdb

让 CMake 支持 gdb 的设置也很容易，只需要指定 `Debug` 模式下开启 `-g` 选项：

```cmake
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
```

## 添加环境检测

有时候可能要对系统环境做点检查，例如要使用一个平台相关的特性的时候。在这个例子中，我们检查系统是否自带 pow 函数。如果带有 pow 函数，就使用它；否则使用我们定义的 power 函数。

### 添加 CheckFunctionExists 宏

首先在顶层 CMakeLists 文件中添加 CheckFunctionExists.cmake 宏，并调用 `check_function_exists` 命令测试链接器是否能够在链接阶段找到 `pow` 函数。

```cmake
# 检查系统是否支持 pow 函数
include (${CMAKE_ROOT}/Modules/CheckFunctionExists.cmake)
check_function_exists (pow HAVE_POW)
```

将上面这段代码放在 `configure_file` 命令前。

### 预定义相关宏变量

接下来修改 `config.h.in` 文件，预定义相关的宏变量。

```cmake
// does the platform provide pow function?
#cmakedefine HAVE_POW
```

### 在代码中使用宏和函数

最后一步是修改 `main.cc` ，在代码中使用宏和函数：

```cpp
#ifdef HAVE_POW
    printf("Now we use the standard library. \n");
    double result = pow(base, exponent);
#else
    printf("Now we use our own Math library. \n");
    double result = power(base, exponent);
#endif
```

## 添加版本号

给项目添加和维护版本号是一个好习惯，这样有利于用户了解每个版本的维护情况，并及时了解当前所用的版本是否过时，或是否可能出现不兼容的情况。首先修改顶层 CMakeLists 文件，在 `project` 命令之后加入如下两行：

```cmake
set (Demo_VERSION_MAJOR 1)#主版本号
set (Demo_VERSION_MINOR 0)#主版本号
```

分别指定当前的项目的主版本号和副版本号。之后，为了在代码中获取版本信息，我们可以修改 [http://config.h.in](https://link.zhihu.com/?target=http%3A//config.h.in) 文件，添加两个预定义变量：

```cpp
// the configured options and settings for Tutorial
#define Demo_VERSION_MAJOR @Demo_VERSION_MAJOR@
#define Demo_VERSION_MINOR @Demo_VERSION_MINOR@
```

这样就可以直接在代码中打印版本信息了：

```cpp
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "config.h"
#include "math/MathFunctions.h"

int main(int argc, char *argv[])
{
    if (argc < 3){
        // print version info
        printf("%s Version %d.%d\n",
            argv[0],
            Demo_VERSION_MAJOR,
            Demo_VERSION_MINOR);
        printf("Usage: %s base exponent \n", argv[0]);
        return 1;
    }
    double base = atof(argv[1]);
    int exponent = atoi(argv[2]);
    
#if defined (HAVE_POW)
    printf("Now we use the standard library. \n");
    double result = pow(base, exponent);
#else
    printf("Now we use our own Math library. \n");
    double result = power(base, exponent);
#endif
    
    printf("%g ^ %d is %g\n", base, exponent, result);
    return 0;
}
```

## 生成安装包

本节将学习如何配置生成各种平台上的安装包，包括二进制安装包和源码安装包。为了完成这个任务，我们需要用到 CPack ，它同样也是由 CMake 提供的一个工具，专门用于打包。首先在顶层的 CMakeLists.txt 文件尾部添加下面几行：

```cmake
# 构建一个 CPack 安装包
include (InstallRequiredSystemLibraries)
set (CPACK_RESOURCE_FILE_LICENSE
  "${CMAKE_CURRENT_SOURCE_DIR}/License.txt")
set (CPACK_PACKAGE_VERSION_MAJOR "${Demo_VERSION_MAJOR}")
set (CPACK_PACKAGE_VERSION_MINOR "${Demo_VERSION_MINOR}")
include (CPack)
```

上面的代码做了以下几个工作：

1. 导入` InstallRequiredSystemLibraries `模块，以便之后导入 CPack 模块；
2. 设置一些 `CPack `相关变量，包括版权信息和版本信息，其中版本信息用了上一节定义的版本号；
3. 导入 `CPack `模块。

接下来的工作是像往常一样构建工程，并执行 `cpack` 命令。

- 生成二进制安装包：

```cmake
cpack -C CPackConfig.cmake
```

- 生成源码安装包

```cmake
cpack -C CPackSourceConfig.cmake
```

















# 多CMakeList编写

例子：

```shell
.
├── 3rdParty
│   └── caffe
│       └── include
│           └── caffe
│               └── common.hpp
├── cmake
│   └── Dependencies.cmake
├── CMakeLists.txt
├── include
│   └── Alg_VIR_Handheld_Video_Interface.h
├── source
│   ├── CMakeLists.txt
│   ├── common
│   │   ├── common.cpp
│   │   └── include
│   │       └── common.hpp
│   └── debug
│       ├── demonstrate
│       │   ├── demonstrate.cpp
│       │   └── include
│       │       └── demonstrate.hpp
│       └── redis_utils
│           ├── include
│           │   └── redis_utils.hpp
│           └── redis_utils.cpp
└── test
    ├── CMakeLists.txt
    └── redis.cpp
```







主CMakeLists.txt可以促使头文件全局有效，但source/Dependencies.cmake里面的头文件比如test/redis.cpp是不能直接引用，需要绝对路径，但是强制引用会把source/里面的引用搞得一塌糊涂，所以需要引用source 里面的头文件的内容，就直接include注册在主CMakeLists.txt里面。

主CMakeLists.txt


```cmake
#主CMakeLists.txt
cmake_minimum_required(VERSION 2.8)
 
SET(CMAKE_EXE_LINKER_FLAGS " -no-pie")
set(CMAKE_CXX_STANDARD 11)
 
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
 
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
 
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/source/common/include)
include_directories(${CMAKE_SOURCE_DIR}/source/debug/redis_utils/include)
 
# ---[ Dependencies
include(cmake/Dependencies.cmake)
 
add_subdirectory(source) 
add_subdirectory(test)
 
 
```





cmake/Dependencies.cmake里面主要负责3rdParty的引用，然后再连到主CMakeLists.txt，利用这句

```cmake
# ---[ Dependencies
include(cmake/Dependencies.cmake)
```

 

cmake/Dependencies.cmake内容，非常简单

```cmake
# protobuf
set(PROTOBUF_ROOT ${CMAKE_SOURCE_DIR}/3rdParty/protobuf)
include_directories(${PROTOBUF_ROOT}/include)
link_directories(${PROTOBUF_ROOT}/lib)
 
# cuda
set(CUDA_ROOT ${CMAKE_SOURCE_DIR}/3rdParty/cuda_100)
include_directories(${CUDA_ROOT}/include)
link_directories(${CUDA_ROOT}/lib)
link_directories(${CUDA_ROOT}/lib/stubs)
 
# opencv
set(OPENCV_ROOT ${CMAKE_SOURCE_DIR}/3rdParty/opencv-3.4.10)
include_directories(${OPENCV_ROOT}/include)
link_directories(${OPENCV_ROOT}/lib)
 
# caffe
set(CAFFE_ROOT ${CMAKE_SOURCE_DIR}/3rdParty/caffe_cuda100)
include_directories(${CAFFE_ROOT}/include)
link_directories(${CAFFE_ROOT}/lib)
```



```cmake
#source/Dependencies.cmake
#test/CMakeLists.txt
#利用这句代码连到工程
add_subdirectory(source) 
add_subdirectory(test)
```



```cmake
#test/CMakeLists.txt内容：包含一个可执行文件目标
# set the test link libs
SET(CMAKE_EXE_LINKER_FLAGS " -no-pie")
add_compile_options(-std=c++11)
 
#link_directories(${CMAKE_SOURCE_DIR}/3rdParty/redisLib)
 
add_executable(redis redis.cpp)
target_link_libraries(redis handheld_video libavv_alg_redis.a libhiredis.a)
 
 
 
add_executable(testModel testModel.cpp)
target_link_libraries(testModel handheld_video)
```



```cmake
#source/Dependencies.cmake：包含一个编译的算法库，和另一个可执行文件目标（注因为主函数被写在source里面，本例忘记把文件在上文中加入了，还望海涵）
cmake_minimum_required(VERSION 3.5.0)
project(handheld_video)
SET(CMAKE_EXE_LINKER_FLAGS " -no-pie")
# set the common link libs
list(APPEND HANDHELD_VIDEO_LINK_LIBS  cuda)
list(APPEND HANDHELD_VIDEO_LINK_LIBS  nvrtc)
list(APPEND HANDHELD_VIDEO_LINK_LIBS  boost_date_time)
 
 
 
 
aux_source_directory(${CMAKE_SOURCE_DIR}/source/common common_src)
aux_source_directory(${CMAKE_SOURCE_DIR}/source/debug/redis_utils redis_utils_src)
aux_source_directory(${CMAKE_SOURCE_DIR}/source/debug/demonstrate demonstrate_src)
 
 
 
add_library(handheld_video SHARED
        ${common_src}
        ${redis_utils_src}
        ${demonstrate_src}
        ../include/Alg_VIR_Handheld_Video_Interface.h ../source/api/Alg_VIR_Handheld_Video_Interface.cpp)
 
target_link_libraries(handheld_video ${TORCH_LIBRARIES} ${HANDHELD_VIDEO_LINK_LIBS})
 
add_executable(redisBaoDing app/redis/redis.cpp)
target_link_libraries(redisBaoDing handheld_video )
```

# CMake使用

## `find_package`

### Motivation

经常在Linux下面写C++程序，尤其是需要集成各种第三方库的工程，肯定对`find_package`指令不陌生。

这是条很强大的指令。可以直接帮我们解决整个工程的依赖问题，自动把头文件和动态链接文件配置好。比如说，在Linux下面工程依赖了`OpenCV`，只需要下面几行就可以完全配置好：

```cmake
add_executable(my_bin src/my_bin.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(my_bin, ${OpenCV_LIBS})
```

工作流程如下：

1. `find_package`在一些目录中查找OpenCV的配置文件。
2. 找到后，`find_package`会将头文件目录设置到`${OpenCV_INCLUDE_DIRS}`中，将链接库设置到`${OpenCV_LIBS}`中。
3. 设置可执行文件的链接库和头文件目录，编译文件。

到现在为止出现了第一个问题。那就是：
**find_package会在哪些目录下面寻找OpenCV的配置文件？**

### `find_package`目录

为什么我们要知道这个问题呢？因为很多库，我们都是自己编译安装的。比如说，电脑中同时编译了`OpenCV2`和`OpenCV3`，我该如何让cmake知道到底找哪个呢？

其实这个问题在CMake官方文档中有非常详细的解答。

首先是查找路径的根目录。

```cmake
<package>_DIR
CMAKE_PREFIX_PATH
CMAKE_FRAMEWORK_PATH
CMAKE_APPBUNDLE_PATH
PATH
```

> `PATH`中的路径如果以`bin`或`sbin`结尾，则自动回退到上一级目录。

找到根目录后，cmake会检查这些目录下的

```cmake
<prefix>/(lib/<arch>|lib|share)/cmake/<name>*/          (U)
<prefix>/(lib/<arch>|lib|share)/<name>*/                (U)
<prefix>/(lib/<arch>|lib|share)/<name>*/(cmake|CMake)/  (U)
```

cmake找到这些目录后，会开始依次找`<package>Config.cmake`或`Find<package>.cmake`文件。找到后即可执行该文件并生成相关链接信息。

现在回过头来看查找路径的根目录。最重要的一个是`PATH`。由于`/usr/bin/`在`PATH`中，cmake会自动去`/usr/(lib/<arch>|lib|share)/cmake/<name>*/`寻找模块，这使得绝大部分我们直接通过`apt-get`安装的库可以被找到。

另外一个比较重要的是`<package>_DIR`。我们可以在调用cmake时将这个目录传给cmake。由于其优先级最高，因此cmake会优先从该目录中寻找，这样我们就可以随心所欲的配置cmake使其找到我们希望它要找到的包。而且除上述指定路径外，cmake还会直接进入`<package>_DIR`下寻找。如我在`3rd_parties`目录下编译了一个`OpenCV`，那么执行cmake时可以使用

```cmake
OpenCV_DIR=../../3rd-party/opencv-3.3.4/build/ cmake .. 
```

配置好编译好了以后，我感兴趣的是另一个问题：
我现在编译出了可执行文件，并且这个可执行文件依赖于`opencv`里的动态库。这个动态库是在cmake时显式给出的。那么，

1. 该执行文件在运行时是如何找到这个动态库的？
2. 如果我把可执行文件移动了，如何让这个可执行文件依然能找到动态库？
3. 如果我把该动态库位置移动了，如何让这个可执行文件依然能找到动态库？
4. 如果我把可执行文件复制到别的电脑上使用，我该把其链接的动态库放到新电脑的什么位置？

### 可执行文件如何寻找动态库

> The linker uses the following search paths to locate required
> shared libraries:
>
> 1. Any directories specified by -rpath-link options.
>
> 2. Any directories specified by -rpath options. The difference
>    between -rpath and -rpath-link is that directories specified by
>    -rpath options are included in the executable and used at
>    runtime, whereas the -rpath-link option is only effective at
>    link time. Searching -rpath in this way is only supported by
>    native linkers and cross linkers which have been configured
>    with the --with-sysroot option.
>
> 3. On an ELF system, for native linkers, if the -rpath and
>    -rpath-link options were not used, search the contents of the
>    environment variable "LD_RUN_PATH".
>
> 4. On SunOS, if the -rpath option was not used, search any
>    directories specified using -L options.
>
> 5. For a native linker, the search the contents of the environment
>    variable "LD_LIBRARY_PATH".
>
> 6. For a native ELF linker, the directories in "DT_RUNPATH" or
>    "DT_RPATH" of a shared library are searched for shared
>    libraries needed by it. The "DT_RPATH" entries are ignored if
>    "DT_RUNPATH" entries exist.
>
> 7. The default directories, normally /lib and /usr/lib.
>
> 8. For a native linker on an ELF system, if the file
>    /etc/ld.so.conf exists, the list of directories found in that
>    file.
>
> If the required shared library is not found, the linker will issue
> a warning and continue with the link.

最重要的是第一条，即`rpath`。这个`rpath`会在编译时将动态库绝对路径或者相对路径（取决于该动态库的cmake）写到可执行文件中。`chrpath`工具可以查看这些路径。

```shell
>>> chrpath extract_gpu
extract_gpu: RPATH=/usr/local/cuda/lib64:/home/dechao_meng/data/github/temporal-segment-networks/3rd-party/opencv-3.4.4/build/lib
```

可以看到，OpenCV的动态库的绝对路径被写到了可执行文件中。因此即使可执行文件的位置发生移动，依然可以准确找到编译时的`rpath`。

接下来的问题：如果我把可执行文件复制到了别人的电脑上，或者我的动态库文件的目录发生了改变，怎样让可执行文件继续找到这个动态库呢？其实是在第五条：`LD_LIBRARY_PATH`。只要将存储动态库的目录加入到`LD_LIBRARY_PATH`中，可执行文件就能正确找到该目录。

这种做法十分常见，比如我们在安装`CUDA`时，最后一步是在`.bashrc`中配置

```shell
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

这样做之后，依赖`cuda`的可执行文件就能够正常运行了。
