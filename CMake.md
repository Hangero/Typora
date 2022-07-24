# CMake Practice

## 特点

1. 开放源代码,使用类 BSD 许可发布

2. 跨平台,并可生成 native 编译配置文件,在 Linux/Unix 平台,生成 makefile,在
   苹果平台,可以生成 xcode,在 Windows 平台,可以生成 MSVC 的工程文件。

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

- SEND_ERROR：产生错误,生成过程被跳过。
- SATUS ：输出前缀为 — 的信息。
- FATAL_ERROR：立即终止所有 cmake 过程。

我们在这里使用的是 STATUS 信息输出,演示了由 PROJECT 指令定义的两个隐式变量
HELLO_BINARY_DIR 和 HELLO_SOURCE_DIR。

### add_executable(hello ${SRC_LIST})

表示最终要生成的elf文件的名字叫hello，使用的源文件是${SRC_LIST}

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

   (..代表父目录，因为父目录存在我们需要的CMakeLists.txt；**如果你在其他地方建立了 build 目录，需要运行 `cmake <CMakeList_path>`，即去寻找CMakeLists.txt**)，查看一下 build 目录，就会发现了生成了编译需要的 Makefile 以及其他的中间文件。

4. 运行 make 构建工程,就会在当前目录(build 目录)中获得目标文件 hello。

## 2

```cmake
#为工程添加一个子目录 src,用来放置工程源代码;
#添加一个子目录 doc,用来放置这个工程的文档 hello.txt
#在工程目录添加文本文件 COPYRIGHT, README;
#在工程目录添加一个 runhello.sh 脚本,用来调用 hello 二进制
#将构建后的目标文件放入构建目录的 bin 子目录;
```

最外层的CMakeLists.txt用于掌控全局，使用add_subdirectory来控制其它目录下的CMakeLists.txt的运行。

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

把握一个简单的原则：在哪里`ADD_EXECUTABLE` 或 `ADD_LIBRARY`，如果需要改变目标存放路径,就在哪里加入上述的定义。

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
| DESTINATION | 定义安装路径，安装后的路径就是${CMAKE_INSTALL_PREFIX}/<DESTINATION 定义的路径> |

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

**DIRECTORY **后面连接的是所在 Source 目录的相对路径,但务必注意:
abc 和 abc/有很大的区别:如果目录名不以/结尾,那么这个目录将被安装为目标路径下的 abc,如果目录名以/结尾,代表将这个目录中的内容安装到目标路径,但不包括这个目录本身。

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

- SCRIPT 参数用于在安装时调用 cmake 脚本文件(也就是<abc>.cmake 文件)
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







