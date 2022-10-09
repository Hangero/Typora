# Shell脚本

## Shell

### 什么是shell

jjj网上有很多shell 的概念介绍，其实都很官方化，如果你对linux 命令很熟悉，那么编写shell 就不是一个难事，shell 本质上是 linux 命令，一条一条命令组合在一起，实现某一个目的，就变成了shell脚本。它从一定程度上 减轻了工作量，提高了工作效率。

### 官方shell 介绍

Shell 通过提示您输入，向操作系统解释该输入，然后处理来自操作系统的任何结果输出，简单来说Shell就是一个用户跟操作系统之间的一个命令解释器。

### 常见的shell

	Bourne Shell（/usr/bin/sh或/bin/sh）
	Bourne Again Shell（/bin/bash）
	C Shell（/usr/bin/csh）
	K Shell（/usr/bin/ksh）
	Shell for Root（/sbin/sh）

 最常用的shell是Bash，也就是Bourne Again Shell。Bash由于易用和免费，在日常工作中被广泛使用，也是大多数Linux操作系统默认的Shell环境。

## shell编程注意事项

1. 命名：后缀为.sh
2. 不能使用特殊符号，空格
3. 首行要以`#!/bin/bash`开头

## Hello World

### 1. #!/bin/bash

#!用来声明脚本由什么shell解释，否则使用默认shell

```shell
ls   /bin | grep sh  #查找shell解释器
```

### 2. echo  "hello world"

```shell
touch Helloworld.sh
vim Helloworld.sh ## i :进入编辑模式  ；esc：回到一般模式  ；:wq：保存并退出
```



### 3.加上可执行权限

```shell
ll Helloworld.sh  #查看权限
chmod +x  Helloword.sh #添加可执行权限
```

### 4. 运行

- ./xxx.sh

  先按照文件中#!指定的解析器解析

  如果#！指定指定的解析器不存在 才会使用系统默认的解析器

- bash xxx.sh

  指明先用bash解析器解析

  如果bash不存在 才会使用默认解析器

- . xxx.sh

  直接使用默认解析器解析（不会执行第一行的#！指定的解析器）但是第一行还是要写的

## 变量

### 系统变量

```shell
# Shell常见的变量之一系统变量，主要是用于对参数判断和命令返回值判断时使用，系统变量详解如下：
$        #引用变量
$0 		#当前脚本的名称
$n 		#当前脚本的第n个参数,n=1,2,…9
$* 		#当前脚本的所有参数(不包括程序本身)
$# 		#当前脚本的参数个数(不包括程序本身)
$? 		#令或程序执行完后的状态，返回0表示执行成功
$$ 		#程序本身的PID号
```

### 环境变量

#### 0

```shell
unset    #清除变量值
read      #从键盘读取；-p：在上一行显示和添加提示
readonly #只读变量
env       #查看环境变量
source filename #导出环境变量，让其他shell脚本识别该变量，设为全局变量；与. filenaem等效
export PATH=$PATH:/需要添加的路径  #在PATH变量中 追加一个路径
declare  #声名shell变量，设置变量属性。也可以写成typeset。declare -i s 表示u强制把s变量当成int型参数运算。
```

<h4 id="1">脚本标量的特殊用法</h4>

```shell
""   #双引号，包含的变量会被解释
''     #单引号，包含的变量被当成字符串处理
``  #反引号，包含内容作为系统命令，并执行其内容，可替换输出为一个变量
\      #转义字符，同c语言中的\n \t \r \a 等    echo命令中需要-e转义
(命令序列)   #由子shell来完成，不影响当前shell中的变量
{命令序列}   #由当前shell执行，会影响当前变量
```



#### 1. 系统环境变量

```shell
#Shell常见的变量之二环境变量，主要是在程序运行时需要设置，环境变量详解如下：

PATH  		#命令所示路径，以冒号为分割；
HOME  	  #打印用户家目录；
SHELL 	  #显示当前Shell类型；
USER  	   #打印当前用户名；
ID    		    #打印当前用户id信息；
PWD   		#显示当前所在路径；
TERM  		#打印当前终端类型；
HOSTNAME   # 显示当前主机名；
PS1         #定义主机命令提示符的；
HISTSIZE    #历史命令大小，可通过 HISTTIMEFORMAT 变量设置命令执行时间;
RANDOM      #随机生成一个 0 至 32767 的整数;
HOSTNAME    #主机名
```

#### 2. 用户环境变量

- 用户自定义变量

  ```shell
  # 常见的变量之三用户变量，用户变量又称为局部变量，主要用在Shell脚本内部或者临时局部使用，系统变量详解如下：
  a=rivers 				                               # 自定义变量A；
  Httpd_sort=httpd-2.4.6-97.tar  #自定义变量N_SOFT；
  BACK_DIR=/data/backup/          #自定义变量BACK_DIR；
  IPaddress=10.0.0.1			              #自定义变量IP1；
  ```

#### 3. 变量的拓展

##### 3.1 判断变量是否存在

```shell
${num:-val} #如果num存在，表达式的值为num；否则为val
${num:=val} #如果num存在，表达式的值为num；否则为val，同时将val的值赋值给num
```

##### 3.2 字符串操作

![](/home/suyu/桌面/Typora/image/Shell脚本/20200420220230876.png)

#### 4.条件测试

test命令：用于测试字符串、文件状态和数字

两种格式：

- test conditon
- [ condition ]    **注意两边加上空格**

1. 文件测试

```shell
-e   # 是否存在
-d  #是目录
-f   #是文件
-r   #可读
-w  #可写
-x    #可执行
-L   #符号连接
-c    #是否字符设备
-b   #是否块设备 
-s   #文件非空
```

2. 字符串测试

test    str_operator    "str"          或      [ str_operator "str" ]

test  "str1"   str_operator  "str2"  或  [ "str1"    str_operator    "str2" ]

```shell
=        #两个字符串相等 
!=      #两个字符串不相等
-z      #空串
-n     #非空串
```

3. 数值测试

test num1 num_operator num2   或  [ num1 num_operator num2 ]

```shell
-eq		 #等于，应用于整型比较 equal；                               equal
-ne		 #不等于，应用于整型比较 not equal；                   not equal
-lt		   #小于，应用于整型比较 letter；                                greater than
-gt		  #大于，应用于整型比较 greater；                            less  than
-le		  #小于或等于，应用于整型比较；                               less equal
-ge      #大于或等于，应用于整型比较；                               greater equal
```

4. 符合语句测试

命令执行控制：

- && 

  command1 && command2

  &&左边命令（command1）**执行成功**（即返回0）shell才执行&&右边的命令（command2）

- ||

  command1 || command2

  ||左边命令（command1）**未0执行成功**（即返回0）shell才执行||右边的命令（command2）

```shell
-a      #(and) 两种情况均成立
-o      #(or)    两种情况任何一个成立
!         #相反状态
```



## Shell编程流程控制语句

### if条件语句

```shell
# If条件判断语句，通常以if开头，fi结尾。也可加入else或者elif进行多条件的判断

# 单分支语句 ---比较大小
	if [ 条件 ];then
		语句1
	fi

# 双分支if 语句
    if [ 条件1 ]; then
		语句1
	else
		语句2
	fi

# 多支条件语句 ---判断成绩
	if [ 条件1 ]; then
		语句1
	elif [ 条件2 ]; then
		语句2
    else
		语句2
	fi  
```

### case条件语句

```shell
#Case选择语句，主要用于对多个选择条件进行匹配输出，与if elif语句结构类似，通常用于脚本传递输入参数，打印出输出结果及内容，其语法格式以Case…in开头，esac结尾。语法格式如下：
case $变量名称  in
  内容1)
    命令
    ;;       #break
  内容2)
    命令
    ;;
*)
不符合以上模式执行的命令
esac
# 每个模式必须以右括号结束，命令结尾以双分号结束。

```

### for循环语句

```shell
形式一
  for 变量名 in 取值列表; 
          do
                          语句 1
          done
形式二
   for ((初始值；限制值；执行步阶))
          do
                           程序段
          done
```

### while循环语句

```shell
while  [ condition ]     #满足条件循环，不满足则退出循环
do
        程序
done

until [ condition ]       #不满足条件循环，满足时退出
do 
         程序
done
```

### break 和continue

```shell
# break 和 continue 语句
  break 是终止循环。
  continue 是跳出当前循环。
#示例 1：在死循环中，满足条件终止循环
while true; do
  let N++
  if [ $N -eq 5 ]; then
    break
fi
  echo $N
done
输出： 1 2 3 4

#示例 2：举例子说明 continue 用法
N=0
while [ $N -lt 5 ]; do
  let N++
if [ $N -eq 3 ]; then
  continue
fi
  echo $N
done

输出： 1 2 4

# 打印 1-100 数字
i=0
while ((i<=100))
do
        echo  $i
        i=`expr $i + 1`
done
```

### selet选择语句



```shell
#select 是一个类似于 for 循环的语句
#Select语句一般用于选择，常用于选择菜单的创建，可以配合PS3来做打印菜单的输出信息，其语法格式以select…in do开头，done结尾：

select i in （表达式） 
do
语句
done

# 选择mysql 版本
#!/bin/bash
# by author rivers on 2021-9-27
PS3="Select a number: "
while true; do
select mysql_version in 5.1 5.6 quit;
 do
  case $mysql_version in
  5.1)
    echo "mysql 5.1"
      break
      ;;
  5.6)
    echo "mysql 5.6"
       break
       ;;
  quit)
    exit
    ;;
  *)
    echo "Input error, Please enter again!"
      break
esac
 done
done

```





### 函数

```shell
#所有函数在使用前必须定义，必须将函数放在脚本开始部分，直至shell解释器首次发现它时，才可以使用

#格式一
函数名() {
      command1
      command2
       ……
}
#格式二
function 函数名() {
      command1
      command2
      ……
}
#函数调用
函数名   param1 param2 ......
#函数可以使用return提前结束并带回返回值
return
return 0  无错误返回
return 1  有错误返回

source fun.sh   #导入函数

```

## 实践

### 查找并杀死进程

[解析](https://blog.csdn.net/weixin_43951166/article/details/121245296?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165776486716781435475034%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=165776486716781435475034&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-2-121245296-null-null.142^v32^experiment_2_v1,185^v2^control&utm_term=ps%20-ef%20%7C%20grep%20%24NAME%20%7C%20grep%20-v%20%240%20%7C%20grep%20-v%20grep%20%7C%20awk%20%7Bprint%20%242%7D&spm=1018.2226.3001.4187)  

```shell
# 杀死helloworld进程
#!/bin/bash

#查看后台，并写入result.txt
top -n 1 > result.txt
echo "已将所有进程信息写入result.txt"

#搜索PID
PID=`ps -ef | grep helloworld|grep -v grep|awk '{print $2}'`#'{print $2}'外面是单引号，整个外面是反引号
echo "已获得进程PID"

#杀死进程
kill $PID
echo "已杀死helloworld进程" 

# 杀死任意进程
#!/bin/sh

NAME=$1
echo $NAME
ID=`ps -ef | grep "$NAME" | grep -v "$0" | grep -v "grep" | awk '{print $2}'`
echo $ID
echo "---------------"
for id in $ID
do
kill -9 $id
echo "killed $id"
done
echo "---------------"
```

### 修改文件后缀

```shell
#!/bin/bash
ALL_SH_FILE=$(find . -type f -name "*.sh")

for file in ${ALL_SH_FILE[*]}
do
        filename=$(echo $file | awk -F'.sh' '{print $1}')
        new_filename="${filename}.shell"
        mv "$file" "$new_filename"
        sed -i '2d' "$new_filename"
done


```

