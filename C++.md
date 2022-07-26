# C++

*参考*

*[黑马程序员](https://www.bilibili.com/video/BV1et411b73Z?p=42&vd_source=e77e5355b23e2c6445aa275d3b71942b)*

## 0.预备知识

### [ELF文件](https://blog.csdn.net/daide2012/article/details/73065204?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165820146616782246458941%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165820146616782246458941&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-73065204-null-null.142^v32^experiment_2_v1,185^v2^control&utm_term=elf%E6%96%87%E4%BB%B6&spm=1018.2226.3001.4187)

## 1.代码规范

>Always code as if the guy who ends up maintaining your code will be a violent psychopath who knows where you live.
>
>​                                                                                                                                   ——John F.Woods,1991

### 1.1#pragma

C/C++中，在使用[预编译](https://so.csdn.net/so/search?q=预编译&spm=1001.2101.3001.7020)指令**#include**的时候，为了防止重复引用造成二义性。

pragma once则由编译器提供保证：同一个文件不会被编译多次。注意这里所说的“同一个文件”是指物理上的一个文件，而不是指内容相同的两个文件。带来的好处是，你不必再费劲想个宏名了，当然也就不会出现宏名碰撞引发的奇怪问题。对应的缺点就是如果某个头文件有多份拷贝，本方法不能保证他们不被重复包含。当然，相比宏名碰撞引发的“找不到声明”的问题，重复包含更容易被发现并修正。

## 2.数组

特点：

- 放在一块连续的内存空间中
- 数组中每一个元素都是相同的数据类型

### 2.1 一维数组的创建和初始化

#### (1)  数组的创建

```cpp
type_t   arr_name   [const_n];
//type_t 是指数组的数据类型
//const_n 是一个常量表达式，用来指定数组的长度

type_t   arr_name   [const_n] = {v1, v2 ...}
//v1,v2代表具体值

type_t   arr_name   [ ] = {v1, v2 ...}
//长度会自动推测
```

#### (2) 数组的初始化

- 数组只能够整体初始化，不能被整体赋值。只能使用循环从第一个逐个遍历赋值。
- 初始化时，数组的维度或元素个数可忽略 ，编译器会根据花括号中元素个数初始化数组元素的个数。
- 当花括号中用于初始化值的个数不足数组元素大小时，数组剩下的元素依次用0初始化。
- 字符型数组在计算机内部用的时对应的ascii码值进行存储的。
  一般用”“引起的字符串，不用数组保存时，一般都被直接编译到字符常量区，并且不可被修改。

### 2.2 一维数组的数组命名

用途：

- 可以统计整个数组在内存中的长度
- 可以获取数组在内存中的首地址

```cpp
sizeof(arr_name)//统计数组长度 Count array length
    
sizeof(arr[0])//统计每个元素所占内存空间 Count the memory space occupied by each element
```

```cpp
cout <<arr_name<<endl;//获得数组在内存中的首地址 Get the first address of the array in memory
```

### 实例

```cpp
//最大值

int arr[5] = ( 1, 2, 3, 5, 4);
int max = 0 ;
for (int i  = 0 ; i < 5; i++)
{
     if ( arr[i]>max)
     {
      max = arr[i];  
     }
}
```

```cpp
//元素逆置

int arr[5] = ( 1, 2, 3, 4, 5);
int start = 0l;
int end = sizeof(arr)/sizeof(arr[0])-1;
 
while(start<end)
{
    int temp =arr[start];
    arr[start]=arr[end];
    arr[end] = temp;
    start++;
    end--; 
}
```

### 冒泡排序

