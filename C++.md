# C++

*参考*

*[黑马程序员](https://www.bilibili.com/video/BV1et411b73Z?p=42&vd_source=e77e5355b23e2c6445aa275d3b71942b)*

## 0. 预备知识

### [ELF文件](https://blog.csdn.net/daide2012/article/details/73065204?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165820146616782246458941%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165820146616782246458941&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-73065204-null-null.142^v32^experiment_2_v1,185^v2^control&utm_term=elf%E6%96%87%E4%BB%B6&spm=1018.2226.3001.4187)

## 1. 代码规范

>Always code as if the guy who ends up maintaining your code will be a violent psychopath who knows where you live.
>
>​                                                                                                                                   ——John F.Woods,1991

### 1.1#pragma

C/C++中，在使用[预编译](https://so.csdn.net/so/search?q=预编译&spm=1001.2101.3001.7020)指令**#include**的时候，为了防止重复引用造成二义性。

pragma once则由编译器提供保证：同一个文件不会被编译多次。注意这里所说的“同一个文件”是指物理上的一个文件，而不是指内容相同的两个文件。带来的好处是，你不必再费劲想个宏名了，当然也就不会出现宏名碰撞引发的奇怪问题。对应的缺点就是如果某个头文件有多份拷贝，本方法不能保证他们不被重复包含。当然，相比宏名碰撞引发的“找不到声明”的问题，重复包含更容易被发现并修正。

## 2. 数组

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

### 2.3 二维数组的创建和初始化

```cpp
type_t   arr_name   [row_n][column_n];

type_t   arr_name   [row_n][column_n] ={ {数据1，数据2}，{数据3， 数据4}};

type_t   arr_name   [row_n][column_n] =  {数据1，数据2，数据3， 数据4};

type_t   arr_name   [ ][column_n] =  {数据1，数据2，数据3， 数据4};

```

> 第二种更直观，可读性更高。
>
> ```cpp
> int arr[2][3] =
> {
> 		{1,2,3}.
> 		{4,5,6}
> };
> ```
>
> 

|      | 0          | 1          | 2          |
| ---- | ---------- | ---------- | ---------- |
| 0    | arr[0]\[0] | arr[0]\[1] | arr[0]\[2] |
| 1    | arr[1]\[0] | arr[1]\[1] | arr[1]\[2] |

```cpp
//外层循环打印行数，内层循环打印列数。

for (int i = 0 ; i <2 ; i++)
{
    for (int j =0 ; j<3; j++)
    {
        cout << arr[2][j]<<endl;
    }
};

//用矩阵的形式呈现
for (int i = 0 ; i <2 ; i++)
{
    for (int j =0 ; j<3; j++)
    {
        cout << arr[2][j]<<"  ";
    }
    cout<<endlo;
};
```



###  二维数组的数组名

- 参看数组所占内存空间
- 获得二维数组首地址

```cpp
cout<< sizeof(arr)/sizeof(arr[0])<<endl;//获得行数Get the number of rows

cout<<sizeof(arr[0])/sizeof(zrr[0][0])<<endl;//获得列数 Get the number of columns
```

```cpp
cout<<(long)arr<<endl;//数组首地址Array first address
cout<<(long)arr[0]<<endl;//第一行首地址Address at the beginning of the first line
cout<<(long)arr[1]<<endl;//第二行首地址
cout<<(long)&arr[0][0]<<endl;//查看元素首地址要用取址符&
```

### 实例

|          | 语文 | 数学 | 英语 |
| -------- | ---- | ---- | ---- |
| **张三** | 100  | 100  | 100  |
| **李四** | 90   | 50   | 100  |
| **王五** | 60   | 70   | 80   |



```cpp
int score[3][3] =
{
    {100, 100 , 100};
    {90  ,   50  , 100};
    {60  ,   70  ,  80 }
}

for (int i=0; i<3 ; i++)
{
    int sum =0 ;
    for (int j = 0 ; i<3 ;j++)
    {
        sum +=score[i][j];
        cout << score[i][j]<<"\t";
    }
    cout<<endl;
}
```





### 数组与矩阵

#### 加减

$$
A\in M_{n\times m}(\mathbb C)  , B\in M_{n\times m}(\mathbb C)\\
\\
A\pm B=C,C\in M_{n\times m}(\mathbb C)\\
C_{ij}= A_{ij}\pm B_{ij}
$$

#### 乘法

$$
A\in M_{n\times r}(\mathbb C)  , B\in M_{r\times m}(\mathbb C),C\in M_{m \times n}(\mathbb C)\\
\\
A \times B = C\\
C_{ij}=\sum_{k=1}^r(A_{ik}\times B_{kj});\forall 1\le i \le m, 1 \le j\le n
$$

```c

#include <stdio.h>
 
int main(void)
{
	int mTemp, rTemp, nTemp;
	scanf("%d %d %d", &mTemp, &rTemp, &nTemp);//格式化为整数
	const int m = mTemp, r = rTemp, n = nTemp;	//注意数组的声明要用常数
	double A[m][r], B[r][n], C[m][n];
 
	//输入A，B两个矩阵
	for (int i = 0; i <= m - 1; i++) {
		for (int j = 0; j <= r - 1; j++) {
			scanf("%lf", &A[i][j]);//%10.5,表示每列数据长度10位，其中5位小数，同一行数据用逗号分隔
			}
		}
	for (int i = 0; i <= r - 1; i++) {
		for (int j = 0; j <= n - 1; j++) {
			scanf("%lf", &B[i][j]);
			}
		}
 
	//将矩阵（二维数组）C 的所有元素全部初始化为零
	for (int i = 0; i <= m - 1; i++) {
		for (int j = 0; j <= n - 1; j++) {
			C[i][j] = 0;
			}
		}
 
	//矩阵乘法
	for (int i = 0; i <= m - 1; i++) {
		for (int j = 0; j <= n - 1; j++) {
			for (int k = 0; k <= r - 1; k++) {
				C[i][j] += A[i][k] * B[k][j];
				}
			}
		}
	
	//输出结果
	for (int i = 0; i <= m - 1; i++) {
		for (int j = 0; j <= n - 1; j++) {
			printf("%10.5f ", C[i][j]);
			}
			printf("\n");
		}
 
	return 0;
}
```

#### 转置

$$
A_{(i,j)}^T=A_{(j,i)}
$$

```c
for(i=0;i<m;i++){
    for(j=0;j<n;j++){
        arrB[i][j]=arrA[i][j];
    }
}
```

#### 稀疏矩阵

在矩阵中，若数值为0的元素数目远远多于非0元素的数目，并且非0元素分布没有规律时，则称该矩阵为**稀疏矩阵**；与之相反，若非0元素数目占大多数时，则称该矩阵为**稠密矩阵**。定义非零元素的总数比上矩阵所有元素的总数为矩阵的稠密度。

## 3. 函数

> 将一段重用代码进行封装

### 3.1  函数的定义

```cpp
返回值类型   函数名   （参数列表）//形参
{
                    函数体语句
                    return 表达式
}
```

### 3.2  函数的调用

```cpp
int add (int num1 ,int num2)
{
		return num1+num2;
}

int main (){
		int a = 10;
		int b = 20;
		int c = add(a,b);//调用函数时，实参的值会传递给形参
}
```

### 3.3  值传递

- 所谓值传递，就是函数调用时实参将数值传递给形参
- 值传递时，如果形参发生改变，不会影响实参

### 3.4  常见样式

- 无参无返
- 有参无返
- 无参有返
- 有参有返

```cpp
  // 无参无返
void test01()
{
    cout<<"this is test01"<<endl;
}

//有参无返
 void test02(int a)
 {
     cout << "this is test02 a= "<<a <<endl;
 }

//无参有返
int test03()
{
    cout<<"this is test03"<<endl;
    return  1000;
}

//有参有返
 int test04(int a)
 {
     cout<<"this is test04 a ="<< a << endl
         return a;
 }

```

### 3.5  函数声明

```cpp
int max (int a, int b);//函数声明
int main ()
{

};
int max ( int a, int b)
{

};
```

### 3.6  函数的分文件编写

作用：让代码结构更清晰

1. 创建后缀名为.h的头文件
2. 创建后缀名为.cpp的源文件
3. 在头文件中写函数的声明
4. 在源文件中写函数的定义

[C++函数分文件编写(VScode2021配置教程)](https://blog.csdn.net/mia0303/article/details/116521747?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165889061516782388029546%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165889061516782388029546&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-116521747-null-null.142^v35^experiment_28w_180w_v1,185^v2^control&utm_term=vscode%E5%87%BD%E6%95%B0%E5%88%86%E6%96%87%E4%BB%B6%E7%BC%96%E5%86%99&spm=1018.2226.3001.4187)



## 4.  指针

作用：可以通过指针间接访问内存

- 内存编号是从0开始记录的，一般用十六进制数字表示
- 可以利用指针变量保存地址（**指针即地址**）

### 4.1  指针变量的定义和使用

```cpp
//变量类型  * 变量名
int a= 10;
int * p ;
p= &a; 

//可以通过指针前加 * 来解引用，找到指针指向的内存中的数据

```

### 4.2  指针所占内存空间

```cpp
cout << sizeof (int *)<<endl;
cout << sizeof(float *)<<endl;
```

### 4.3  空指针和野指针

空指针：指针变量指向内存中编号为0的空间

用途：初始化指针

```cpp
int *p = NULL;
//0~255之间的内存编号是系统占用内存，不允许用户访问
```

野指针：指针变量指向非法的内存空间

```cpp
  int * p = (int *)0x1100;//尽量避免野指针
```

### 4.4  const修饰指针

- const 修饰指针--常量指针 ： 指针的指向可以修改，但修改
- const 修饰常量--指针常量  ：指针的指向不可以改，指针指向的值可以改
- const 既修饰指针，又修饰常量 ：都不可以改

```cpp
const int * p = &a;//Const decorated constant

int * const p = &a;//Const decorated pointer

const int * const p = &a;
```

### 4.5  指针和数组

作用：利用指针访问数组中的元素

```cpp
int arr[0] = {1,2,3,4,5,6,7,8,9,10}
int * p = arr;//arr就是数组首地址 Arr is the first address of the array
p++;//让指针向后偏移4个字节
```

### 4.6  指针和函数

作用：利用指针作函数参数，可以修改实参的值

#### 值传递



#### 地址传递

```cpp
viod swap(int * p1, int *p2)
{
    int temp = *p1;
    *p1  = *p2;
    *p2 = temp;
}
swap(&a,&b)
```

## 5.  结构体

结构体用于用户自定义的数据类型，允许用户存储不同的数据类型。

> 内置类型组成的集合

### 5.1  结构体的定义和使用

```cpp
struct 结构体名  {结构体成员列表}

struct 结构体名  变量名
struct 结构体名  变量名 = {成员1值，成员2值}
```

```cpp
//创建结构体
struct Student
{
    string name;
    int age;
    int score;
}
 
//创建具体学生
//struct的关键字可以省略
//第一种
struct Student s1;
s1.name="张三"；//注意要包含string头文件
s1.age=18;
s1.score = 100;
    
//第二种
struct Student s2={"张三",18,19};

//第三种

```



### 5.2  结构体数组

作用：将自定义的结构体放入数组中方便维护

```cpp
struct  结构体名   数组名 [元素个数] = { {}, {},...{}}
```

1. 定义结构体
2. 创建结构体数组
3. 给结构体数组中的元素赋值
4. 遍历结构体数组

```cpp
struct Student
{
    string name;
    int age;
    int score;
}

struct Student stuArray[3]=
{
    {"张三",18,19};
    {"张三",18,19};
    {"张三",18,19};
}

stuArray[2].name ="赵六"
```

### 5.3  结构体指针

作用：通过指针访问结构体中的成员。

- 利用操作符`->`可以通过结构体指针访问结构体属性

```cpp
struct student
{
    string name;
    int age;
    int score;
}
int main ()
{
    //创建学生结构体变量
    struct student  s =    {"张三",18,19};
    //通过指针指向结构体变量
    student * p = &s;
    //通过指针访问结构体变量中的数据
    cout<<p->name;
    
}
 
```

### 5.4  结构体嵌套结构

作用：结构体中的成员可以是另一个结构体

### 5.5 结构体做函数参数

作用：将结构体作为参数向函数中传递

- 值传递
- 地址传递

```cpp
struct student
{
    string name;
    int age;
    int score;
}
//值传递
void printstudent1(struct student s)
{
    cout<<s.age;
}

//地址传递
void printstudent2(struct student * s)
{
    cout<<p->name;
}
```

### 5.6  结构体中const使用场景

作用：防止误操作

> 将函数中的形参改成指针，可以减少内存空间，而且不会复制出新的副本

```cpp
void printstudent2(const struct student * s)
{
    cout<<p->name;
}
```







































































## 排序算法



