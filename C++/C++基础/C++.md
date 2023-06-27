# C++

*参考*

*[黑马程序员](https://www.bilibili.com/video/BV1et411b73Z?p=42&vd_source=e77e5355b23e2c6445aa275d3b71942b)*

## 0. 预备知识

### [ELF文件](https://blog.csdn.net/daide2012/article/details/73065204?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165820146616782246458941%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165820146616782246458941&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-73065204-null-null.142^v32^experiment_2_v1,185^v2^control&utm_term=elf%E6%96%87%E4%BB%B6&spm=1018.2226.3001.4187)

(Executable and Linkable Format, ELF)

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

#### 4.6.1 值传递



#### 4.6.2 地址传递

```cpp
viod swap(int * p1, int *p2)
{
    int temp = *p1;
    *p1  = *p2;
    *p2 = temp;
}
swap(&a,&b)
```

### 4.7 智能指针

在c++中，动态内存的管理式通过一对运算符来完成的：new,在动态内存中为对象分配空间并返回一个指向该对象的指针，我们可以选择对对象进行初始化；delete，接受一个动态对象的指针，销毁该对象，并释放与之关联的内存。动态内存的使用很容易出现问题，因为确保在正确的时间释放内存是极其困难的。有时使用完对象后，忘记释放内存，造成内存泄漏的问题。

- 所谓的**智能指针本质就是一个类模板**，它可以创建任意的类型的指针对象，当智能指针对象使用完后，**对象就会自动调用析构函数去释放该指针所指向的空间**。

#### 4.7.1 shared_ptr 

**shared_ptr** 是**C++11**提供的一种[智能指针](https://so.csdn.net/so/search?q=智能指针&spm=1001.2101.3001.7020)类，它足够智能，可以在任何地方都不使用时自动删除相关指针，从而帮助彻底消除内存泄漏和悬空指针的问题。
它遵循共享所有权的概念，即不同的 shared_ptr 对象可以与相同的指针相关联，并在内部使用引用计数机制来实现这一点。

**每个 shared_ptr 对象在内部指向两个内存位置：**

- 指向对象的指针。
- 用于控制引用计数数据的指针。

**共享所有权在参考计数的帮助下的工作模式：**

- 当新的 shared_ptr 对象与指针关联时，则在其构造函数中，将与此指针关联的引用计数增加1。
- 当任何 shared_ptr 对象超出作用域时，则在其析构函数中，它将关联指针的引用计数减1。如果引用计数变为0，则表示没有其他 shared_ptr 对象与此内存关联，在这种情况下，它使用delete函数删除该内存。

##### （1）创建 shared_ptr 对象

```cpp
std::shared_ptr<int> p1(new int());//使用原始指针创建 shared_ptr 对象
```

上面这行代码在堆上创建了两块内存：

- 存储`int`。
- 用于引用计数的内存，管理附加此内存的 shared_ptr 对象的计数，最初计数将为1。

```cpp
p1.use_count();//检查 shared_ptr 对象的引用计数
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



# C++核心编程



## 1.  内存分区模型

C++程序在执行时，将内存大方向划成**四个区域**

- 代码区：存放函数体的二进制代码，由操作系统进行管理
- 全局区：存放全局变量和静态变量以及常量
- 栈区：由编译器自动分配释放，存放函数的参数值，局部变量等
- 堆区：由程序员分配和释放，若程序员不释放，程序结束时由操作系统释放

[backward-cpp](https://blog.csdn.net/ccf19881030/article/details/113446856?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522166009863716782246428333%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=166009863716782246428333&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-113446856-null-null.142^v40^new_blog_pos_by_title,185^v2^control&utm_term=backward-cpp&spm=1018.2226.3001.4187)

**内存四区的意义**

> 不同区域存放的数据，赋予不同的生命周期，给我们更大的灵活。

### 1.1  程序运行前

在程序编译后，生成exe可执行程序，**未执行该程序前**分为两个区域

**代码区：**

- 存放CPU执行的机械命令
- 代码区是**共享**的，共享的目的是对于频繁被执行的程序，只需要在内存中有一份代码即可
- 代码区是**只读**的，使2其只读的愿意是防止程序意外地修改了它的指令

**全局区：**

- 全局变量和静态变量存放在此
- 全局还包含了常量区，字符串常量和其他常量也存放在此
- **该区域的数据在程序结束后由操作系统释放**

```cpp
int g_a = 10;//全局变量
const int c_g_a = 10;//全局常量

int main ()
{
    int a = 10;//局部变量
    static int s_a = 10 ;//前加static，属于静态变量
}
//常量 //字符串常量 
 string "hello,world"
             //const修饰的常量  //const修饰的全局变量
     
                                                    //const修饰的局部变量
const int c_l_a = 10;


```



不在全局区中：

1. 局部变量
2. const 修饰的局部变量（局部常量）

在全局区中：

1. 全局变量
2. 静态变量 static关键字
3. 常量        字符串常量

​                          const修饰的全局变量（全局常量）

### 1.2  程序运行后

**栈区：**

- 由编译器自动分配释放，存放函数的参数值，局部变量等
- **注意**，不要返回局部变量的地址，栈区开辟的数据由编译器自动释放

```cpp
int * func ()
{
      int a =10 ;//局部变量，存放在栈区，栈区的数据在函数执行完后自动释放
      return &a;
}

int *func ()
{
    int a =10;
    return &a;
}
int main()
{
    int * p = func();
    cout<<*p <<endl;//第一次可以打印正确数字，是因为编译器做了保留
    cout<<*p<<endl;//不再被保留
}
```

​     **堆区：**

- 由程序员分配和释放，若程序员不释放，程序结束时由操作系统释放
- 在cpp中主要是利用new在堆区开辟内存

```cpp
int func()
{
    //利用new关键字可以将数据开辟到堆区
    //指针本质是一个局部变量，放在栈上，指针保存的数据存放在堆区上。
  int *p =new int(10);
    return p;
}

int main()
{
    //在堆区开辟数据
    int *p=func();
    cout<<*p<<endl;
}
```

### 1.3  new操作符

- `new`:在堆区开辟数据
- `delete`:手动释放

```cpp
int func()
{
   //利用new创建的数据，会返回该数据类型的指针
  int *p =new int(10);
    return p;
}
void test01()
{
    int * p= func();
    cout<<*p<<endl;
    delete p;
}
void test02()
{
    int * arr = new int[10];
    delete[] arr;//释放数组时，要加上[]
}

int main()
{
    test01();
    test02();
}
```

​              

## 2.  引用

### 2.1  引用的基本使用

作用：给变量起别名 **别名可以和原名一样**

```cpp
数据类型      &别名= 原名
```

对引用的数据进行修改，会直接修改内存的存储。

### 2.2  引用的注意事项

- 引用必须初始化
- 引用在初始化后，不可以改变指向

### 2.3  引用做函数参数

作用：函数传参时，可以利用引用让形参修饰实参

优点：可以简化指针修改实参

```cpp
void  swap(int *a , int *b){
    
}//地址传递  指针

void swap(int &a ,int &b){
    
}//引用

```

### 2.4  引用做函数返回值

作用：可以作为函数返回值

注意：

- 不要返回局部变量（**局部变量存放在栈中，用完释放**）
- 函数的调用可以作为左值（等号左边）

```cpp
//不要返回局部变量
int& test01()
{
		 int a= 10;
   		 return a;//局部变量存放在栈中，用完释放
}

int main ()
{
    int &ref =test01();//第一次是编译器做了保留
    int &ref =test01();//属于a的内存被释放 
    cout<<ref;
}

//函数的调用可以作为左值
int& test02()
{
		static  int a= 10;//静态变量，存放在全局区，全局区上的数据在程序结束后由系统释放
   		 return a;//返回的是a的同名引用
}
int main ()
{
    int &ref =test02();
    int &ref =test02();
    cout<<ref;
    test02() = 1000;//如果函数的返回值是引用，则函数调用可以作为左值
    cout<<ref;
}
```

### 2.5  引用的本质

本质：指针常量

```cpp
void func(int& ref){
    ref =100;
}

int main(){
    int a =10;
    
    int &ref =a ;
    ref =20;
    cout <<a<<endl;
    cout <<ref <<endl;
    
    func(a);
    return 0;
}
```



> ```cpp
> //声明指针： 
> char* pc;
> //声明引用： 
> char c = 'A';
> char& rc = c;
> ```
>
> 它们的区别：
>
> - **从现象上看**，指针在运行时可以改变其所指向的值，而引用一旦和某个对象绑定后就不再改变。这句话可以理解为：指针可以被重新赋值以指向另一个不同的对象。但是引用则总是指向在初始化时被指定的对象，以后不能改变，但是指定的对象其内容可以改变。
> - **从内存分配上看**，程序为指针变量分配内存区域，而不为引用分配内存区域，因为引用声明时必须初始化，从而指向一个已经存在的对象。引用不能指向空值。
> - **从编译上看**，程序在编译时分别将指针和引用添加到符号表上，符号表上记录的是变量名及变量所对应地址。指针变量在符号表上对应的地址值为指针变量的地址值，而引用在符号表上对应的地址值为引用对象的地址值。符号表生成后就不会再改，因此指针可以改变指向的对象（指针变量中的值可以改），而引用对象不能改。这是使用指针不安全而使用引用安全的主要原因。从某种意义上来说引用可以被认为是不能改变的指针。
> - **不存在指向空值的引用这个事实**意味着使用引用的代码效率比使用指针的要高。因为在使用引用之前不需要测试它的合法性。相反，指针则应该总是被测试，防止其为空。
> - 理论上，对于指针的级数没有限制，但是引用只能是一级。如下：
>     int\** p1; // 合法。指向指针的指针
>     int\*& p2; // 合法。指向指针的引用
>     int&* p3; // 非法。指向引用的指针是非法的
>     int&& p4; // 非法。指向引用的引用是非法的
>     注意上述读法是从左到右。





### 2.6  常量引用

作用：常量引用主要用来修饰形参，防止误操作

在函数形参列表中，可以加const修饰形参，防止形参改变实参。

<font color=sandybrown>只读</font>

```cpp
const int & ref = 10;
//加上cosnt之后，编译器将代码修改为
//int temp =10; int & ref = temp;
```



## 3.  函数提高

### 3.1  函数默认参数

在C++中，函数的形参列表是可以有默认值的

```cpp
返回值类型   函数名  （参数  =  默认值 ）{}
```

```cpp
int func(int a , int b , int c)
```

注意事项

1. 如果某个位置已经有了默认参数，那么从那个位置之后，从左到右都必须有默认值
2. 如果函数声明有了默认参数，那么函数的实现就不能有默认参数

### 3.2  函数的占位参数

C++中函数的形参列表中可以有占位参数，用来做占位，调用函数时必须填补该位置

```
返回值类型    函数名  （数据类型）{}
```

### 3.3  函数重载

#### 3.3.1  概述

作用：函数名可以相同，提高复用率

满足条件：

- 同一个作用域下
- 函数名相同
- 函数参数的**类型不同**，或**个数不同**，或**顺序不同**

<font color=sandybrown>函数返回值不可以作为重载条件</font>

```cpp
void func()
{
    cout<<"!"<<endl;
}

void func(int a)
{
    cout<<"?"<<endl;
}
```

#### 3.3.2  注意事项

- 引用作为重载条件
- 函数重载碰到函数默认参数

```cpp
//引用作为重载条件
void fun(int &a)
{
    cout<<"fun(int &a)"<<endl;
}

void fun(const int &a)
{
    cout<<"fun(const  int &a)"<<endl;
}

int main(){
    int a =10 ;
    fun(a);//调用第一个函数
    fun(10);//调用第二个函数
}

//碰到默认参数
void func(int a,int b =10)
{
    cout<<a<<endl;
}

void func(int a)
{
    cout<<a<<endl;
}
```

## 4.  类和对象

<hr>

<table><tr><td bgcolor = grey>C++面向对象三大特性：封装、继承、多态</td></tr></table>

<hr>

### 4.1  封装

#### 4.1.1  封装的意义

- 将属性和行为作为一个整体，表现真实事物

```cpp
class  类名{ 访问权限：属性 / 行为 }
```

```cpp
class Circle
{
    //访问权限
    public:
    	//属性
    int m_r;
    	//行为
    double calculate()
    {
        return 2*PI*m_r;
    }
};

int main(){
    //通过圆类，创建具体的对象
    Circle cl;
    //给对象 的属性赋值
    cl.m_r =10;
    
    cout<<cl.calculate,,endl;
}


//赋值有两种
cl.m_r =10 ;
cl. getr(10);
```

类中的属性和行为 我们统一称为  成员

1. 属性：成员属性  成员变量

2. 行为：成员函数  成员方法

- 将属性和行为加以权限控制

访问权限有三种：   

| 权限     |           | 类内     | 类外       |
| -------- | --------- | -------- | ---------- |
| 公共权限 | public    | 可以访问 | 可以访问   |
| 保护权限 | protected | 可以访问 | 不可以访问 |
| 私有权限 | prviate   | 可以访问 | 不可以访问 |

*protected和prviate区别主要体现在继承中*

```cpp
class person
{
    public:
    	string name;
    protected:
    	string car;
    private:
    	int password;
    public:
    	void fun()
        {
            name = "zhangsan";
            car = "tuolaiji";
            password =1234;
        }
}
```

#### 4.1.2  struct和class区别

唯一区别在于<font color=sandybrown>默认的访问权限不同</font>

- class默认权限私有
- struct默认权限共有

#### 4.1.3  成员属性设置为私有

优点1：将所有成员属性设置为私有，可以自己控制读写权限

优点2：对于写权限，我们可以检测数据的有效性

```cpp
class person
{
    public:
    //设置姓名
    void setname(string name)
    {
        m_name =name;
    }
    //读取姓名
    string getname()
    {
        return m_name;
    }
    //获取年龄 
    int getage()
    {
        m_age=0;
        return age;
    }
    //设置情人  只写
    void setlover(string lover)
    { 
        m_lover =lover;
    }
    private:
    string m_name;//可读可写
    int   m_age;//只读
    string m_lover;//只写
}
```



注意用成员函数和全局函数判断的区别

```cpp
bool  isSame(Cube &c)
{
    if (m_L==c.getL()&&m_W==c.getW()&&n_H==c.getH())
}

bool  isSame(Cube &c1, Cube &c2)
{
    if (c1.getL()==c2.getL()&&c1.getW()==c2.getW()&&c1.getH()==c2.getH())
}
```

### 4.2 对象的初始化和清理

#### 4.2.1 构造函数和析构函数

c++利用了**构造函数**和**析构函数**解决上述问题，这两个函数将会被编译器自动调用，完成对象初始化和清理工作。

对象的初始化和清理工作是编译器强制要我们做的事情，因此如果**我们不提供构造和析构，编译器会提供**

**编译器提供的构造函数和析构函数是空实现。**

- 构造函数：主要作用在于创建对象时为对象的成员属性赋值，构造函数由编译器自动调用，无须手动调用。
- 析构函数：主要作用在于对象**销毁前**系统自动调用，执行一些清理工作。

**构造函数语法：**`类名(){}`

1. 构造函数，没有返回值也不写void
2. 函数名称与类名相同
3. 构造函数可以有参数，因此可以发生重载
4. 程序在调用对象时候会自动调用构造函数，无须手动调用,而且只会调用一次

**析构函数语法：** `~类名(){}`

1. 析构函数，没有返回值也不写void
2. 函数名称与类名相同,在名称前加上符号`~`
3. 析构函数不可以有参数，因此不可以发生重载
4. 程序在对象销毁前会自动调用析构，无须手动调用,而且只会调用一次

#### 4.2.2 构造函数的分类和调用

两种分类方式：

 按参数分为： 有参构造和无参构造

 按类型分为： 普通构造和拷贝构造

三种调用方式：

 括号法

 显示法

 隐式转换法

```cpp
//1、构造函数分类
// 按照参数分类分为 有参和无参构造   无参又称为默认构造函数
// 按照类型分类分为 普通构造和拷贝构造

class Person {
public:
	//无参（默认）构造函数
	Person() {
		cout << "无参构造函数!" << endl;
	}
	//有参构造函数
	Person(int a) {
		age = a;
		cout << "有参构造函数!" << endl;
	}
	//拷贝构造函数
	Person(const Person& p) {
        // 将传入的人身上的所有属性，拷贝到我身上
		age = p.age;
		cout << "拷贝构造函数!" << endl;
	}
	//析构函数
	~Person() {
		cout << "析构函数!" << endl;
	}
public:
	int age;
};

//2、构造函数的调用
//调用无参构造函数
void test01() {
	Person p; //调用无参构造函数
}

//调用有参的构造函数
void test02() {

	//2.1  括号法，常用
	Person p1(10);
	//注意1：调用无参构造函数不能加括号，如果加了编译器认为这是一个函数声明
	//Person p2();

	//2.2 显式法
	Person p2 = Person(10); 
	Person p3 = Person(p2);
	//Person(10)单独写就是匿名对象  当前行结束之后，马上析构

	//2.3 隐式转换法
	Person p4 = 10; // Person p4 = Person(10); 
	Person p5 = p4; // Person p5 = Person(p4); 

	//注意2:不能利用 拷贝构造函数 初始化匿名对象 编译器认为是对象声明
	//Person p5(p4);
}

int main() {

	test01();
	//test02();

	system("pause");

	return 0;
}

```

#### 4.2.3 拷贝函数调用时机

C++中拷贝构造函数调用时机通常有三种情况

- 使用一个已经创建完毕的对象来初始化一个新对象
- 值传递的方式给函数参数传值
- 以值方式返回局部对象

```cpp
class Person {
public:
	Person() {
		cout << "无参构造函数!" << endl;
		mAge = 0;
	}
	Person(int age) {
		cout << "有参构造函数!" << endl;
		mAge = age;
	}
	Person(const Person& p) {
		cout << "拷贝构造函数!" << endl;
		mAge = p.mAge;
	}
	//析构函数在释放内存之前调用
	~Person() {
		cout << "析构函数!" << endl;
	}
public:
	int mAge;
};

//1. 使用一个已经创建完毕的对象来初始化一个新对象
void test01() {

	Person man(100); //p对象已经创建完毕
	Person newman(man); //调用拷贝构造函数
	Person newman2 = man; //拷贝构造

	//Person newman3;
	//newman3 = man; //不是调用拷贝构造函数，赋值操作
}

//2. 值传递的方式给函数参数传值
//相当于Person p1 = p;
void doWork(Person p1) {}
void test02() {
	Person p; //无参构造函数
	doWork(p);
}

//3. 以值方式返回局部对象
Person doWork2()
{
	Person p1;
	cout << (int *)&p1 << endl;
	return p1;
}

void test03()
{
	Person p = doWork2();
	cout << (int *)&p << endl;
}


int main() {

	//test01();
	//test02();
	test03();

	system("pause");

	return 0;
}

```



#### 4.2.4 构造函数调用规则



默认情况下，c++编译器至少给一个类添加3个函数

1．默认构造函数(无参，函数体为空)

2．默认析构函数(无参，函数体为空)

3．默认拷贝构造函数，对属性进行值拷贝

构造函数调用规 则如下：

- 如果用户定义有参构造函数，c++不在提供默认无参构造，但是会提供默认拷贝构造
- 如果用户定义拷贝构造函数，c++不会再提供其他构造函数

```cpp
class Person {
public:
	//无参（默认）构造函数
	Person() {
		cout << "无参构造函数!" << endl;
	}
	//有参构造函数
	Person(int a) {
		age = a;
		cout << "有参构造函数!" << endl;
	}
	//拷贝构造函数
	Person(const Person& p) {
		age = p.age;
		cout << "拷贝构造函数!" << endl;
	}
	//析构函数
	~Person() {
		cout << "析构函数!" << endl;
	}
public:
	int age;
};

void test01()
{
	Person p1(18);
	//如果不写拷贝构造，编译器会自动添加拷贝构造，并且做浅拷贝操作
	Person p2(p1);

	cout << "p2的年龄为： " << p2.age << endl;
}

void test02()
{
	//如果用户提供有参构造，编译器不会提供默认构造，会提供拷贝构造
	Person p1; //此时如果用户自己没有提供默认构造，会出错
	Person p2(10); //用户提供的有参
	Person p3(p2); //此时如果用户没有提供拷贝构造，编译器会提供

	//如果用户提供拷贝构造，编译器不会提供其他构造函数
	Person p4; //此时如果用户自己没有提供默认构造，会出错
	Person p5(10); //此时如果用户自己没有提供有参，会出错
	Person p6(p5); //用户自己提供拷贝构造
}

int main() {

	test01();

	system("pause");

	return 0;
}

```



## 5. 多线程

> 多线程（multithreading），是指从软件或者硬件上实现多个线程并发执行的技术。具有多线程能力的计算机因有硬件支持而能够在同一时间执行多于一个线程，进而提升整体处理性能。
> 在一个程序中，这些独立运行的程序片段叫作“线程”（Thread），利用它编程的概念就叫作“多线程处理”。

### 5.1 进程与线程的区别

> 进程是正在运行的程序的实例，而线程是是进程中的实际运作单位。

区别：

- 一个程序有且只有一个进程，但可以拥有至少一个的线程。
- 不同进程拥有不同的地址空间，互不相关，而不同线程共同拥有相同进程的地址空间。



### 5.2 std::thread 成员函数

#### 5.2.1 构造&析构函数

| 函数                                                         | 类别           | 作用                                       |
| ------------------------------------------------------------ | -------------- | ------------------------------------------ |
| thread() noexcept                                            | 默认构造函数   | 创建一个线程， 什么也不做                  |
| template <class Fn, class… Args> explicit thread(Fn&& fn, Args&&… args) | 初始化构造函数 | 创建一个线程， 以`args`为参数 执行`fn`函数 |
| thread(const thread&) = delete                               | 复制构造函数   | （已删除）                                 |
| thread(thread&& x) noexcept                                  | 移动构造函数   | 构造一个与`x` 相同的对象,会破坏`x`对象     |
| ~thread()                                                    | 析构函数       | 析构对象                                   |

#### 5.2.2 常用成员函数

| 函数                             | 作用                                                         |
| -------------------------------- | ------------------------------------------------------------ |
| void join()                      | 等待线程结束并清理资源（会阻塞）                             |
| bool joinable()                  | 返回线程是否可以执行join函数                                 |
| void detach()                    | 将线程与调用其的线程分离，彼此独立执行（此函数必须在线程创建时立即调用，且调用此函数会使其不能被join） |
| std::thread::id get_id()         | 获取线程id                                                   |
| thread& operator=(thread && rhs) | 见移动构造函数（如果对象是joinable的，那么会调用`std::terminate()`结果程序） |



### 5.3 基础知识

#### 5.3.1 创建多线程

```cpp
#include<iostream>
#include<thread>
using namespace std;
void proc(int a)
{
    cout << "我是子线程,传入参数为" << a << endl;
    cout << "子线程中显示子线程id为" << this_thread::get_id()<< endl;
}
int main()
{
    cout << "我是主线程" << endl;
    int a = 9;
    thread th2(proc,a);//第一个参数为函数名，第二个参数为该函数的第一个参数，如果该函数接收多个参数就依次写在后面。此时线程开始执行。
    cout << "主线程中显示子线程id为" << th2.get_id() << endl;
    th2.join()；//此时主线程被阻塞直至子线程执行结束。
    return 0;
}

```

#### 5.3.2 互斥量操作

```cpp
#include<iostream>
#include<thread>
#include<mutex>
using namespace std;
mutex m;//实例化m对象，不要理解为定义变量
void proc1(int a)
{
    m.lock();
    cout << "proc1函数正在改写a" << endl;
    cout << "原始a为" << a << endl;
    cout << "现在a为" << a + 2 << endl;
    m.unlock();
}

void proc2(int a)
{
    m.lock();
    cout << "proc2函数正在改写a" << endl;
    cout << "原始a为" << a << endl;
    cout << "现在a为" << a + 1 << endl;
    m.unlock();
}
int main()
{
    int a = 0;
    thread th1(proc1, a);
    thread th2(proc2, a);
    th1.join();
    th2.join();
    return 0;
}

```

不推荐直接去调用成员函数lock()，因为如果忘记unlock()，将导致锁无法释放，使用lock_guard或者unique_lock能避免忘记解锁这种问题。

- **lock_guard()**:
  声明一个局部的lock_guard对象，在其构造函数中进行加锁，在其析构函数中进行解锁。

  最终的结果就是：**创建即加锁，作用域结束自动解锁**。从而使用lock_guard()就可以替代lock()与unlock()。
  通过设定作用域，使得lock_guard在合适的地方被析构（在互斥量锁定到互斥量解锁之间的代码叫做临界区（需要互斥访问共享资源的那段代码称为临界区），临界区范围应该尽可能的小，即lock互斥量后应该尽早unlock），通过使用{}来调整作用域范围，可使得互斥量m在合适的地方被解锁：

```cpp
#include<iostream>
#include<thread>
#include<mutex>
using namespace std;
mutex m;//实例化m对象，不要理解为定义变量
void proc1(int a)
{
    lock_guard<mutex> g1(m);//用此语句替换了m.lock()；lock_guard传入一个参数时，该参数为互斥量，此时调用了lock_guard的构造函数，申请锁定m
    cout << "proc1函数正在改写a" << endl;
    cout << "原始a为" << a << endl;
    cout << "现在a为" << a + 2 << endl;
}//此时不需要写m.unlock(),g1出了作用域被释放，自动调用析构函数，于是m被解锁

void proc2(int a)
{
    {
        lock_guard<mutex> g2(m);
        cout << "proc2函数正在改写a" << endl;
        cout << "原始a为" << a << endl;
        cout << "现在a为" << a + 1 << endl;
    }//通过使用{}来调整作用域范围，可使得m在合适的地方被解锁
    cout << "作用域外的内容3" << endl;
    cout << "作用域外的内容4" << endl;
    cout << "作用域外的内容5" << endl;
}
int main()
{
    int a = 0;
    //thread proc1(proc1, a);
    //thread proc2(proc2, a);
    //proc1.join();
    //proc2.join();                不允许函数名等于变量名
    thread th1(proc1, a);
    thread th2(proc2, a);
    th1.join();
    th2.join();
    return 0;
}

```

lock_gurad也可以传入两个参数，第一个参数（m）由`adopt_lock`标识时，表示构造函数中不再进行互斥量锁定，因此此时需要提前手动锁定。

- adopt_lock
  - std::adopt_lock参数表示互斥量已经被lock，不需要再重复lock
  - 该互斥量之前必须已经lock，才可以使用该参数

```cpp
#include<iostream>
#include<thread>
#include<mutex>
using namespace std;
mutex m;//实例化m对象，不要理解为定义变量
void proc1(int a)
{
    m.lock();//手动锁定
    lock_guard<mutex> g1(m,adopt_lock);
    cout << "proc1函数正在改写a" << endl;
    cout << "原始a为" << a << endl;
    cout << "现在a为" << a + 2 << endl;
}//自动解锁

void proc2(int a)
{
    lock_guard<mutex> g2(m);//自动锁定
    cout << "proc2函数正在改写a" << endl;
    cout << "原始a为" << a << endl;
    cout << "现在a为" << a + 1 << endl;
}//自动解锁
int main()
{
    int a = 0;
    thread th1(proc1, a);
    thread th2(proc2, a);
    th1.join();
    th2.join();
    return 0;
}

```



- **unique_lock()**:

  类似lock_guard。unique_lock的第二个参数，除了可以是`adopt_lock`，还可以是`try_to_lock`与`defer_lock`;

  - try_to_lock: 尝试去锁定，得保证锁处于unlock的状态,然后尝试现在能不能获得锁；尝         试用mutex的lock()去锁定这个mutex，但如果没有锁定成功，会立即返回，不会阻塞在那里

    > 如果有一个线程被lock，而且执行时间很长，那么另一个线程一般会被阻塞在那里，反而会造成时间的浪费。那么使用了try_to_lock后，如果被锁住了，它不会在那里阻塞等待，它可以先去执行其他没有被锁的代码

  - defer_lock: 始化了一个没有加锁的mutex;

**<font color = SandyBrown>std::unqiue_lock的时空间性能均劣于std::lock_guard，这也是它为灵活性付出的代价</font>**
std::unqiue_lock内存在某种标志用于表征其实例是否拥有特定的互斥量，显然，这些标志需要占据空间，并且标志的检查与更新也需要耗费时间



```cpp
#include<iostream>
#include<thread>
#include<mutex>
using namespace std;
mutex m;
void proc1(int a)
{
    unique_lock<mutex> g1(m, defer_lock);//始化了一个没有加锁的mutex
    cout << "不拉不拉不拉" << endl;
    g1.lock();//手动加锁，注意，不是m.lock();注意，不是m.lock();注意，不是m.lock()
    cout << "proc1函数正在改写a" << endl;
    cout << "原始a为" << a << endl;
    cout << "现在a为" << a + 2 << endl;
    g1.unlock();//临时解锁
    cout << "不拉不拉不拉"  << endl;
    g1.lock();
    cout << "不拉不拉不拉" << endl;
}//自动解锁

void proc2(int a)
{
    unique_lock<mutex> g2(m,try_to_lock);//尝试加锁，但如果没有锁定成功，会立即返回，不会阻塞在那里；
    cout << "proc2函数正在改写a" << endl;
    cout << "原始a为" << a << endl;
    cout << "现在a为" << a + 1 << endl;
}//自动解锁
int main()
{
    int a = 0;
    thread th1(proc1, a);
    thread th2(proc2, a);
    th1.join();
    th2.join();
    return 0;
}
//unique_lock所有权的转移

//mutex m;
//{  
//    unique_lock<mutex> g2(m,defer_lock);
//    unique_lock<mutex> g3(move(g2));//所有权转移，此时由g3来管理互斥量m
//    g3.lock();
//    g3.unlock();
//    g3.lock();
//}

```

- **release**()
  - 解除unique_lock和mutex对象的联系，并将原mutex对象的`指针`返回出来
  - 如果之前的mutex已经加锁，需在后面自己手动unlock解锁

```cpp
std::mutex mlock;

void work1(int& s) {
	for (int i = 1; i <= 5000; i++) {
		std::unique_lock<std::mutex> munique(mlock);   // 这里是自动lock
		std::mutex *m = munique.release();
		s += i;
		m->unlock();
	}
}
```



#### 5.3.2 不同域中互斥量的所有权的传递

互斥量所有权传递十分常见，比如在某个函数内完成对互斥量的上锁，并在其后将其所有权转交至调用者以保证它可以在该锁的保护范围内执行额外操作

```cpp
std::unique_lock<std::mutex> get_lock() {
  extern std::mutex some_mutex;//别处定义的some_mutex，此处引用
  std::unique_lock<std::mutex> lk(some_mutex);
  prepare_data();
  return lk;
} 

void process_data() {
  std::unique_lock<std::mutex> lk(get_lock());
  do_something(); 
}
```

> 在 C 语⾔中，修饰符 extern ⽤在变量或者函数的声明前，⽤来说明 “ **此变量/函数是在别处定义的，要在此处引⽤** ” 。**注意 extern 声明的位置对其作⽤域也有关系** ，如果是在 main 函数中进⾏声明的，则只能在 main 函数中调⽤，在 其它函数中不能调⽤。其实要调⽤其它⽂件中的函数和变量，只需把该⽂件⽤ #include 包含进来即可，为啥要⽤ extern？**因为⽤ extern 会加速程序的编译过程**，这样能节省时间。

<font color = SandyBrown>函数或变量可以声明多次 (如果是变量多次声明需要加 extern 关键字)，但定义只能有一次。</font>

> 在 C++ 中 extern 还有另外⼀种作⽤，⽤于指示 C 或者 C ＋＋函数的调⽤规范。⽐如在 C ＋＋ 中调⽤ C 库函数，就需要在 C++ 程序中⽤ extern “C” 声明要引⽤的函数。这是给链接器⽤的，告诉链接器在链接的时候⽤ C 函数规范来链接。主要原因是 C++  和 C 程序编译完成后在⽬标代码中命名规则不同，⽤此来解决名字匹配的问题。

对unique_lock的对象来说，一个对象只能和一个mutex锁唯一对应，不能存在一对多或者多对一的情况，不然会造成死锁的出现

所以如果想要传递两个unique_lock对象对mutex的权限，需要运用到移动语义或者移动构造函数

注意，unique_lock和lock_guard都不能复制，lock_guard不能移动，但是unique_lock可以

```cpp
// unique_lock 可以移动，不能复制
std::unique_lock<std::mutex> guard1(_mu);
std::unique_lock<std::mutex> guard2 = guard1;  // error
std::unique_lock<std::mutex> guard2 = std::move(guard1); // ok

// lock_guard 不能移动，不能复制
std::lock_guard<std::mutex> guard1(_mu);
std::lock_guard<std::mutex> guard2 = guard1;  // error
std::lock_guard<std::mutex> guard2 = std::move(guard1); // error

```



#### 5.3.4 锁的粒度

锁的粒度是一个摆手术语(hand-waving term)，用来描述一个锁保护着的数据量大小

一个细粒度锁(a fine-grained lock)能够保护较小的数据量

一个粗粒度锁(a coarse-grained lock)能够保护较多的数据量

互斥锁保证了线程间的同步，但是却将并行操作变成了串行操作，这对性能有很大的影响，所以我们要尽可能的**减小锁定的区域**，也就是使用**细粒度锁**

```cpp
class LogFile {
    std::mutex _mu;
    ofstream f;
public:
    LogFile() {
        f.open("log.txt");
    }
    ~LogFile() {
        f.close();
    }
    void shared_print(string msg, int id) {
        {
            std::lock_guard<std::mutex> guard(_mu);
            //do something 1
        }
        //do something 2
        {
            std::lock_guard<std::mutex> guard(_mu);
            // do something 3
            f << msg << id << endl;
            cout << msg << id << endl;
        }
    }

};

```

上面的代码中，一个函数内部有两段代码需要进行保护，这个时候使用lock_guard就需要创建两个局部对象来管理同一个互斥锁（其实也可以只创建一个，但是锁的力度太大，效率不行），修改方法是使用unique_lock。它提供了lock()和unlock()接口，能记录现在处于上锁还是没上锁状态，在析构的时候，会根据当前状态来决定是否要进行解锁（lock_guard就一定会解锁）

```cpp
class LogFile {
    std::mutex _mu;
    ofstream f;
public:
    LogFile() {
        f.open("log.txt");
    }
    ~LogFile() {
        f.close();
    }
    void shared_print(string msg, int id) {

        std::unique_lock<std::mutex> guard(_mu);
        //do something 1
        guard.unlock(); //临时解锁

        //do something 2

        guard.lock(); //继续上锁
        // do something 3
        f << msg << id << endl;
        cout << msg << id << endl;
        // 结束时析构guard会临时解锁
        // 这句话可要可不要，不写，析构的时候也会自动执行
        // guard.ulock();
    }
```

上面的代码可以看到，在无需加锁的操作时，可以先临时释放锁，然后需要继续保护的时候，可以继续上锁，这样就无需重复的实例化lock_guard对象，还能减少锁的区域。同样，可以使用`std::defer_lock`设置**初始化的时候不进行默认的上锁操作**

```cpp
void shared_print(string msg, int id) 
{
    std::unique_lock<std::mutex> guard(_mu, std::defer_lock);
    //do something 1

    guard.lock();
    // do something protected
    guard.unlock(); //临时解锁

    //do something 2

    guard.lock(); //继续上锁
    // do something 3
    f << msg << id << endl;
    cout << msg << id << endl;
    // 结束时析构guard会临时解锁
}
```

- **condition_variable**:
  需要#include<condition_variable>;
  - wait(locker):在线程被阻塞时，该函数会自动调用 locker.unlock() 释放锁，使得其他被阻塞在锁竞争上的线程得以继续执行。另外，一旦当前线程获得通知(通常是另外某个线程调用 notify_* 唤醒了当前线程)，wait() 函数此时再自动调用 locker.lock()。
  - notify_all():唤醒所有等待的线程
  - notify_once():随机唤醒一个等待的线程

#### 5.3.5 异步进程

```cpp
#include <future>
```

async是一个函数模板，用来启动一个异步任务，它返回一个future类模板对象，future对象起到了占位的作用，刚实例化的future是没有储存值的，但在调用future对象的get()成员函数时，主线程会被阻塞直到异步线程执行结束，并把返回结果传递给future，即通过FutureObject.get()获取函数返回值。

> 相当于你去办政府办业务（主线程），把资料交给了前台，前台安排了人员去给你办理（async创建子线程），前台给了你一个单据（future对象），说你的业务正在给你办（子线程正在运行），等段时间你再过来凭这个单据取结果。过了段时间，你去前台取结果，但是结果还没出来（子线程还没return），你就在前台等着（阻塞），直到你拿到结果（get()）你才离开（不再阻塞）。

```cpp
#include <iostream>
#include <thread>
#include <mutex>
#include<future>
#include<Windows.h>
using namespace std;
double t1(const double a, const double b)
{
	double c = a + b;
	Sleep(3000);//假设t1函数是个复杂的计算过程，需要消耗3秒
	return c;
}

int main() 
{
	double a = 2.3;
	double b = 6.7;
	future<double> fu = async(t1, a, b);//创建异步线程线程，并将线程的执行结果用fu占位；向函数t1传入a,b两个参数，等待运行结果时，用fu占位提醒正在等待
	cout << "正在进行计算" << endl;
	cout << "计算结果马上就准备好，请您耐心等待" << endl;
	cout << "计算结果：" << fu.get() << endl;//阻塞主线程，直至异步线程return
        //cout << "计算结果：" << fu.get() << endl;//取消该语句注释后运行会报错，因为future对象的get()方法只能调用一次。
	return 0;
}

```

> Windows.h
>
> **Ubuntu里用unistd.h**
>
> system("cls");//清屏，清除运行框里所有的内容
> system("pause");//暂停，按任意键继续
> system("shutdown -s -t 60")//最后一个数字代表几秒，表示在几秒后关机
> system("shutdown -a");//取消关机的命令（与上一个搭配）
> Sleep(时间(毫秒));代表的是让计算机暂停输入输出特定时间，括号里写要暂停的毫秒数，如Sleep(1000)代表休眠1000毫秒(1秒)

- shared_future
  future与shard_future的用途都是为了占位，但是两者有些许差别。
  - future的`get()`成员函数是**转移数据所有权**;future对象的get()只能调用一次；无法实现多个线程等待同一个异步线程，一旦其中一个线程获取了异步线程的返回值，其他线程就无法再次获取。
  - shared_future的`get()`成员函数是**复制数据**。shared_future对象的get()可以调用多次；**可以实现多个线程等待同一个异步线程**，每个线程都可以获取异步线程的返回值。


| 类            | 语义 | 可否调用多次 |
| ------------- | ---- | ------------ |
| future        | 转移 | 否           |
| shared_future | 赋值 | 可           |



#### 5.3.6 原子类型automic

原子操作指“不可分割的操作”；也就是说这种操作状态要么是完成的，要么是没完成的。互斥量的加锁一般是针对一个代码段，而原子操作针对的一般都是一个变量。
automic是一个模板类，使用该模板类实例化的对象，提供了一些保证原子性的成员函数来实现共享数据的常用操作。

> 在以前，定义了一个共享的变量(int i=0)，多个线程会操作这个变量，那么每次操作这个变量时，都是用lock加锁，操作完毕使用unlock解锁，以保证线程之间不会冲突；
> 现在，实例化了一个类对象(automic I=0)来代替以前的那个变量，每次操作这个对象时，就不用lock与unlock，这个对象自身就具有原子性，以保证线程之间不会冲突。

automic对象提供了常见的原子操作（通过调用成员函数实现对数据的原子操作）：

- store是原子写操作，
- load是原子读操作。
- exchange是于两个数值进行交换的原子操作。

即使使用了automic，也要注意执行的操作是否支持原子性。一般atomic原子操作，针对++，–，+=，-=，&=，|=，^=是支持的。

#### 5.3.7 实例

```cpp
#include <iostream>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include<unistd.h>
using namespace std;

deque<int> q;
mutex mu;
condition_variable cond;
int c = 0;//缓冲区的产品个数

void producer() { 
	int data1;
	while (1) {//通过外层循环，能保证生成用不停止
		if(c < 3) {//限流
			{
				data1 = rand();
				unique_lock<mutex> locker(mu);//锁
				q.push_front(data1);
				cout << "存了" << data1 << endl;
				cond.notify_one();  // 通知取
				++c;
			}
			usleep(500);
		}
	}
}

void consumer() {
	int data2;//data用来覆盖存放取的数据
	while (1) {
		{
			unique_lock<mutex> locker(mu);
			while(q.empty())
				cond.wait(locker); //wati()阻塞前先会解锁,解锁后生产者才能获得锁来放产品到缓冲区；生产者notify后，将不再阻塞，且自动又获得了锁。
			data2 = q.back();//取的第一步
			q.pop_back();//取的第二步
			cout << "取了" << data2<<endl;
			--c;
		}
		usleep(1500);
	}
}
int main() {
	thread t1(producer);
	thread t2(consumer);
	t1.join();
	t2.join();
	return 0;
}

```







### 5.4 高级知识

#### 5.4.1 线程池

不采用线程池时：

创建线程 -> 由该线程执行任务 -> 任务执行完毕后销毁线程。即使需要使用到大量线程，每个线程都要按照这个流程来创建、执行与销毁。

虽然创建与销毁线程消耗的时间 远小于 线程执行的时间，但是对于需要频繁创建大量线程的任务，创建与销毁线程 所占用的时间与CPU资源也会有很大占比。**为了减少创建与销毁线程所带来的时间消耗与资源消耗，因此采用线程池的策略：**

程序启动后，预先创建一定数量的线程放入空闲队列中，这些线程都是处于阻塞状态，基本不消耗CPU，只占用较小的内存空间。接收到任务后，线程池选择一个空闲线程来执行此任务。任务执行完毕后，不销毁线程，线程继续保持在池中等待下一次的任务。

线程池所解决的问题：

-  需要频繁创建与销毁大量线程的情况下，减少了创建与销毁线程带来的时间开销和CPU资源占用。（**省时省力**）

- 实时性要求较高的情况下，由于大量线程预先就创建好了，接到任务就能马上从线程池中调用线程来处理任务，略过了创建线程这一步骤，提高了实时性。（**实时**）

















































## 6. boost库

> boost库是一个功能强大，构造精巧，跨平台的免费的C++开源库。它使得C++编程更优雅、更有活力、更高产，C++11的标准有三分之二来自boost库。在boost1.57版本时，就一共包含了129个组件，分为25个大类，涵盖了文本处理，容器，迭代器，算法，图像处理，模板元编程，并发编程等许多领域。
> 

### 6.1 [boost::bind](https://blog.csdn.net/weixin_34227447/article/details/93958241?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522166565372216782388075952%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=166565372216782388075952&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-93958241-null-null.142^v55^control,201^v3^add_ask&utm_term=boost%3A%3Abind&spm=1018.2226.3001.4187)

boost::bind是标准库函数std::bind1st和std::bind2nd的一种泛化形式。其可以支持函数对象、函数、函数指针、成员函数指针，**并且绑定任意参数到某个指定值上或者将输入参数传入任意位置。**

一般来说boost::bind有两种方式的调用，一种是对自由方法，也取非类方法， 一种是对类方法。

```cpp
boost::bind(函数名, 参数1，参数2，…)

boost::bind(&类名::方法名，类实例指针，参数1，参数2）
```

**需要注意的一点是，boost::bind里的参数个数一定要与被bind的函数相同。**

#### 6.1.1 通过函数和函数指针使用bind

>  函数指针：function pointers



```cpp
//给定以下两个函数
int f (int a , int b )
{
    return a+b;
}

int g (int a, int b ,int c){
    return a+b+c;
}

//可绑定所有参数
bind(f,1,2)//f(1,2)
bind(g,1,2,3)//g(1,2,3)
//可以选择性绑定参数
bind(f,_1,5)(x)//f(x,5),其中_1是一个占位符，表示用第一个参数来替换
bind(f,_2,_1)(x,y)//f(y,x),_1代表参数列表中的第一个位置上的参数,所以注意顺序
bind(g, _1, 9, _1)(x)//g(x, 9, x);
bind(g, _3, _3, _3)(x, y, z)//g(z, z, z);
```

说明：

- 传入bind函数的参数一般为变量的copy

- 如果想传入变量的引用，可以使用boost::ref和boost::cref

  如：

  ```cpp
  int i = 5;
  bind(f,ref(i),_1)(x);
  bind(f,cref(i),_1)(x);
  
  reference_wrapper<T>  ref(T& t);
  reference_wrapper<T>   cref(T const& t);
  ```

  

#### 6.1.2 通过函数对象使用bind

> 函数对象：function objects

```cpp
truct F
 {
     int operator()(int a, int b) { return a – b; }
     bool operator()(long a, long b) { return a == b; }
 };
 
F f;
int x = 100;
bind<int>(f, _1, _1)(x);        // f(x, x)
//可能某些编译器不支持上述的bind语法，可以用下列方式代替：
//boost::bind(boost::type<int>(), f, _1, _1)(x);
```

默认情况下，bind拥有的是函数对象的`copy`，但是也可以使用`boost::ref`和`boost::cref`来传入函数对象的引用，尤其是当该函数对象是`non-copyable`或者`expensive to copy`。

#### 6.1.3 通过指向成员的指针使用bind

> 指向成员的指针：pointers to members

bind将传入的成员（数据成员和成员函数）指针作为第一个参数，其行为如同使用boost::mem_fn将成员指针转换为一个函数对象，即：

bind(&X::f, args);    等价于`bind<R>(mem_fn(&X::f), args)`，其中R为X::f的返回类型（成员函数）或类型（数据成员）。

> mem_fun_ref是STL中对于成员函数引用的实现，主要为了兼容泛型算法使用成员函数，对容器元素进行具体操作。在使用成员函数作为函数参数时就得根据具体的函数要求是指针类型和引用类型，来区分是使用`men_fun`还是`men_fun_ref`。

```cpp
 struct X
 {
     bool f(int a);
 };
 
 X x;
 shared_ptr<X> p(new X);
 int i = 5;
 
bind(&X::f, ref(x), _1)(i);        // x.f(i)
bind(&X::f, &x, _1)(i);            // (&x)->f(i)
bind(&X::f, x, _1)(i);            // x.f(i)
bind(&X::f, p, _1)(i);            // p->f(i)
```



#### 6.1.4 使用嵌套绑定

> 嵌套绑定：nested binds

```cpp
bind(f, bind(g, _1))(x)
```

在外部bind计算之前，内部bind先被计算（如果内部有多个bind，则计算顺序不定）。如上，根据参数x，先计算bind(g, _1)(x)，生成g(x)，然后计算bind(f, g(x))(x)，最后生成f(g(x))。

 



## 7. 智能指针

 C++内存分四大块：

1. 全局 主函数运行前使用，初始化
2. 静态 变量第一次使用前，初始化
   以上两块内存都会在程序结束后自动释放
3. 堆区 由程序员管理，C++管理方法有new delete等关键字
4. 栈区 由[编译器](https://so.csdn.net/so/search?q=编译器&spm=1001.2101.3001.7020)管理，**存放程序的局部变量和参数**

因此我们需要关注堆区的[内存管理](https://so.csdn.net/so/search?q=内存管理&spm=1001.2101.3001.7020)。内存管理经常会碰到忘记释放造成的内存泄露。
在C++中引入了[智能指针](https://so.csdn.net/so/search?q=智能指针&spm=1001.2101.3001.7020)，有`shared_ptr`，`unique_ptr`和`weak_ptr`。

包含头文件

```cpp
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>
//或者
#include<memory>
```

### shared_ptr

shared_ptr，顾名思义是**多个指针指向一块内存**。
被管理对象有一个引用计数，这个计数记录在每个指针上，几个shared_ptr指向它，这个数字就是几，当没有任何shared_ptr指向它时，引用计数为0，这时，自动释放对象。

> 即，*当一个所有指向这块内存的指针生命周期结束时，这块内存会被释放*
> ***智能指针的唯一作用，就是自动delete对象***，即C++11的新特性，内存管理机制。
> 智能指针既然会自动delete对象，我们就不能再去手动delete对象了，否则，也会发生多次释放的问题

```cpp
#include <iostream>
#include <memory>
 
class Test {
public:
    // 无参构造函数
    Test();
    // 有参数的构造函数
    explicit Test(int a);
    // 析构函数
    ~Test();
};
 
Test::Test() {
    std::cout << "无参构造函数" << std::endl;
}
 
Test::Test(int a) {
    std::cout << "有参构造函数，a=" << a << std::endl;
}
 
Test::~Test() {
    std::cout << "析构函数" << std::endl;
}
 
int main(int argc, const char * argv[]) {
    auto p1 = new Test; // 划分堆空间
    std::shared_ptr<Test> sp(p1); // 创建智能指针
    std::cout << sp.use_count() << std::endl; // 打印引用计数
    {
        std::shared_ptr<Test> sp2(sp); // 创建另一个智能指针
        std::cout << sp.use_count() << std::endl; // 打印引用计数
    } // sp2生命周期结束，sp引用计数减1
    std::cout << sp.use_count() << std::endl; // 打印引用计数
    
    return 0;
}
```

### make_shared的引入

存在这样一个问题

```cpp

int main(int argc, const char * argv[]) {
    auto p1 = new Test; // 划分堆空间
    std::shared_ptr<Test> sp(p1); // 创建智能指针
    std::shared_ptr<Test> sp2(p1); // 创建另一个智能指针
    
    return 0;
    }

```

这段程序会抛出异常`double free detected`

new关键字返回的是对应的指针类型。

此处用了两个智能指针管理同一块内存，因为sp 和sp2不知道彼此的存在，所以也会**重复释放**。

同一个对象只能用同一套内存管理体系，**如果它已经有智能指针了，那么再创建智能指针时，需要通过原来已有的指针创建**，而不能重复用原始空间来创建。

为此,STL库提供了make_shared函数，其原型为

> make_shared函数的主要功能是在[动态内存](https://so.csdn.net/so/search?q=动态内存&spm=1001.2101.3001.7020)中分配一个对象并初始化它，返回指向此对象的shared_ptr;由于是通过shared_ptr管理内存，因此一种安全分配和使用动态内存的方法。

```cpp
template <typename T, typename ...Args>
std::shared_ptr<T> std::make_shared(Args && ...args)
```

官方鼓励用make_shared函数来创建对象，而不要手动去new，这样就可以防止我们去使用原始指针创建多个引用计数体系。

```cpp
int main(int argc, const char * argv[]) {
    auto sp = std::make_shared<int>(); // 分配堆空间，创建智能指针
    auto sp2 = sp; // 创建另一个智能指针
    
    return 0;
}
```



## 8. 继承

[C++：继承](https://blog.csdn.net/qq_62718027/article/details/125922249?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522167981668416800182147982%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=167981668416800182147982&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-125922249-null-null.142^v76^wechat,201^v4^add_ask,239^v2^insert_chatgpt&utm_term=C%2B%2B%E7%BB%A7%E6%89%BF&spm=1018.2226.3001.4187)



# STL

广义上：

- 容器
- 算法
- 迭代器

容器和算法之间通过迭代器进行无缝连接

细分的六大组件：

- 容器：各种数据结构，用于存放数据
- 算法：各种常见算法
- 迭代器：扮演容器与算法的胶合剂
- 仿函数：类似函数，可以作为算法的某种策略
- 适配器（配接器）：修饰接口
- 空间配置器：负责空间的配置与管理

## 容器

容器，将运用最广泛的数据结构实现出来，如：数组，链表，树，栈，队列，集合，映射表

- 序列式容器：强调值的排序
- 关联式容器：二叉树结构

### 通用

#### emplace_back()

vector 、string、list等容器提供的所有成员函数，在这些成员函数中，可以用来给容器中添加元素的函数有 2 个，分别是 push_back() 和 emplace_back() 函数。

##### 用法

功能：和 push_back() 相同，都是在 vector 容器的尾部添加一个元素。

```cpp
template <class... Args>  
void emplace_back (Args&&... args);
```

##### 与push_back()的区别

- 是否考虑原地构造

  push_back()：向容器中加入一个右值元素(临时对象)时，首先会调用构造函数构造这个临时对象，然后需要调用拷贝构造函数将这个临时对象放入容器中。原来的临时变量释放。这样造成的问题就是临时变量申请资源的浪费。

  emplace_back()：引入了右值引用，转移构造函数，在插入的时候直接构造，只需要构造一次即可。

  也就是说，两者的底层实现的机制不同。push_back() 向容器尾部添加元素时，**首先会创建这个元素，然后再将这个元素拷贝或者移动到容器中**（如果是拷贝的话，事后会自行销毁先前创建的这个元素）；而 emplace_back() 在实现时，则是**直接在容器尾部创建这个元素**，省去了拷贝或移动元素的过程。

  ```cpp
  #include <vector> 
  #include <iostream> 
  using namespace std;
  class testDemo
  {
  public:
      testDemo(int num):num(num){
          std::cout << "调用构造函数" << endl;
      }
      testDemo(const testDemo& other) :num(other.num) {
          std::cout << "调用拷贝构造函数" << endl;
      }
      testDemo(testDemo&& other) :num(other.num) {
          std::cout << "调用移动构造函数" << endl;
      }
  private:
      int num;
  };
  
  int main()
  {
      cout << "emplace_back:" << endl;
      std::vector<testDemo> demo1;
      demo1.emplace_back(2);  
  
      cout << "push_back:" << endl;
      std::vector<testDemo> demo2;
      demo2.push_back(2);
  }
  
  emplace_back:
  调用构造函数
  push_back:
  调用构造函数
  调用移动构造函数
  ```

  

  <table><tr><td bgcolor = gray>emplace_back() 的执行效率比 push_back() 高。</td></tr></table>

  - 考虑尾插左值和右值

[谈谈c++11 emplace_back](https://blog.csdn.net/weixin_45880571/article/details/119450328?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522167030735216782427467160%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=167030735216782427467160&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-119450328-null-null.142^v67^control,201^v4^add_ask,213^v2^t3_esquery_v1&utm_term=emplace_back&spm=1018.2226.3001.4187)





























### std::vector

<font color =sandybrown>包含头文件#include\<vector></font>

#### 概念

单段数组，可以动态扩展

**动态扩展**：

不是在原空间后继续接新空间，而是找更大的内存空间，将原数据拷贝到新空间，然后释放原空间。

![](/home/suyu/Typora/Language/image/C++/b1f11ac20bfe4160b357c5a150d95819.jpeg)

#### 构造函数

函数原型：

- `vector<T> v;` //采用模板实现类实现，默认构造函数

- `vector(v.begin(), v.end()); `//将v[begin(), end())区间中的元素拷贝给本身。

- `vector(n, elem); `//构造函数将n个elem拷贝给本身。

- `vector(const vector &vec); `//拷贝构造函数。

  

  ```cpp
  vector<int>v1;
  
  vector<int>v2(v1.begin(), v1.end());
  
  vector<int>v3(10,100);//10个100
  
  vector<int>v4(v3);
  ```

  

```cpp
vector<int> v;

//尾插数据
v.push_back(10);

//通过迭代器访问容器数据
vector<int>::itertor itBegin = v.begin();//起始迭代器，指向容器第一个元素
vector<int>::itertor itEnd = v.end();//结束迭代器，指向容器最后一个元素的下一个位置

//第一种遍历方式
while(itBegin != itEnd)
{
    cout<<*itBegin<<endl;
    itBegin++;
}

//第二种
for(vector<int>::iterator it = v.begin();it!=v.end();it++
    {
        cout<<*it<<endl;
    }
//第三种，类用算法
    #include<algorithm>
    void myPrint(int val){
        cout <<val<<endl;
    }
    for_each(v.begin(),v.end(),myPrint);
        
 //存放自定义数据类型
    class Person{
        public:
        Person(string name,int age)
        {
            this->m_Name =name;
            this->m_Age =age;
        }
        string m_Name;
        int m_Age;
    }
    void test(){
vector<Person> v;
        Person p1("aaa",10);
        
        v.push_bacl(p1);
    }
    for(vector<int>::iterator it = v.begin();it!=v.end();it++
    {
        cout<<"姓名"<<(*it).m_Name<<endl;
    }
```

#### 赋值操作

- `vector& operator=(const vector &vec);`//重载等号操作符
- `assign(beg, end);` //将[beg, end)区间中的数据拷贝赋值给本身。
- `assign(n, elem);` //将n个elem拷贝赋值给本身

#### 容量和大小

- `empty();` //判断容器是否为空
- `capacity();` //容器的容量
- `size();` //返回容器中元素的个数

- resize(int num); //重新指定容器的长度为num，若容器变长，则以默认值填充新位置。//如果容器变短，则末尾超出容器长度的元素被删除。

- resize(int num, elem); //重新指定容器的长度为num，若容器变长，则以elem值填充新位置。//如果容器变短，则末尾超出容器长度的元素被删除

#### 插入和删除

- `push_back(ele);` //尾部插入元素ele
- `pop_back();` //删除最后一个元素
- `insert(const_iterator pos, ele);` //迭代器指向位置pos插入元素ele
- `insert(const_iterator pos, int count,ele);`//迭代器指向位置pos插入count个元素ele
- `erase(const_iterator pos);` //删除迭代器指向的元素
- `erase(const_iterator start, const_iterator end);`//删除迭代器从start到end之间的元素
- `clear();` //删除容器中所有元素

#### 数据存取

- `at(int idx);` //返回索引idx所指的数据
- `operator[];` //返回索引idx所指的数据
- `front();` //返回容器中第一个数据元素
- `back();` //返回容器中最后一个数据元素

```cpp

#include <vector>

void printVector(vector<int>& v) {

	for (vector<int>::iterator it = v.begin(); it != v.end(); it++) {
		cout << *it << " ";
	}
	cout << endl;
}

//插入和删除
void test01()
{
	vector<int> v1;
	//尾插
	v1.push_back(10);
	v1.push_back(20);
	v1.push_back(30);
	v1.push_back(40);
	v1.push_back(50);
	printVector(v1);
	//尾删
	v1.pop_back();
	printVector(v1);
	//插入
	v1.insert(v1.begin(), 100);
	printVector(v1);

	v1.insert(v1.begin(), 2, 1000);
	printVector(v1);

	//删除
	v1.erase(v1.begin());
	printVector(v1);

	//清空
	v1.erase(v1.begin(), v1.end());
	v1.clear();
	printVector(v1);
}

int main() {

	test01();

	system("pause");

	return 0;
}

```

#### 接口

```cpp
//构造函数
vector<T> v ; //使用模板类，默认构造函数
vector(v.begin(),v.end()); //将[v.begin(),v.end())区间中的元素拷贝给本身
vextor(n,elem); //将n个elem拷贝给本身
vector(const vector &v) ; //拷贝构造函数

//赋值操作
vector& operator=(const vector &v); //重载赋值运算符
assign(v.begin(),v.end()); //将[v.begin(),v.end())区间中的元素赋值给本身
assign(n,elem); //将n个elem赋值给本身

//容量与大小
empty(); //判断容器是否为空，为空返回1，否则返回0
capacity(); //返回容器的容量
size(); //返回容器的大小,即容器中元素的个数
resize(int num); //重新指定容器的长度为num，若容器变长，则以默认值0填充新位置,如果容器变短，则末尾超过容器长度的元素被删除
resize(int num,int elem); //重新指定容器的长度为num，若容器变长，则以elem填充新位置,如果容器变短，则末尾超过容器长度的元素被删除

//resize特性:长赋值，短截断

//插入和删除
push_back(ele); //尾部插入元素ele
pop_back(); //删除最后一个元素
insert(const_iterator pos,ele); //在迭代器指向的位置pos处插入一个元素ele
insert(const_iterator pos,int count,ele); //在迭代器指向的位置pos处插入count个元素ele
erase(const_iterator pos); //删除迭代器指向的元素
erase(const_iterator begin,const_iterator end); //删除迭代器从begin到end之间的元素
clear(); //删除容器中所有元素

//数据存取
at(int idx); //返回索引idx所指的数据
operator[]; //返回[]内索引所指的数据
front(); //返回容器中第一个元素
back(); //返回容器中最后一个元素

//互换容器
swap(v); //容器v和当前容器互换

//预留空间
reserve(int len);//容器预留len个元素长度，也就是把容量扩为len，预留的位置并不初始化，同时也不可访问

```

> ```cpp
> //为了方便测试，我们先定义一个打印输出的函数，利用迭代器实现，这个函数下面会经常使用，迭代器可以理解为指针
> 
> void printVector(vector<int>& v)
> {	//利用迭代器打印 v
> 	for (vector<int>::iterator it = v.begin(); it != v.end(); ++it)
> 	{
> 		cout << *it << " ";
> 	}
> 	cout << endl;
> }
> ```







### std::pair

<font color =sandybrown>包含头文件#include\<utility></font>

#### 概念

`std::pair`是将2个数据组合成一个数据，如STL中的`map`就是将`key`和`value`放在一起来保存。另一个应用是，当一个函数需要返回2个数据的时候，可以选择`pair`。 **pair的实现是一个结构体，主要的两个成员变量是`first `,`second`** 。因为是使用struct不是class，所以可以直接使用pair的成员变量。

#### 构造

```cpp
 pair<int, double> p1;  //使用默认构造函数
 pair<int, double> p2(1, 2.4);  //用给定值初始化
 pair<int, double> p3(p2);  //拷贝构造函数
```

也可以使用`std::make_pair`函数，`make_pair`函数的定义如下：

```cpp
template pair make_pair(T1 a, T2 b) { return pair(a, b); }
```

一般make_pair都使用在需要pair做参数的位置，可以直接调用make_pair生成pair对象。 另一个使用的方面就是pair可以接受隐式的类型转换，这样可以获得更高的灵活度。但是这样会出现如下问题：

```cpp
std::pair<int, float>(1, 1.1);
std::make_pair(1, 1.1);
```

**其中第一个的second变量是float类型，而make_pair函数会将second变量都转换成double类型。**这个问题在编程是需要引起注意。

eg1：

```cpp
#include <iostream>
 
#include <utility>
#include <string>
using namespace std;
 
 
int main () 
{
    std::pair <string,double> product1 ("tomatoes",3.25);
    std::pair <string,double> product2;
    std::pair <string,double> product3;
 
    product2.first = "lightbulbs"; // type of first is string
    product2.second = 0.99; // type of second is double
    product3 = make_pair ("shoes",20.0);
 
    cout << "The price of " << product1.first << " is $" << product1.second << "\n";
    cout << "The price of " << product2.first << " is $" << product2.second << "\n";
    cout << "The price of " << product3.first << " is $" << product3.second << "\n";
    return 0;
}
```

#### 赋值

```cpp
 //利用make_pair
pair<int, double> p1;
p1 = make_pair(1, 1.2);
//变量间赋值：
pair<int, double> p1(1, 1.2);
pair<int, double> p2 = p1;
```



#### 访问

```cpp
pair<int, double> p1;  //使用默认构造函数
p1.first = 1;
p1.second = 2.5;
cout << p1.first << ' ' << p1.second << endl;

```





### std::list

#### 概念

功能：将数据进行链式存储

**链表**（list）是一种物理存储单元上非连续的存储结构，数据元素的逻辑顺序是通过链表中的指针链接实现的

链表的组成：链表由一系列**结点**组成

结点的组成：一个是存储数据元素的**数据域**，另一个是存储下一个结点地址的**指针域**

STL中的链表是一个双向循环链表

![img](https://img-blog.csdnimg.cn/1e1793e32de84c3797993fde0d045214.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAd2VjaGFubm4=,size_19,color_FFFFFF,t_70,g_se,x_16#pic_center)

![image-20221118183515721](/home/suyu/.config/Typora/typora-user-images/image-20221118183515721.png)

由于链表的存储方式并不是连续的内存空间，因此链表list中的迭代器只支持前移和后移，属于**双向迭代器**

list的优点：

- 采用动态存储分配，不会造成内存浪费和溢出
- 链表执行插入和删除操作十分方便，修改指针即可，不需要移动大量元素

list的缺点：

- 链表灵活，但是空间(指针域) 和 时间（遍历）额外耗费较大

List有一个重要的性质，插入操作和删除操作都不会造成原有list迭代器的失效，这在vector是不成立的。

总结：STL中**List和vector是两个最常被使用的容器**，各有优缺点

#### 构造函数

函数原型：

- `list<T> lst`; //list采用采用模板类实现,对象的默认构造形式：
- `list(beg,end)`; //构造函数将[beg, end)区间中的元素拷贝给本身。
- `list(n,elem)`; //构造函数将n个elem拷贝给本身。
- `list(const list &lst)`; //拷贝构造函数。

```cpp
#include <list>

void printList(const list<int>& L) {

	for (list<int>::const_iterator it = L.begin(); it != L.end(); it++) {
		cout << *it << " ";
	}
	cout << endl;
}

void test01()
{
	list<int>L1;
	L1.push_back(10);
	L1.push_back(20);
	L1.push_back(30);
	L1.push_back(40);

	printList(L1);

	list<int>L2(L1.begin(),L1.end());
	printList(L2);

	list<int>L3(L2);
	printList(L3);

	list<int>L4(10, 1000);
	printList(L4);
}

int main() {

	test01();

	system("pause");

	return 0;
}

```

总结：list构造方式同其他几个STL常用容器，熟练掌握即可

#### 赋值与交换

功能描述：

- 给list容器进行赋值，以及交换list容器

函数原型：

- `assign(beg, end)`; //将[beg, end)区间中的数据拷贝赋值给本身。
-  `assign(n, elem)` ; //将n个elem拷贝赋值给本身。
- `list& operator=(const list &lst)`; //重载等号操作符
- `swap(lst)`; //将lst与本身的元素互换。

```cpp
#include <list>

void printList(const list<int>& L) {

	for (list<int>::const_iterator it = L.begin(); it != L.end(); it++) {
		cout << *it << " ";
	}
	cout << endl;
}

//赋值和交换
void test01()
{
	list<int>L1;
	L1.push_back(10);
	L1.push_back(20);
	L1.push_back(30);
	L1.push_back(40);
	printList(L1);

	//赋值
	list<int>L2;
	L2 = L1;
	printList(L2);

	list<int>L3;
	L3.assign(L2.begin(), L2.end());
	printList(L3);

	list<int>L4;
	L4.assign(10, 100);
	printList(L4);

}

//交换
void test02()
{

	list<int>L1;
	L1.push_back(10);
	L1.push_back(20);
	L1.push_back(30);
	L1.push_back(40);

	list<int>L2;
	L2.assign(10, 100);

	cout << "交换前： " << endl;
	printList(L1);
	printList(L2);

	cout << endl;

	L1.swap(L2);

	cout << "交换后： " << endl;
	printList(L1);
	printList(L2);

}

int main() {

	//test01();

	test02();

	system("pause");

	return 0;
}

```



#### 大小操作

功能描述：

对list容器的大小进行操作
函数原型：

- `size()`; //返回容器中元素的个数

- `empty()`; //判断容器是否为空

- `resize(num)`; //重新指定容器的长度为num，若容器变长，则以默认值填充新位置。 //如果容器变短，则末尾超出容器长度的元素被删除。

- `resize(num, elem)`; //重新指定容器的长度为num，若容器变长，则以elem值填充新位置。

```cpp
#include <list>

void printList(const list<int>& L) {

	for (list<int>::const_iterator it = L.begin(); it != L.end(); it++) {
		cout << *it << " ";
	}
	cout << endl;
}

//大小操作
void test01()
{
	list<int>L1;
	L1.push_back(10);
	L1.push_back(20);
	L1.push_back(30);
	L1.push_back(40);

	if (L1.empty())
	{
		cout << "L1为空" << endl;
	}
	else
	{
		cout << "L1不为空" << endl;
		cout << "L1的大小为： " << L1.size() << endl;
	}

	//重新指定大小
	L1.resize(10);
	printList(L1);

	L1.resize(2);
	printList(L1);
}

int main() {

	test01();

	system("pause");

	return 0;
}

```

总结：

- 判断是否为空 — empty
- 返回元素个数 — size
- 重新指定个数 — resize

#### 插入和删除

功能描述：

对list容器进行数据的插入和删除
函数原型：

- `push_back(elem)`;//在容器尾部加入一个元素
- `pop_back()`;//删除容器中最后一个元素
- `push_front(elem)`;//在容器开头插入一个元素
- `pop_front()`;//从容器开头移除第一个元素
- `insert(pos,elem)`;//在pos位置插elem元素的拷贝，返回新数据的位置。
- `insert(pos,n,elem)`;//在pos位置插入n个elem数据，无返回值。
- insert(pos,beg,end);//在pos位置插入[beg,end)区间的数据，无返回值。
- `clear()`;//移除容器的所有数据
- `erase(beg,end)`;//删除[beg,end)区间的数据，返回下一个数据的位置。
- `erase(pos)`;//删除pos位置的数据，返回下一个数据的位置。
- `remove(elem)`;//删除容器中所有与elem值匹配的元素。

总结：

- 尾插 — push_back
- 尾删 — pop_back
- 头插 — push_front
- 头删 — pop_front
- 插入 — insert
- 删除 — erase
- 移除 — remove
- 清空 — clear

#### 数据存取

**功能描述：**

- 对list容器中数据进行存取

**函数原型：**

- `front();` //返回第一个元素。
- `back();` //返回最后一个元素。

总结：

- list容器中不可以通过[]或者at方式访问数据

  *愿意list本质是链表，不是连续的线性空间存储数据，迭代器也不支持随机访问*

- 返回第一个元素 — front

- 返回最后一个元素 — back

#### 反转和排序

**功能描述：**

- 将容器中的元素反转，以及将容器中的数据进行排序

**函数原型：**

- `reverse();` //反转链表
- `sort();` //链表排序

#### 排序案例

```cpp
#include <list>
#include <string>

class Person {
public:
	Person(string name, int age , int height) {
		m_Name = name;
		m_Age = age;
		m_Height = height;
	}

public:
	string m_Name;  //姓名
	int m_Age;      //年龄
	int m_Height;   //身高
};


bool ComparePerson(Person& p1, Person& p2) {

	if (p1.m_Age == p2.m_Age) {
		return p1.m_Height  > p2.m_Height;
	}
	else
	{
		return  p1.m_Age < p2.m_Age;
	}

}


void test01() {

	list<Person> L;

	Person p1("刘备", 35 , 175);
	Person p2("曹操", 45 , 180);
	Person p3("孙权", 40 , 170);
	Person p4("赵云", 25 , 190);
	Person p5("张飞", 35 , 160);
	Person p6("关羽", 35 , 200);

	L.push_back(p1);
	L.push_back(p2);
	L.push_back(p3);
	L.push_back(p4);
	L.push_back(p5);
	L.push_back(p6);

    for(list<Person>::iterator it = L.begin();it != L.end();it++){
        cout << "姓名： " << it->m_Name << " 年龄： " << it->m_Age 
              << " 身高： " << it->m_Height << endl;
    }
    cout << "---------------------------------" << endl;
	L.sort(ComparePerson); //排序

	for (list<Person>::iterator it = L.begin(); it != L.end(); it++) {
		cout << "姓名： " << it->m_Name << " 年龄： " << it->m_Age 
              << " 身高： " << it->m_Height << endl;
	}
}

int main() {

	test01();

	system("pause");

	return 0;
}
```



## 算法

- 质变算法：运算中会更改区间内的元素内容，如拷贝，替换，删除

- 非质变算法：不会更改区间内的元素内容，如查找，计数，遍历，寻找极值

## 迭代器

提供一种方法，使之能够寻访某个容器所含的各个元素，但由不暴露容器内部表示方式。每个容器都由自己的迭代器。类似指针
