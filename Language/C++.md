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

### 智能指针

在c++中，动态内存的管理式通过一对运算符来完成的：new,在动态内存中为对象分配空间并返回一个指向该对象的指针，我们可以选择对对象进行初始化；delete，接受一个动态对象的指针，销毁该对象，并释放与之关联的内存。动态内存的使用很容易出现问题，因为确保在正确的时间释放内存是极其困难的。有时使用完对象后，忘记释放内存，造成内存泄漏的问题。

- 所谓的**智能指针本质就是一个类模板**，它可以创建任意的类型的指针对象，当智能指针对象使用完后，**对象就会自动调用析构函数去释放该指针所指向的空间**。

#### 使用和定义



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

- 不要返回局部变量
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
    prot
        ected:
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

### vector

<font color =sandybrown>包含头文件#include<vector></font>

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



## 算法

- 质变算法：运算中会更改区间内的元素内容，如拷贝，替换，删除

- 非质变算法：不会更改区间内的元素内容，如查找，计数，遍历，寻找极值

## 迭代器

提供一种方法，使之能够寻访某个容器所含的各个元素，但由不暴露容器内部表示方式。每个容器都由自己的迭代器。类似指针
