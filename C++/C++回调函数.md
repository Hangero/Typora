# C++回调函数

参考文献：

[C++中回调函数使用详解方法大全](https://blog.csdn.net/wangjianbo09/article/details/107358935?ops_request_misc=&request_id=&biz_id=102&utm_term=c++%E7%9B%B8%E6%9C%BA%E5%9B%9E%E8%B0%83%E5%87%BD%E6%95%B0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-2-107358935.142^v72^wechat,201^v4^add_ask&spm=1018.2226.3001.4187)

[c++回调函数](https://blog.csdn.net/xiachong27/article/details/112554438?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522167534187016782427481105%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=167534187016782427481105&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-2-112554438-null-null.142^v72^wechat,201^v4^add_ask&utm_term=c%2B%2B%E7%9B%B8%E6%9C%BA%E5%9B%9E%E8%B0%83%E5%87%BD%E6%95%B0&spm=1018.2226.3001.4187)

[c++11 回调函数（以相机SDK采集图像的方式进行讲解）](https://blog.csdn.net/imv123/article/details/121767440?ops_request_misc=&request_id=&biz_id=102&utm_term=%E7%9B%B8%E6%9C%BA%E5%92%8C%E5%9B%9E%E8%B0%83%E5%87%BD%E6%95%B0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-5-121767440.142^v72^wechat,201^v4^add_ask&spm=1018.2226.3001.4187)

[指针函数和函数指针](https://blog.csdn.net/u010280075/article/details/88914424?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522167534587616800215035959%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=167534587616800215035959&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-88914424-null-null.142^v72^wechat,201^v4^add_ask&utm_term=%E5%87%BD%E6%95%B0%E6%8C%87%E9%92%88&spm=1018.2226.3001.4187)

## 函数指针

指针是一个变量，是用来指向[内存](https://so.csdn.net/so/search?q=内存&spm=1001.2101.3001.7020)地址的。一个程序运行时，所有和运行相关的物件都是需要加载到内存中，这就决定了程序运行时的任何物件都可以用指针来指向它。函数是存放在内存代码区域内的，它们同样有地址，因此同样可以用指针来存取函数，**把这种指向函数入口地址的指针称为函数指针**。

> 其声明形式如下所示：
>
> ```cpp
> ret (*p)(args, ...);
> ```
>
> 其中，`ret`为返回值，`*p`作为一个整体，代表的是指向该函数的指针，`args`为形参列表。其中`p`被称为**函数指针变量** 。

对于一个Hello World程序：

```cpp
int main(int argc,char* argv[])
{
    printf("Hello World!\n");
    return 0;
}
```

采用函数调用的形式来实现：

```cpp
void Invoke(char* s);
 
int main(int argc,char* argv[])
{
    Invoke("Hello World!\n");
    return 0;
}
 
void Invoke(char* s)
{
    printf(s);
}
```

用函数指针的方式来实现：

```cpp
void Invoke(char* s);
 
int main()
{
    void (*FunPointer)(char* s);    //声明一个函数指针(FunPointer)        
    FunPointer=Invoke;              //将Invoke函数的入口地址赋值给FunPointer
    FunPointer("Hello World!\n");   //函数指针fp实现函数调用
    return 0;
}
 
void Invoke(char* s)
{
    printf(s);
}
```

由上知道：函数指针函数的声明之间唯一区别就是，用指针名`（*FunPointer）`代替了函数名Invoke，这样这声明了一个函数指针，然后进行赋值`FunPointer=Invoke`就可以进行函数指针的调用了。声明函数指针时，只要函数返回值类型、参数个数、参数类型等保持一致，就可以声明一个函数指针了。注意，函数指针必须用括号括起来 `void (*FunPointer)(char* s)`。

> 实际中，为了方便，通常用宏定义的方式来声明函数指针，实现程序如下：
>
> ```cpp
> typedef void (*FP)(char* s);
> void Invoke(char* s);
>  
> int main(int argc,char* argv[])
> {
>     FP fp;      //通常是用宏FP来声明一个函数指针fp
>     fp=Invoke;
>     fp("Hello World!\n");
>     return 0;
> }
>  
> void Invoke(char* s)
> {
>     printf(s);
> }
> ```
>
> 

## 函数指针数组

```cpp
#include <iostream>
#include <string>
using namespace std;
 
typedef void (*FP)(char* s);
void f1(char* s){cout<<s;}
void f2(char* s){cout<<s;}
void f3(char* s){cout<<s;}
 
int main(int argc,char* argv[])
{
    void* a[]={f1,f2,f3};   //定义了指针数组，这里a是一个普通指针
    a[0]("Hello World!\n"); //编译错误，指针数组不能用下标的方式来调用函数
 
    FP f[]={f1,f2,f3};      //定义一个函数指针的数组，这里的f是一个函数指针
    f[0]("Hello World!\n"); //正确，函数指针的数组进行下标操作可以进行函数的间接调用
    
    return 0;
}
```



## 回调函数

假设有AB两个类，B类是我们自己写的应用类，A类为第三方类相机类，如下：

<img src="https://img-blog.csdnimg.cn/09dc4b047c4748d1905f5815d4bf78d3.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBA6Iyc6Iyc5Zi76Iyc6Iyc,size_20,color_FFFFFF,t_70,g_se,x_16" alt="img" style="zoom: 67%;" />

对于应用类实时的到相机采集到的图片，我们有两种方法：

- 通过循环轮询A类对象调用相关成员函数获取到实时图像。但会造成资源浪费，而且占用很大的CPU资源。
- 只有A类采集到图像时才去通知B类去获取图像。这就是回调函数。

回调函数，就是使用者自己定义一个函数，把这个函数作为参数传入其他函数中，由其他函数在运行时来调用的函数。函数是你实现的，但由其他函数调用（通过参数传递）简单来说，**就是由别人的函数运行期间来回调你实现的函数。**

本例中回调函数主要过程就是：把B类的函数注册（传递函数指针）给 A类，当A类采集到图像，就调用刚注册进来的B类函数，这样就间接完成了，由A类去通知B类去做某事。



<img src="https://img-blog.csdnimg.cn/66a88147b27a42a68395132228dbd424.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBA6Iyc6Iyc5Zi76Iyc6Iyc,size_20,color_FFFFFF,t_70,g_se,x_16" alt="img" style="zoom:67%;" />



标准Hello World程序：

```cpp
int main(int argc,char* argv[])
{
    printf("Hello World!\n")；
    return 0;
}
```

修改成函数回调样式：

```cpp
//定义回调函数
void PrintfText() 
{
    printf("Hello World!\n");
}
 
//定义实现回调函数的"调用函数"
void CallPrintfText(void (*callfuct)())
{
    callfuct();
}
 
//在main函数中实现函数回调
int main(int argc,char* argv[])
{
    CallPrintfText(PrintfText);
    return 0;
}
```

修改成带参的回调样式：

```cpp
//定义带参回调函数
void PrintfText(char* s) 
{
    printf(s);
}
 
//定义实现带参回调函数的"调用函数"
void CallPrintfText(void (*callfuct)(char*),char* s)
{
    callfuct(s);
}
 
//在main函数中实现带参的函数回调
int main(int argc,char* argv[])
{
    CallPrintfText(PrintfText,"Hello World!\n");
    return 0;
}
```



对于相机：

```cpp
#include <iostream>
#include <functional>
using namespace std;

/*回调函数原型声明*/
typedef function<void(int)> CALLBACK;

/*相机SDK底层A类*/
class A_Camera
{
public:
	void regeditCallBack(CALLBACK fun)/*注册回调函数*/
	{
		_fun = fun;
	}

	void getFrame()/*内部获取图像函数（B类调用者不需要关心它什么时候会执行）*/
	{
		/*采集到一帧数据_frame*/
		/****内部操作***/
		/***内部操作***/

		_frame = rand() % 10;
		_fun(_frame);/*回传给B_My类*/
	}

private:
	int _frame;
	CALLBACK _fun;
};

/*应用层B类*/
class B_My
{
public:
	void callBackFun(int frame)/*获取到A类的图像，此时frame就是一帧数据*/
	{
		cout << "B类获取到一帧数据：" << frame << endl;
	}
};

int main(int argc, char **argv)
{
	/*声明应用层B类对象*/
	B_My B; 

	auto Fun = bind(&B_My::callBackFun, B, placeholders::_1);/*中转一下,利用C++11特性*/

	/*声明底层相机A类*/
	A_Camera camera;
	camera.regeditCallBack(Fun);/*把B类的方法注册给A类*/

	/*以下只是模拟A类内部触发获取到图片，一共模拟触发10次*/
	for (int i = 0; i < 10; ++i)
	{
		camera.getFrame();
	}

	return 0;
}
```































