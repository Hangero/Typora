## 栈帧

[【详解】函数栈帧——多图（c语言）](https://blog.csdn.net/Zero__two_/article/details/120781099?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522167031174916782428658009%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=167031174916782428658009&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-120781099-null-null.142^v67^control,201^v4^add_ask,213^v2^t3_esquery_v1&utm_term=%E6%A0%88%E5%B8%A7&spm=1018.2226.3001.4187)

### 函数栈帧

**C语言中，每个栈帧对应着一个未运行完的函数。栈帧中保存了该函数的返回地址和局部变量。**

通过这句话我们可以提炼出两个关键信息：

> 1.每个未运行完的函数都有一个对应的栈帧
>
> 2.栈帧保存了函数的返回地址和局部变量

### 栈帧准备知识

#### 内存分区

内存中主要分为栈区，堆区，静态区，以及其他部分。

>栈区：由高地址往低地址增长，主要用来存放局部变量，函数调用开辟的空间，与堆共享一段空间。（本篇重点）
>
>堆区：由低地址向高地址增长，动态开辟的空间就在这里（malloc，realloc，calloc，free），与栈共享一段空间。
>
>静态区：主要存放全局变量和静态变量。 
>

<img src="https://img-blog.csdnimg.cn/ace2defc08c3421fa8307eb306535b69.PNG?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAWmVybzBUdzA=,size_20,color_FFFFFF,t_70,g_se,x_16" alt="img" style="zoom: 25%;" />

什么是栈

前面已经知道栈中存放了函数调用开辟的空间即栈帧，因此我们要明白什么是栈帧，必须先知道什么是栈。

栈是一种数据结构，是一种只能在一端进行插入和删除操作的特殊线性表。它按照先进后出的原则存储数据，先进入的数据被压入栈底，最后的数据在栈顶，需要读数据的时候从栈顶开始弹出数据（最后放入的数据被最先读出来）。**这就是栈最大的特点"先入后出，后入先出"，而往栈中放数据我们称作压栈（push），拿出栈中的数据我们叫出栈（pop）。**

#### esp，ebp，eax寄存器

| ebp  | ebp是基址指针，保存调用者函数的地址，总是指向当前栈帧栈底 |
| ---- | --------------------------------------------------------- |
| esp  | esp是被调函数指针，总指向函数栈栈顶                       |
| eax  | 累加器，用来乘除法，与函数返回值(本篇主要关注第二个功能） |

 **简单来讲就是esp和ebp是两个指针，ebp指向\**当前栈帧栈底\**，esp指向\**函数栈栈顶\**\**。\****

<img src="https://img-blog.csdnimg.cn/7aa6310c682e4db188656297550a1b73.PNG?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAWmVybzBUdzA=,size_20,color_FFFFFF,t_70,g_se,x_16" alt="img" style="zoom:25%;" />

**能看到，ebp并不是指向整个函数栈的栈底，而是指向当前栈帧的栈底，而由于esp总是指向栈顶，且栈只允许一个方向的操作，因此esp指向其实也是当前栈帧的栈顶，不过当前栈帧的栈顶始终与栈顶相同，因此说esp指向的是栈顶。**

### 栈帧创建与销毁全过程

假设有以下程序

```cpp
#include<stdio.h>
 
int add(int a, int b)
{
	int c = 0;
	c = a + b;
	return c;
}
 
int main()
{
	int a = 1;
	int b = 1;
	int sum;
	sum = add(a, b);
	return 0;
}
```

### 调用函数之前

**此时我们准备执行函数调用"sum = add(a,b);"此时栈中如下:**

<img src="https://img-blog.csdnimg.cn/7aa6310c682e4db188656297550a1b73.PNG?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAWmVybzBUdzA=,size_20,color_FFFFFF,t_70,g_se,x_16" alt="img" style="zoom:25%;" />

> ### 将传入函数的值放入栈中
>
> 由于函数调用涉及到传参，因此我们在调用函数之前，需要先将传入的参数保存，以方便函数的调用，因此需要将add函数的a=1,b=2,push入栈保存

<img src="https://img-blog.csdnimg.cn/5174a00f25d14d699c517425af954292.PNG?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAWmVybzBUdzA=,size_20,color_FFFFFF,t_70,g_se,x_16" alt="img" style="zoom:25%;" />



### 函数的执行

> ### 1.保护当前ebp
>
> 由于我们马上要创建新的栈帧空间，因此ebp和esp都得将变动，为了能够让我们调用完add函数后还能让ebp回到当前位置我们需要对ebp的值进行保护，即**将此时ebp的值压入栈**（至于为什么不需要保护esp，看到后面你就能明白）



<img src="https://img-blog.csdnimg.cn/5c4dcd41689c408ea9d14007df095d07.PNG?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAWmVybzBUdzA=,size_20,color_FFFFFF,t_70,g_se,x_16" alt="img" style="zoom:25%;" />

> ###  2.创建所需调用函数的栈帧空间
>
> 令ebp指向当前esp的位置并根据add函数的参数个数，创建一个大小合适的空间。

**①** **ebp指向当前esp的位置** 

<img src="https://img-blog.csdnimg.cn/03e0b76841dc4f97a5ef9c035169a120.PNG?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAWmVybzBUdzA=,size_20,color_FFFFFF,t_70,g_se,x_16" alt="img" style="zoom:25%;" />

**②创建空间**

<img src="https://img-blog.csdnimg.cn/383af7e1b8454ff198945f9b6015e567.PNG?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAWmVybzBUdzA=,size_20,color_FFFFFF,t_70,g_se,x_16" alt="img" style="zoom:25%;" />

> ###  3.保存局部变量
>
> 将add函数中创建的变量"int c = 0"放入刚刚开辟的栈帧空间中



<img src="https://img-blog.csdnimg.cn/c61112bc95f64c798d3681df2fdcb00d.PNG?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAWmVybzBUdzA=,size_20,color_FFFFFF,t_70,g_se,x_16" alt="img" style="zoom:25%;" />

> ###  4.参数运算
>
> 根据形参与局部变量，进行对应的运算，这里执行"c = a +b", 得到 c = 2,放入刚才c对应的位置。

![img](https://img-blog.csdnimg.cn/fafd1f5ad2b642e6b6d8bd99de3cfbed.PNG?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAWmVybzBUdzA=,size_20,color_FFFFFF,t_70,g_se,x_16)

### 函数返回

> ### 1.存储返回值
>
> 现在我们已经达成了目的"add(a,b)",要将之前创建的add的函数栈销毁，以使得我们能够回到main函数中正常执行，而在销毁add的函数栈帧前我们的main函数可还没有拿到运算结果，因此我们需要先将需要返回的值存储起来，存储的位置就是前面提到的eax寄存器，这里"return c",我们将c的值放到eax寄存器中。

<img src="https://img-blog.csdnimg.cn/cb212b9328e64f5ca23964d5d7e9d4fb.PNG?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAWmVybzBUdzA=,size_20,color_FFFFFF,t_70,g_se,x_16" alt="img" style="zoom:25%;" />

> ###  2.销毁空间
>
> 拿到了运算结果后，我们就没有任何任何顾虑了，可以直接销毁函数的栈桢空间了。

<img src="https://img-blog.csdnimg.cn/4dd66fd6557c4dc4ae4fda16da1ec8fa.PNG?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAWmVybzBUdzA=,size_20,color_FFFFFF,t_70,g_se,x_16" alt="img" style="zoom:25%;" />



> ### 3.ebp回上一栈帧栈底
>
> 此时ebp拿到之间存储的上一栈帧栈底的值，回到相应的位置，于此同时，存储的ebp没有用了，也将被销毁。



<img src="https://img-blog.csdnimg.cn/8aebb4d318a343238a512837b507eded.PNG?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAWmVybzBUdzA=,size_20,color_FFFFFF,t_70,g_se,x_16" alt="img" style="zoom:25%;" />



> ### 4.销毁形参
>
> 形参也不再有用，因此也随即销毁。（\**这里也让我们明白：由于形参在调用完函数后就会销毁，且与实参根本不是同一地址，因此形参的改变无法影响实参。\**）

<img src="https://img-blog.csdnimg.cn/ccd1a7c2bdec4ad8bfbfd4c1f4e8e0bc.PNG?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAWmVybzBUdzA=,size_20,color_FFFFFF,t_70,g_se,x_16" alt="img" style="zoom:25%;" />



> ###  5.main函数拿到返回值
>
> 在讲解main函数怎么拿到返回值前，我想先问一个问题：
>
> 上图中所谓的前一栈帧指的是什么？
>
> 大家都知道，我们编写的c程序都是从一个main函数开始的，实际上，代码并不是直接从main函数开始运行的，main函数的本质也是一个被其他代码调用的函数，至于被谁调用，这里就不展开讲解了，这里提出这个问题是想要大家知道：
>
> **main函数是一个函数，它有自己的栈帧。**
>
> **因此所谓的前一栈帧实际上就是调用add函数的main函数的栈帧。**
>
> 因此我们要让main函数拿到返回值，只需要把eax寄存器中的值放入main栈帧中sum对应的位置就行。（这里也能让我们明白：由于我们只有一个eax寄存器，因此c语言的函数只能有一个返回值。）
>







## 继承和多态

- C++中的类是什么？它在C++语言中扮演什么样的角色？
- 类的继承是什么？C++是如何利用继承的？
- 什么是运行时多态？以及如何在C++中使用它

> 《Effective C++》条款01：视C++为一个语言联邦
>
> - C
> - Object-Orient C++(C with Classes)
> - Template C++
> - STL

### **类和多态**

> 《Effective C++》
>
> 接口(interface)：当我使用术语“接口”时，我一般谈论的是函数的签名(signature)或class的可访问元素(如public接口，protected接口，private接口)，或是针对某temple类型参数需为有效的一个表达式

封装的概念是C++中类的核心，语言标准允许我们通过将成员设置为public使其对外暴露，或者设置为private将它们隐藏。一个优秀的设计通常有且仅有private数据成员，仅通过public方法对外提供数据访问的接口。这些public接口就像是一个合约，类的设计者允诺这个类将提供某些操作和功能。类的private成员及方法则作为实现的一部分，并且只要public接口有效，可以任意修改。举个例子，下面的类表达了一个有理数，并且支持增量操作，其暴露出的public接口如：

```cpp
 class Rational {
 public:
     Rational& operator+=(const Rational& rhs);
 };
```

一个良好设计的类不会通过public接口暴露非必要的实现细节，因为实现的细节不是“合约”的一部分，不过接口的文档有时倒是会提醒你接口会有哪些限制罢了。比方说，如果我们承诺所有的有理数中，分子和分母不存在公倍数，那么当两个有理数相加时，需要一个约分的步骤。这个步骤最好用private方法来实现它，因为他可能还会调用其他的细节操作。因此类的客户不需要去执行这些复杂的操作，因为在这个方法的结果返回给调用者时，private方法已经完成了约分：

```cpp
 class Rational {
 public:
     Rational& operator+=(const Rational& rhs);
 private:
     long n_; // 分子
     long d_; // 分母
     void reduce();
 };
 Rational& Rational::operator+=(const Rational& rhs) {
     n_ = n_*rhs.d_ + rhs.n_*d_;
     d_ = d_*rhs.d_;
     reduce();
     return *this;
 }
 Rational a, b;
 a += b;
```

类方法拥有数据成员的特殊访问权限，因此可以直接访问private数据成员。注意这里的**operator+=是类的成员函数，它是由a对象调用并作用于自身的**。如果一个成员函数通过“直呼其名”引用了一个数据成员，那么它访问的就是调用这个函数的对象中的那个数据成员，可以想象在这里编译器自动给我们显式地添加了`this->n`_和`this->d_`。访问另一个对象的的成员则需要通过指向另一个对象的指针或者引用来进行，不过对于非成员函数而言，这样的访问可能会收到访问权限控制的限制。

顺便一提，C++也支持C风格的结构体struct。但是在C++中，struct并不仅仅局限为一个数据的聚合体，它可以有成员方法，private和public访问权限控制，以及任何作为类可以有的属性。从语言的层面来看，类和对象的区别仅仅在于默认访问权限。在类（class）中，所有的成员默认是private访问权限，而struct是public访问权限。

> 类是属于Object-Orient C++(C with Classes)的，而结构体是属于C的

### **继承和派生**

> 方法即函数

在C++中，类的派生有两个主要的目的。一方面，它允许我们表达对象间的关系；另一方面，它允许我们通过许多更简单的类型来构建复杂的类型。这两种目的都是通过类的继承达成的。

继承是C++中使用类和对象的核心概念。继承允许我们扩展已有类来定义新的类。当一个类派生自另一个类的时候，某些情况下，可以包含父类的所有数据成员和算法，并且添加它自己的算法和数据成员。在C++中，区分private继承和public继承非常重要。

> private

#### **Public继承**

Public继承方式继承了类的公有接口，并且继承了所有实现——基类的数据成员也会成为派生类的一部分。**接口的继承是区分public继承和private继承的标志**，因为**公有继承会将基类的共有接口作为派生类共有接口的一部分对外暴露**，就像上面讲过的合约一样。

通过public继承自基类，并且遵循基类接口的限制条件，派生类就可以将其绑定在与基类相同的合约上，同时可以对合约进行适当的扩展和增补。由于public继承的派生类遵循了合约的规范，因此所有能用到基类的地方都可以用派生类来代替，但是这样就无法用到派生类对于基类的扩展特性了。

这个特点通常被称为“is-a”法则，一个派生类的实例也是一个基类的实例。但是我们在C++中对于“is-a”关系的阐释不能全凭直觉。例如，正方形是一种矩形嘛？如果它是，那么我们就可以尝试从Rectangle类中派生出Square类：

```cpp
 class Rectangle {
 public:
     double Length() const { return length_; }
     double Width() const { return width_; }
     //...
 private:
     double l_;
     double w_;
 };
 class Square : public Rectangle {
     //...
 };
```

但是我们马上就能发现有点不对头，派生类的正方形拥有两个数据成员，而现实中的正方形并不需要，因为只需要知道一条边长即可确定一个正方形。这个看起来并不是很糟糕，但是考虑一下如果我们在矩形类的“合约中”提供了任意缩放的功能：

```cpp
 class Rectangle {
 public:
     void Scale(double sl, double sw) { // 对长和宽按不同比例缩放
         length_ *= sl;
         width_ *= sw;
     }
 //...
 };
 
```

现在回头看，通过public继承而来的正方形也具有了这样的任意缩放性质。事实上，通过public继承，我们相当于承诺在能用矩形的地方都可以用正方形代替。很显然，这样的承诺条件并不能够被满足。当我们派生类的客户准备调整一个正方形的比例时，这种操作显然不合法。也许我们可以忽略这个调用，或者抛出运行时异常。但不论如何，我们都违背了“合约”的精神。对于C++来说，这种情况只有一种解决方案，那就是：正方形不是矩形。反之亦然，我们无法通过正方形能够提供的接口来派生出合法的矩形。

由于public继承隐含了“is-a”的关系，语言层面就允许我们对引用和指针进行宽泛地转化。首先，派生类的指针是可以隐式转换为基类指针的，对于引用也是如此：

```cpp
 class Base { ... };
 class Derived : public Base { ... };
 Derived* d = new Derived;
 Base* b = d; // 隐式转换
```

这样的转换总是合法的，因为**派生类的实例（public继承）一定是基类的实例**。反之则不一定，如果要转换必须显式转换：

```cpp
 Base* b = new Derived; // *b 是指向派生类的
 Derived* d = b; // 无法通过编译
 Derived* d = static_cast<Derived*>(b); // 显式强制转换
 
```

这样的转换无法通过隐式方式进行的原因是，如果要让这样的转换行为合法，那么基类的指针必须真实地指向一个派生类对象，否则这将会是未定义行为。作为程序员，必须显式地断言，使用static_cast，从逻辑上表示出程序员对于当前进行的转换是有意而为之并且确认合法的。如果不确定这样的转换是否合法，我们也有其他的途径可以尝试，下一小节将会介绍这样的转换（dynamic_cast）。

#### **Private继承**

C++的另一种类型的继承则是private继承。当我们声明一个private继承时，**派生类并不会对基类的公有接口进行扩展，相反，所有基类的方法都变成派生类的私有方法**了。派生类的所有public接口，都要从零开始编写。这样一来，派生类的对象就不能顺理成章替代基类原有的位置了。派生类所能得到的则是基类的所有内部实现，即算法和数据成员。这种继承方式通常来说也被成为“has-a”模式，也就是派生类中“拥有”一个基类的实例。

对于私有继承的派生类而言，基类之于派生类的关系就如同数据成员之于类的关系。通常如果没有特别的其他原因，类的组合是比私有继承更最合适的方式

那么，还有什么其他的理由会用到私有继承呢？还是有一些可能性的。

首先，在派生类中，可以通过`using`声明将一些基类的public成员方法重新暴露出去：

```cpp
 class Container : private std::vector<int> {
 public:
     using std::vector<int>::size;
     //...
 };
```

……

### **多态与虚函数**

我们讨论public继承时提到了，派生类可以在任何期望可以用到基类的地方做替换。即使存在这个要求，知晓实际对象的真实类型也是很有用的

```cpp
 Derived d;
 Base& b = d;
 //...
 b.some_method(); // b其实是派生类
```

`some_method()`是基类提供的public接口的一部分，因此在派生类中也必须有效。但是在基类接口提供的合约的灵活性规范内，我们可以对它进行适当的修改。例如，之前我们做过对飞行类鸟类的类型派生设计。假设FlyingBird类提供了`fly()`接口，每一种派生自该类的鸟类都同样具有这样的接口。但是，老鹰和秃鹫飞行的方式略有不同，也就是说，老鹰和秃鹫对`fly()`的实现可以有差异。任何对于任意`FlyingBird`类调用`fly()`接口的代码可能会得到不同的效果

```cpp
 class FlyingBird : public Bird {
 public:
     virtual void fly(double speed, double direction) {
         //... 某个飞行速度，高度等 ...
     }
     //...
 };
 
```

派生类继承了这个成员函数的声明和实现，如果这个实现满足了派生类的要求，那么就无需改动。但如果派生类需要修改实现，它可以通过覆写基类的实现来达成：

```cpp
 class Vulture : public FlyingBird {
 public:
     virtual void fly(double speed, double direction) {
     //... move the bird but accumulate exhaustion if too fast ...
     }
 };
 
```

当一个虚函数被调用的时候，C++的运行时系统就会判定对象的真实类型。通常，这类信息在编译时不可知，而必须在运行时决定：

```cpp
 void hunt(FlyingBird& b) {
 b.fly(...); // Could be Vulture or Eagle
     ...
 };
 Eagle e;
 hunt(e); // hunt参数b的真实类型是Eagle, FlyingBird::fly() 被调用
 Vulture v;
 hunt(v); // hunt参数b的真实类型是Vulture, Vulture::fly() 被调用
```

这种**在多个基类对象上调用同样方法但其运行结果不同的编程技巧，称为“运行时多态”**。在C++中，对象要具有多态性质必须至少拥有一个虚函数，并且只有提供了虚函数的接口才能实现多态的特性。

**在C++中，我们可以在基类中拒绝为虚函数提供实现。这样的函数我们称他为纯虚函数，任何包含纯虚函数的类都被称为抽象类：**

```cpp
 class FlyingBirt {
 public:
     virtual void fly(...) = 0; // 纯虚函数
 };
 
```

抽象类仅仅定义了接口，实现接口的任务就落在了派生类的身上。如果一个派生类的父类包含了纯虚函数，那么任何在此派生链上的类都必须对纯虚函数进行实现。换言之，拥有纯虚函数的类无法被实例化。然而，我们可以拥有指向真实派生类对象的基类指针或引用。

> C++语法中有一些小技巧，当我们覆写虚函数的时候，不要求明确写出virtual关键字。如果基类声明了一个虚函数，那么子类的同名、同参数的函数就自动具备了虚函数的属性，并且会覆盖基类的实现。注意，如果派生类中的同名函数参数列表与基类同名虚函数不同，那么这个函数不会覆盖任何东西，但是会遮蔽（shadow）基类中的同名函数。当程序员意图覆盖基类虚函数但没有抄对声明的时候，可能会导致很难发现的bug：
>
> ```cpp
>  class Eagle : public FlyingBird {
>  public:
>      virtual void fly(int speed, double direction);
>  };
>  
> ```
>
> 这里的参数类型与基类中不相同。尽管Eagle中的fly也是虚函数，但是它并没有覆写基类的虚函数。而由于基类中的`fly()`是纯虚函数，这个bug可以被发现，因为纯虚函数没有被实现。然而，如果基类提供了`fly()`的实现，那么编译器就无法发现这个隐秘的bug。

好在C++11中提供了override关键字，它可以极大地方便我们甄别出这类隐秘的bug，任何意图覆写基类的函数中都可以在声名时加入override关键词：

```cpp
 class Eagle : public FlyingBird {
 public:
     void fly(int speed, double direction) override;
 };
```

这里的virtual关键字是可选的，但是如果基类中不存在名为fly的虚函数，那么这样的代码也会触发编译错误。

当我们不确定基类指针指涉对象的真实类型时，使用static_cast是危险的。但是我们可以利用dynamic_cast来帮助判断。当我们用尝试转换一个基类指针到派生类指针时，如果转换的类型正确，那么我们就可以得到一个正确的派生类指针；否则，dynamic_cast就会返回一个空指针：

```cpp
 class Base { ... };
 class Derived : public Base { ... };
 Base* b1 = new Derived; // 真的派生类
 Base* b2 = new Base; // 不是派生类
 Derived* d1 = dynamic_cast<Derived*>(b1); // 成功转换
 Derived* d2 = dynamic_cast<Derived*>(b2); // 转换失败 d2 == nullptr
 
```

### **总结**

> public继承和private继承不是针对基类的成员类型而言，而是针对继承后基类的成员在派生类中的类型而言。pulic继承将基类的public继承为public，privat继承将基类的public继承为private；基类的private不会被继承。
>
> （派生类和基类的指针和引用）
>
> 随着基类的派生，派生类会对基类的函数接口有着更细更丰富的实现，为了满足接口相同（方便调用），实现不同（功能丰富），可以通过为基类的函数实现添加`virtual`关键字，表示是可以被覆盖重写的，这样的函数被称为**虚函数**，进一步，基类中拒绝为虚函数提供实现，这样的虚函数被称为**纯虚函数**，这样的类被称为**抽象类**。而由于函数重载的存在，若派生类中出现与基类同名但不同参数列表的函数，就会对函数进行重载而不是覆盖，为此，出现了`override`关键字，用于派生类，表示是对基类虚函数的覆盖，请求编译器检查。









