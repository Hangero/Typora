# C++隐式转换与explicit关键字

**隐式转换**

> 隐式转换**只能进行一次**。

```cpp
#include <iostream>

class Entity
{
private:
    std::string m_Name;
    int m_Age;
public:
    Entity(const std::string& name)
        : m_Name(name), m_Age(-1) {}

    Entity(int age)
        : m_Name("Unknown"), m_Age(age) {}
};

int main()
{
    Entity test1("lk");
    Entity test2(23); 
    Entity test3 = "lk"; //error!只能进行一次隐式转换
    Entity test4 = std::string("lk");
    Entity test5 = 23; //发生隐式转换


    std::cin.get();
}
```

如上，在test5中，int型的23就被隐式转换为一个Entity对象，这是**因为Entity类中有一个Entity(int age)构造函数，因此可以调用这个构造函数，然后把23作为他的唯一参数，就可以创建一个Entity对象。**

同时我们也能看到，对于语句`Entity test3 = "lk";`会报错，原因是**只能进行一次隐式转换**，`"lk"`是`const char`数组，这里需要先转换为`std::string`，再从string转换为Entity变量，两次隐式转换是不行的，所以会报错。但是写为`Entity test4 = std::string("lk");`就可以进行隐式转换。

最好不写`Entity test5 = 23;`这样的函数，应尽量避免隐式转换。因为`Entity test2(23);`更清晰。



**explicit 关键字**

- explicit是用来当你想要显示地调用构造函数，而不是让C++编译器隐式地把任何整形转换成Entity
- 我有时会在数学运算库的地方用到explicit，因为我不想把数字和向量来比较。一般explicit很少用到。
- 如果你在构造函数前面加上explicit，这意味着这个构造函数不会进行隐式转换
- 如果你想用一个整数构造一个Entity对象，那你就必须显示的调用这个构造函数，**explicit会禁用隐式转换**，explicit关键字放在构造函数前面

```cpp
#include <iostream>
  class Entity
  {
  private:
    std::string m_Name;
    int m_Age;
  public:
    Entity(const std::string& name)
        : m_Name(name), m_Age(-1) {}

    explicit Entity(int age)  //声明为explicit
        : m_Name("Unknown"), m_Age(age) {}
  };

  int main()
  {
    Entity test1("lk");
    Entity test2(23); 
      Entity test3 = "lk"; 
    Entity test4 = std::string("lk");
    Entity test5 = 23; //error！禁用隐式转换


    std::cin.get();
  }
```

加了explicit后还想隐式转换，则可以：

```cpp
Entity test5 = (Entity)23; //ok
```