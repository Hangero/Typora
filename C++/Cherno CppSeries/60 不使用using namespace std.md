# 不使用`using namespace std`

1.不容易分辨各类函数的来源

> 比如我在一个自己的库中定义了一个vector，而标准库里又有一个vector，那么如果用了using namespace std 后，所用的vector到底是哪里的vector呢？

```cpp
std::vector<int>vec1;   //good
DiyClass::vector<int>vec2   //good

using namespace std;
using namespace DiyClass    //万一有其他人用了DiyClass的命名空间
vector<int>vec3 //便会有歧义，完全不知道到底是哪里的vector
```

2.一定**不要**在**头文件内**使用`using namespace std`

> 如果别人用了你的头文件，就会把这些命名空间用在了你原本没有打算用的地方，会导致莫名其妙的产生bug，如果有大型项目，追踪起来会很困难。 如果公司有自己的模板库，然后里面有很多重名的类型或者函数，就容易弄混；

3.可以就在一些小作用域里用，**但能不用就不用！养成良好的代码书写习惯！**