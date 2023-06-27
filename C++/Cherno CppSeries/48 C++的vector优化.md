# C++的vector优化

 vecctor的优化策略：

> **问题1：**当向vector数组中**添加新元素**时，为了扩充容量，**当前的vector的内容会从内存中的旧位置复制到内存中的新位置**(产生一次复制)，然后删除旧位置的内存。 简单说，push_back时，容量不够，会自动调整大小，重新分配内存。这就是将代码拖慢的原因之一。 **解决办法：** vertices.reserve(n) ，直接指定容量大小，避免重复分配产生的复制浪费。
> **问题2：**在非vector内存中创建对象进行初始化时，即push_back() 向容器尾部添加元素时，首先会创建一个临时容器对象（不在已经分配好内存的vector中）并对其追加元素，然后再将这个对象拷贝或者移动到【我们真正想添加元素的容器】中 。这其中，就造成了一次复制浪费。 **解决办法：** **emplace_back**，直接在容器尾部创建元素，即直接在已经分配好内存的那个容器中直接添加元素，不创建临时对象。

简单的说：

> **reserve提前申请内存**，避免动态申请开销 **emplace_back直接在容器尾部创建元素**，省略拷贝或移动过程

```cpp
#include <iostream>
#include <vector>

struct Vertex
{
    float x, y, z;

    Vertex(float x, float y, float z)
        : x(x), y(y), z(z)
    {
    }

    Vertex(const Vertex& vertex)
        : x(vertex.x), y(vertex.y), z(vertex.z)
    {
        std::cout << "Copied!" << std::endl;
    }
};

int main()
{
    std::vector<Vertex> vertices;
    vertices.push_back(Vertex(1, 2, 3 )); //同vertices.push_back({ 1, 2, 3 });
    vertices.push_back(Vertex(4, 5, 6 ));
    vertices.push_back(Vertex(7, 8, 9 ));

    std::cin.get();
}
```

输出：

```cpp
Copied!
Copied!
Copied!
Copied!
Copied!
Copied!
```

**发生六次复制的原因：**

理解一：

> 环境:VS2019，x64，C++17标准，经过我自己的测试，vector扩容因子为1.5，初始的capacity为0.
> 第一次push_back，capacity扩容到1，临时对象拷贝到真正的vertices所占内存中，第一次Copied；第二次push_back，发生扩容，capacity扩容到2，vertices发生内存搬移发生的拷贝为第二次Copied，然后再是临时对象的搬移，为第三次Copied；接着第三次push_back，capacity扩容到3（2*1.5 = 3，3之后是4，4之后是6...），vertices发生内存搬移发生的拷贝为第四和第五个Copied，然后再是临时对象的搬移为第六个Copied；

理解二：

```cpp
std::vector<Entity> e;
    Entity data1 = { 1,2,3 }; 
    e.push_back( data1); // data1->新vector内存
    Entity data2 = { 1,2,3 }; 
    e.push_back( data2 ); //data1->新vector内存   data2->vector新vector内存  删除旧vector内存
    Entity data3 = { 1,2,3 };
    e.push_back(data3);  // data1->新vector内存  data2->vector新vector内存  data3->vector新vector内存  删除旧vector内存
所以他的输出的次数分别是1，3，6
他的复制次数你可以这样理解递增。 1+2+3+4+5+....
```

解决:

```cpp
int main()
{   
    std::vector<Vertex> vertices;
    //ver 1 : copy 6 times
    vertices.push_back({ 1,2,3 });
    vertices.push_back({ 4,5,6 });
    vertices.push_back({ 7,8,9 });

    //ver 2 : copy 3 times
    vertices.reserve(3);
    vertices.push_back({ 1,2,3 });
    vertices.push_back({ 4,5,6 });
    vertices.push_back({ 7,8,9 });

    //ver 3 : copy 0 times
    vertices.reserve(3);
    vertices.emplace_back(1, 2, 3);
    vertices.emplace_back(4, 5, 6);
    vertices.emplace_back(7, 8, 9);

    std::cin.get();
}
```