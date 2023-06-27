# C++类型判断

`typeid` 是 C++ 中的一个操作符，它允许你在运行时获取对象的类型信息。它是 C++ 中运行时类型信息（RTTI）特性的一部分。`typeid` 操作符返回一个 `std::type_info` 对象，表示所提供表达式的类型。

以下是使用 `typeid` 的一般语法：

```cpp
cppCopy code#include <typeinfo>

typeid(表达式)
```

通过调用 `typeid` 并传入相应的表达式，你可以获取该表达式的类型信息。

`typeid` 操作符可以用于自定义类型的判断。你可以通过传递一个对象或表达式给 `typeid` 操作符来获取其类型信息，无论是内置类型还是自定义类型。

下面是一个示例，展示了如何在 C++ 中使用 `typeid` 来判断自定义类型：

```cpp
cppCopy code#include <iostream>
#include <typeinfo>

class MyClass {
    // 类定义
};

int main() {
    MyClass obj;

    const std::type_info& type = typeid(obj);

    std::cout << "Type: " << type.name() << std::endl;

    if (type == typeid(MyClass)) {
        std::cout << "obj 是 MyClass 类型" << std::endl;
    } else {
        std::cout << "obj 不是 MyClass 类型" << std::endl;
    }

    return 0;
}
```

在上面的示例中，我们创建了一个名为 `MyClass` 的自定义类，并创建了一个 `MyClass` 类型的对象 `obj`。然后，我们使用 `typeid` 操作符获取 `obj` 的类型信息，并与 `typeid(MyClass)` 进行比较来判断它是否是 `MyClass` 类型。

请注意，`typeid` 操作符返回的类型信息是在编译时确定的，因此它对于多态类型（使用虚函数）可能不会返回预期的结果。如果你需要在运行时处理多态类型的对象，通常更适合使用虚函数和动态类型转换。