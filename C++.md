# C++



## 预备知识

### [ELF文件](https://blog.csdn.net/daide2012/article/details/73065204?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165820146616782246458941%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165820146616782246458941&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-73065204-null-null.142^v32^experiment_2_v1,185^v2^control&utm_term=elf%E6%96%87%E4%BB%B6&spm=1018.2226.3001.4187)

## 代码规范

>Always code as if the guy who ends up maintaining your code will be a violent psychopath who knows where you live.
>
>​                                                                                                                                   ——John F.Woods,1991

### 1.#pragma

C/C++中，在使用[预编译](https://so.csdn.net/so/search?q=预编译&spm=1001.2101.3001.7020)指令**#include**的时候，为了防止重复引用造成二义性。

pragma once则由编译器提供保证：同一个文件不会被编译多次。注意这里所说的“同一个文件”是指物理上的一个文件，而不是指内容相同的两个文件。带来的好处是，你不必再费劲想个宏名了，当然也就不会出现宏名碰撞引发的奇怪问题。对应的缺点就是如果某个头文件有多份拷贝，本方法不能保证他们不被重复包含。当然，相比宏名碰撞引发的“找不到声明”的问题，重复包含更容易被发现并修正。