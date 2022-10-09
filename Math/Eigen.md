# Eigen

*参考*

*[Eigen：基础入门到使用](https://blog.csdn.net/QLeelq/article/details/111599195?spm=1001.2014.3001.5506)*

Eigen是可以用来进行[线性](https://so.csdn.net/so/search?q=线性&spm=1001.2101.3001.7020)代数、矩阵、向量操作等运算的C++库，它里面包含了很多算法，支持多平台。

Eigen采用源码的方式提供给用户使用，在使用时只需要包含Eigen的[头文件](https://so.csdn.net/so/search?q=头文件&spm=1001.2101.3001.7020)即可进行使用。之所以采用这种方式，是因为Eigen采用模板方式实现，由于模板函数不支持分离编译，所以只能提供源码而不是动态库的方式供用户使用。

Eigen头文件的默认安装位置是：“/usr/include/eigen3”。

## 模块

| 模块        | 头文件                       | 内容                                                         |
| ----------- | ---------------------------- | ------------------------------------------------------------ |
| Core        | #include <Eigen/Core>        | Matrix和Array类，基础线性代数运算、数组操作                  |
| LU          | #include <Eigen/LU>          | 求逆、行列式、LU分解                                         |
| Cholesky    | #include <Eigen/Cholesky>    | LLT和LDLT的cholesky分解                                      |
| Householder | #include <Eigen/Householder> | householder变换                                              |
| SVD         | #include <Eigen/SVD>         | SVD分解                                                      |
| QR          | #include <Eigen/QR>          | QR分解                                                       |
| Geometry    | #include <Eigen/Geometry>    | 变换、平移、缩放、2D和3D的旋转（四元数，欧拉角）             |
| Eigenvalues | #include <Eigen/Eigenvalues> | 特征值、特征向量分解                                         |
| Sparse      | #include <Eigen/Sparse>      | 稀疏矩阵存储以及基本线性代数运算                             |
| Dense       | #include <Eigen/Dense>       | Core, Geometry, LU, Cholesky, SVD, QR, and Eigenvalues头文件 |
| Eigen       | #include <Eigen/Eigen>       | 包含Dense和Sparse头文件，也就是整个Eigen头文件               |

## 矩阵基础

### 矩阵和向量

1. matrix

```cpp
Matrix<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
```

- scalar：矩阵的类型，包括float、double、int、复数float
- RowsAtCompileTime：行数
- ColsAtCompileTime：列数

```cpp
typedef Matrix<float, 4, 4> Matrix4f;//4行4列，float
```

Eigen约定：”d”表示double类型，”f”表示float类型，”i”表示整数，”c”表示复数。

读取行和列的数量方法：

```cpp
 int r = matrix.rows();//行
 int c = matrix.cols();//列
```

2. 向量vectors

```cpp
typedef Matrix<float, 3, 1> Vector3f;//3行1列，float
typedef Matrix<int, 1, 2> RowVector2i;//1行，2列，int
```

### 动态矩阵

```cpp
typedef Matrix<double, Dynamic, Dynamic> MatrixXd;//n行，n列。double类型矩阵。
typedef Matrix<int, Dynamic, 1> VectorXi;//动态列向量，n行1列
```

### 定义

定义之前，应该写下使用空间或者在程序中手写添加Eigen，如下：

```cpp
using Eigen::MatrixXd;
using Eigen::Vector2d;
```

- 未初始化类型

```cpp
Matrix3f a;//3-by-3，float，未初始化
MatrixXf b;// n-by-n，float，未初始化
MatrixXf a(10,15);//10-by-15，float，未初始化
VectorXf b(30);//00-by-1，未初始化
Matrix3f a(3,3);//固定长度也可以这么写,3-by-3，float，未初始化
```

- 初始化类型

```cpp
Matrix3f m;//3-by-3，float
m << 1, 2, 3,
     4, 5, 6,
     7, 8, 9;//不换行也行，只是为了看着方便，因为是行优先
```

使用已知的来定义未知的矩阵：

```cpp
RowVectorXd vec1(3);//1-by-3, double，行向量
vec1 << 1, 2, 3;
RowVectorXd vec2(4);
vec2 << 1, 4, 9, 16;
RowVectorXd joined(7);
joined << vec1, vec2;//
```

类似的：

```cpp
MatrixXf matA(2, 2);
matA << 1, 2, 3, 4;
MatrixXf matB(4, 4);
matB << matA, matA/10, matA/10, matA;

/*matB输出：
1   2   0.1  0.2
3   4   0.3  0.4
0.1 0.2   1   2
0.3 0.4   3   4*/

```

4位以内的向量还可以用下列方式进行定义并初始化：

```cpp
Vector2d a(5.0, 6.0);//2-by-1，内容为5.0和6.0；
Vector3d b(5.0, 6.0, 7.0);
Vector4d c(5.0, 6.0, 7.0, 8.0);

```

### 访问矩阵元素

矩阵元素访问用(i,j)的形式，下标从0开始。

```cpp
#include <iostream>
#include <Eigen/Dense>
 
using namespace Eigen;
 
int main()
{
  MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << "Here is the matrix m:\n" << m << std::endl;
  VectorXd v(2);
  v(0) = 4;
  v(1) = v(0) - 1;
  std::cout << "Here is the vector v:\n" << v << std::endl;
}

```

### 重置矩阵大小

- resize()

可以通过rows(),cols()和size()设置矩阵的大小，同时也可以用resize()来重置矩阵大小，而且resize的大小发生改变的话会删除原来的值，相当于重新定义了一个矩阵。如果resize()的大小没有改变，则元素依旧保存。

```cpp
#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

int main()
{
    MatrixXd m(2,5);
    m<<1,2,3,4,5,6,7,8,9,10;
    cout<<m<<endl;
    m.resize(2,5);//因为size大小不变，元素保留,换成(5,2)也会保留元素
    cout<<m<<endl;
    m.resize(4,3);//重置大小，且内容为0
    cout<<m.rows()<<endl//4行
       <<m.cols()<<endl//3列
      <<m.size()<<endl//大小12
     <<m<<endl;//清空原来的元素，m为0
}

```

```cpp
1  2  3  4  5
 6  7  8  9 10
 1  2  3  4  5
 6  7  8  9 10
4
3
12
0 0 0
0 0 0
0 0 0
0 0 0
```

- "="重置大小

对于动态矩阵可以通过赋值操作改变大小，但是固定矩阵则会报错。

```cpp
#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

int main()
{
    MatrixXd a(3,3);
    MatrixXd b(2,2);
    b<<1,2,3,4;
    a = b;//a的大小也是(2,2)
    cout<<a<<endl;
}
```

Eigen推荐：当矩阵小于16的时候使用固定矩阵。使用静态矩阵对`性能有极大的好处`。`动态矩阵是在堆上操作的`。比如下列操作：

```cpp
MatrixXf mymatrix(rows,columns); 
```

类似于定义了这样一个数组：

```cpp
float *mymatrix = new float[rows*columns];
```

## 矩阵的运算

矩阵重载了C++中的运算符，例如"+", “-”, “+=”。

### 加法和减法

- 二元操作符+/-表示两矩阵相加：a+b
- 一元操作符-表示对矩阵取负：-a
- 组合操作法+=或者-=：a+=b

```cpp
#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;

int main()
{
  Matrix2d a;
  a << 1, 2,
       3, 4;
  MatrixXd b(2,2);
  b << 2, 3,
       1, 4;
  std::cout << "a + b =\n" << a + b << std::endl;
  std::cout << "a - b =\n" << a - b << std::endl;
  std::cout << "Doing a += b;" << std::endl;
  a += b;
  std::cout << "Now a =\n" << a << std::endl;
  Vector3d v(1,2,3);
  Vector3d w(1,0,0);
  std::cout << "-v + w - v =\n" << -v + w - v << std::endl;
}

```

```cpp
a + b =
3 5
4 8
a - b =
-1 -1
 2  0
Doing a += b;
Now a =
3 5
4 8
-v + w - v =
-1
-4
-6
```

### 标量的乘除法

标量乘除法表示**矩阵中的每个元素**都对标量进行相应的运算。

```cpp
Matrix2d a;
  a << 1, 2,
       3, 4;
  Vector3d v(1,2,3);
  std::cout << "a * 2.5 =\n" << a * 2.5 << std::endl;
  std::cout << "0.1 * v =\n" << 0.1 * v << std::endl;
  std::cout << "Doing v *= 2;" << std::endl;
  v *= 2;
  std::cout << "Now v =\n" << v << std::endl;

a * 2.5 =
2.5   5
7.5  10
0.1 * v =
0.1
0.2
0.3
Doing v *= 2;
Now v =
2
4
6

```

### 转置矩阵、共轭矩阵、伴随矩阵

在数学中转置矩阵、共轭矩阵、伴随矩阵分别表示为$a^T$，$\overline{a}$，$a^{*}$ 。在Eigen中他们分别用transpose(),conjugate(),adjoint()表示。转置矩阵很好理解，我们再复习一下共轭矩阵和伴随矩阵。

1. **共轭矩阵**（conjugate matrix）

2. **伴随矩阵**（Adjoint matrix）

由代数余子式组成的矩阵。如下：

![](/home/suyu/Typora/Math/image/Eigen/20201224215113308.png)

代数余子式$A_{ij}=(-1)^{i+j}M_{ij}$，也就是划去i行j列之后，剩下元素组成的n-1阶行列式的值。

```cpp
#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

int main()
{
  //MatrixXcf a = MatrixXcf::Random(2,2);//随机产生2-by-2的复数矩阵
  Matrix3d a;
  a<< 1,2,3,4,5,6,7,8,9;
  cout<<"矩阵a："<<endl<<a<<endl;
  cout<<"转置矩阵："<<endl<<a.transpose()<<endl;
  cout<<"共轭矩阵："<<endl<<a.conjugate()<<endl;
  cout<<"伴随矩阵："<<endl<<a.adjoint()<<endl;
  Matrix3d b;
  b = a.transpose();
  cout<<"矩阵a："<<endl<<a<<endl;
  cout<<"矩阵b："<<endl<<a<<endl;
}
```

注意这里不能用自己的转置(或者其他操作)直接赋值给自己，例如下面的情况会报错：

```cpp
a = a.transpose();//不允许
```

如果需要改变自己的状态，可以用transposeInPlace() ，adjointInPlace()来代替。

```cpp
a.transposeInPlace();//直接进行转置，但是也不允许用“=”给自己赋值
```

### 矩阵的乘法

矩阵的相乘，矩阵与向量的相乘也是使用操作符*，共有 * 和*=两种操作符，其用法可以参考如下代码:

```cpp
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
int main()
{
  Matrix2d mat;
  mat << 1, 2,
         3, 4;
  Vector2d u(-1,1), v(2,0);
  std::cout << "Here is mat*mat:\n" << mat*mat << std::endl;
  std::cout << "Here is mat*u:\n" << mat*u << std::endl;
  std::cout << "Here is u^T*mat:\n" << u.transpose()*mat << std::endl;
  std::cout << "Here is u^T*v:\n" << u.transpose()*v << std::endl;
  std::cout << "Here is u*v^T:\n" << u*v.transpose() << std::endl;
  std::cout << "Let's multiply mat by itself" << std::endl;
  mat = mat*mat;//这个操作是允许的
  std::cout << "Now mat is mat:\n" << mat << std::endl;
}

```

```cpp
Here is mat*mat:
 7 10
15 22
Here is mat*u:
1
1
Here is u^T*mat:
2 2
Here is u^T*v:
-2
Here is u*v^T:
-2 -0
 2  0
Let's multiply mat by itself
Now mat is mat:
 7 10
15 22

```

### 矩阵的点乘和叉乘

**点乘**：相同位元素相乘，并进行相加操作得到一个标量。相当于$a·b=（a^T）*b$。同时也可以用a.adjoint()*b来表示。

**叉乘**：向量积，数学中又称外积、叉积。而且叉乘是有顺序的。

点乘与叉乘使用dot()和cross()操作完成。

```cpp
#include <iostream>
#include <Eigen/Dense>
 
using namespace Eigen;
using namespace std;
int main()
{
  Vector3d v(1,2,3);
  Vector3d w(0,1,2);
 
  cout << "Dot product: " << v.dot(w) << endl;
  double dp = v.adjoint()*w; // automatic conversion of the inner product to a scalar
  cout << "Dot product via a matrix product: " << dp << endl;
  cout << "Cross product:\n" << v.cross(w) << endl;
}

```

**注意**：Eigen中，`叉乘仅对3维列向量才可以使用`，点乘对任意大小向量均可使用。

### 基本算数运算

```cpp
#include <iostream>
#include <Eigen/Dense>

using namespace std;
int main()
{
  Eigen::Matrix2d mat;
  mat << 1, 2,
         3, 4;
  cout << "Here is mat.sum():       " << mat.sum()       << endl;//求和
  cout << "Here is mat.prod():      " << mat.prod()      << endl;//求积
  cout << "Here is mat.mean():      " << mat.mean()      << endl;//求平均值
  cout << "Here is mat.minCoeff():  " << mat.minCoeff()  << endl;//求最小值
  cout << "Here is mat.maxCoeff():  " << mat.maxCoeff()  << endl;//求最大值
  cout << "Here is mat.trace():     " << mat.trace()     << endl;//迹，也就是对角线的和
}
Here is mat.sum():       10
Here is mat.prod():      24
Here is mat.mean():      2.5
Here is mat.minCoeff():  1
Here is mat.maxCoeff():  4
Here is mat.trace():     5

```

## 矩阵的块操作

块操作是以矩形的形式对矩阵或者数组进行操作。块操作可以作为左值也可以作为右值。

### 块操作的使用

块操作对静态、动态矩阵或者数组都可以使用。

块操作有两种使用方式，其中块的起始位置为（i,j）块大小为 (p,q)。也就是说从i开始取p个元素，从j开始取q个元素。

```cpp
matrix.block(i,j,p,q);
matrix.block<p,q>(i,j);
```

- **右值举例：**

```cpp
#include <Eigen/Dense>
#include <iostream>

using namespace std;

int main()
{
  Eigen::MatrixXf m(4,4);
  m <<  1, 2, 3, 4,
        5, 6, 7, 8,
        9,10,11,12,
       13,14,15,16;
  cout << "Block in the middle" << endl;
  cout << m.block<2,2>(1,1) << endl << endl;
  for (int i = 1; i <= 3; ++i)
  {
    cout << "Block of size " << i << "x" << i << endl;
    cout << m.block(0,0,i,i) << endl << endl;
  }
}

Block in the middle
 6  7
10 11

Block of size 1x1
1

Block of size 2x2
1 2
5 6

Block of size 3x3
 1  2  3
 5  6  7
 9 10 11

```

- **左值举例**

```cpp
#include <Eigen/Dense>
#include <iostream>
using namespace std;
using namespace Eigen;
int main()
{
  Array22f m;
  m << 1,2,
       3,4;
  Array44f a = Array44f::Constant(0.6);//4×4数组的每一项都赋0.6
  cout << "Here is the array a:" << endl << a << endl << endl;
  a.block<2,2>(1,1) = m;//把m赋值给a指定的块
  cout << "Here is now a with m copied into its central 2x2 block:" << endl << a << endl << endl;
  a.block(0,0,2,3) = a.block(2,1,2,3);//a指定的块赋给相应的块
  cout << "Here is now a with bottom-right 2x3 block copied into top-left 2x3 block:" << endl << a << endl << endl;
}

Here is the array a:
0.6 0.6 0.6 0.6
0.6 0.6 0.6 0.6
0.6 0.6 0.6 0.6
0.6 0.6 0.6 0.6

Here is now a with m copied into its central 2x2 block:
0.6 0.6 0.6 0.6
0.6   1   2 0.6
0.6   3   4 0.6
0.6 0.6 0.6 0.6

Here is now a with bottom-right 2x3 block copied into top-left 2x3 block:
  3   4 0.6 0.6
0.6 0.6 0.6 0.6
0.6   3   4 0.6
0.6 0.6 0.6 0.6

```

### 块的行操作和列操作

- 行操作：第i行，matrix.row(i)
- 列操作：第j列，matrix.col(j)

行、列操作均是从0开始的

```cpp
#include <Eigen/Dense>
#include <iostream>
using namespace std;

int main()
{
  Eigen::MatrixXf m(3,3);
  m << 1,2,3,
       4,5,6,
       7,8,9;
  cout << "Here is the matrix m:" << endl << m << endl;
  cout << "2nd Row: " << m.row(1) << endl;
  m.col(2) += 3 * m.col(0);//第0列的3倍，加到第2列上
  cout << "After adding 3 times the first column into the third column, the matrix m is:\n";
  cout << m << endl;
}

```

