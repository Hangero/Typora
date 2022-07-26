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

