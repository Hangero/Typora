# SOLVEPnP

## 算法理解

### 基础

#### 最小二乘法

**最小二乘准则**，是进行最小二乘平差计算的一个基本原则.它是求解不定线性方程组的一个附加条件。在任何平差计算中，所列出的方程式的个数，总是少于方程中所包含的未知量的个数，因此其解不惟一在最小二乘准则下求解，可以得到一组惟一解.若在平差中，只有观测值为随机量时，最小二乘准则为：

​                                                                      $$V^TPV=min$$

**线性回归**，是利用称为线性回归方程的最小平方函数对一个或多个自变量和因变量之间关系进行建模的一种回归分析。其表达形式为$y = w'x+e$，e为误差服从均值为0的正态分布。

一般地，影响y的因素往往不止一个，假设有$x_1，x_2，...，x_k$，k个因素，通常可考虑如下的线性关系式：
$$
y= \beta_0+\beta_1x_1+...+\beta_kx_k+\epsilon
$$
**最小二乘法的矩阵解法**

[一文让你彻底搞懂最小二乘法（超详细推导）](https://blog.csdn.net/MoreAction_/article/details/106443383?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522166176256716781683951714%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=166176256716781683951714&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-106443383-null-null.142^v42^new_blog_pos_by_title,185^v2^control&utm_term=%E6%9C%80%E5%B0%8F%E4%BA%8C%E4%B9%98%E6%B3%95&spm=1018.2226.3001.4187)

作为对LaTex的练习，自己手敲了一遍。

对于m个含有n-1个特征的样本，对于每一个特征：
$$
h_1= \beta_0+\beta_1x_{1,1}+...+\beta_kx_{1,n-1}\\
h_2= \beta_0+\beta_1x_{2,1}+...+\beta_kx_{2,n-1}\\
\vdots\\
h_m= \beta_0+\beta_1x_{m,1}+...+\beta_kx_{m,n-1}\\
$$
为便于矩阵表示，设$1=x_{i,0},i\in1,2,\cdots,m$，可得：
$$
h_1= \beta_0x_{1,0}+\beta_1x_{1,1}+...+\beta_kx_{1,n-1}\\
h_2= \beta_0x_{2,0}+\beta_1x_{2,1}+...+\beta_kx_{2,n-1}\\
\vdots\\
h_m= \beta_0x_{m,0}+\beta_1x_{m,1}+...+\beta_kx_{m,n-1}\\
$$
写成矩阵的形式：
$$
\begin{pmatrix}h_1\\h_2\\ \vdots \\h_m \end{pmatrix}=\begin{pmatrix}x_{1,0}&x_{1,1}&\cdots&x_{1,n-1}\\x_{2,0}&x_{2,1}&\cdots&x_{2,n-1}\\ \vdots &\vdots &\vdots&\vdots \\x_{m,0}&x_{m,1}&\cdots&x_{m,n-1}\end{pmatrix}\begin{pmatrix}\beta_1\\\beta_2\\ \vdots \\\beta_m \end{pmatrix}
$$
即
$$
\bf h =\bf X \beta
$$
其中，$\bf h$为mx1的向量, 代表模型的理论值，$\beta$为nx1的向量，$\bf X$为mxn维的矩阵，m代表样本的个数,n代表样本的特征数，于是目标损失函数用矩阵表示为：

$$
J(\beta)=||\bf h -\bf Y||^2 =||\bf X \beta -\bf Y||^2=(\bf X\beta-\bf Y)^T(\bf X\beta-\bf Y)
$$
其中$\bf Y$是样本的输出向量, 维度为mx1。

**矩阵求导公式**

[矩阵求导](https://zhuanlan.zhihu.com/p/273729929)
$$
\frac{\partial X^TA}{\partial X}=\frac{\partial A^TX}{\partial X}=A\\
\frac{\partial X^TAX}{\partial X}=AX+A^TX
$$
对目标函数化简：
$$
J(\beta)&=(\bf X\beta-\bf Y)^T(\bf X\beta-\bf Y)\\
 &=(\beta^T \bf X^T - Y^T)(\bf X\beta-\bf Y)\\
 &=\beta^T\bf X^T\bf X\beta -\beta^T\bf X^T\bf Y -\bf Y^T\bf X\beta+\bf Y^T\bf Y
$$
求导令其等于0：
$$
\frac{\partial J(\beta)}{\partial \beta}=2\bf X^T\bf X\beta -2\bf X^T\bf Y=0
$$
解得：
$$
\beta =(\bf X^TX)^{-1}\bf X^TY
$$

#### 岭回归

当为奇异矩阵时（不可逆），此时最小二乘法的解析解无法进行计算。导致$X^TX$不可逆的可能原因是$X$中的列向量线性相关，即数据中存在特征冗余，某些特征可以根据其它特征的线性组合来得到。
另外，当$X^TX$为 **病态矩阵(ill-conditioned matrix)** 时，最小二乘法也会失效，此时虽然也能按照解析解进行计算，但由于解的方差过大，不稳定，结果精度很差。

> 求解方程组时如果对数据进行较小的扰动，则得出的结果具有很大波动，这样的矩阵称为病态矩阵。

**岭回归**是一种改良版的最小二乘法，它放弃了最小二乘法的无偏性，但更加稳定可靠，在实际应用中也更加常用。

岭回归解为：
$$
\beta =(X^TX+\lambda E)^{-1}X^TY
$$
其中λ 是一个超参数，称为岭系数，$E$是单位矩阵。此时，$(X^TX+\lambda E)$一定可逆（满秩）。


#### 最速下降法

主要优化模型：
$$
\underset{x\in R^2}{min}f(x)
$$
最速下降法由于只考虑当前下降最快而不是全局下降最快，在求解非线性无约束问题时，最重要的是得到每一步迭代的方向$d^{(k)}$和每一步下降的长度$\lambda^{(k)}$。考虑函数$f(x)$在点$x^{(k)}$沿着方向d的方向导数，当$f$连续可微时，方向导数为负，说明函数值沿着该方向下降；方向导数越小（负值)，表明下降的越快，因此确定搜索方向$d^{(k)}$的一个想法就是以$f(x)$在点$x^{(k)}$方向导数最小的方向作为搜索方向。

**搜索方向$d^{(k)}$的确定**

设方向d为单位向量，$||d||=1$，从点$x^{(k)}$按方向d，步长$\lambda$进行搜索得到下一个点$x^{(k+1)}=x^{(k)}+\lambda_kd^{(k)}$

(计算过程略，太难打了！！)

可以确定最速下降方向为
$$
-\bigtriangledown f(x^{(k)})
$$
**步长$\lambda^{(k)}$的确定**





### 非线性最小二乘问题

基本思想：把非线性最小二乘问转化成一系列线性最小二乘问题，通过迭代求解。

$F(x)=\sum_{i=1}^{m}f_i(x)^2$，其中$x=(x_1,x_2,...,x_n)^T$,把极小化$F(x)$
$$
minF(x)=\sum_{i=1}^{m}f_i(x)^2
$$
称为最小二乘问题，当$f_i(x)$不全为线性函数时，为非线性最小二乘问题。

**第一步 线性化**

把$f_i(x)$在$x^{(k)}$处泰勒展开，省略高阶项后函数记作$\phi_i(x)$，此时$\phi_i(x)$是$f_i(x)$在$x=x^{(k)}$处的线性函数，可近似代表$f_i(x)$

**第二步 求导**

令$\Psi(x)=\sum_{i=1}^{m}\phi_i(x)^2$

#### 高斯牛顿法

把$f_i(x)$在$x^{(k)}$处一阶泰勒展开，用$\phi_i(x)$近似可得
$$
\phi_i(x)&=f_i(x^{(k)})+\bigtriangledown f_i(x^{(k)})^T(x-x^{(k)})\\
&=\bigtriangledown f_i(x^{(k)})^Tx-[\bigtriangledown f_i(x^{(k)})^Tx^{(k)}-f_i(x^{(k)})]
$$
(到这算不懂了，$\bigtriangledown f_i(x^{(k)})^T$看不懂了，再补一下数学知识再回来)

最后得
$$
x-x^{(k)}=-(A_k^TA_k)^{-1}A_k^Tf^{(k)}
$$


####  列文伯格-马夸尔特算法

对高斯牛顿法进行改进
$$
x-x^{(k)}=-(A_k^TA_k+\lambda E)^{-1}A_k^Tf^{(k)}
$$
其中$E$是单位矩阵，$\lambda$是正实数。

- 当$\lambda=0$时，LM算法退化为高斯牛顿算法
- 当$\lambda$很大时，退化成最速下降算法

$\lambda$的值会根据优化进行的状态随时调整，控制着x前进的方向和前进的步长。

- 前期类似最速下降法，使x快速收敛到极值点附近
- 后期类似高斯牛顿法，减缓x前进步伐使x稳定收敛

### PnP

[PnP](https://blog.csdn.net/qq_29462849/article/details/120838218?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522166177367916782350850916%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=166177367916782350850916&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-120838218-null-null.142^v42^new_blog_pos_by_title,185^v2^control&utm_term=pnp&spm=1018.2226.3001.4187)

### APnP

[APnP](https://blog.csdn.net/App_12062011/article/details/82144348?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522166177369416782390530157%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=166177369416782390530157&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-82144348-null-null.142^v42^new_blog_pos_by_title,185^v2^control&utm_term=ap3p&spm=1018.2226.3001.4187)

(是在是算不下去，我先补一点矩阵分析的知识，回来再算)

## 源码

flages ：调用的处理方法

我在slovepnp.cpp这个文件中只找到了SLOVEPNP，没有找到SOLVEPNP_ITERATIVE和SOLVEPNP_APNP

![](/home/suyu/Typora/View/image/PnP/2022-08-29 21-39-02 的屏幕截图.png)

不过，能看出APnP貌似适用了PnP。

SOLVEPNP_ITERATIVE使用的是列文伯格-马夸尔特算法，不过我没有找到具体的文章介绍怎么适用。

SOLVEPNP_APNP适用的是APnP的方法，比经典P3P算法的更快，更鲁棒，更准确。

