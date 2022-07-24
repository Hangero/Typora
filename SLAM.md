# SLAM

## 基本名词

### 内参和外参

- 内参

即内参矩阵，是像在像素坐标系下齐次坐标与光心坐标系下齐次坐标的转换关系。
$$
Z\begin{pmatrix}u\\v\\1\\ \end{pmatrix} = \begin{pmatrix}f_x&0&c_x\\0&f_y&c_y\\0&0&1\\ \end{pmatrix}\begin{pmatrix}X\\Y\\Z\\ \end{pmatrix} =KP
$$
其中
$$
K= \begin{pmatrix}f_x&0&c_x\\0&f_y&c_y\\0&0&1\\ \end{pmatrix}
$$
即内参矩阵。

- 外参

即外参矩阵，是相机在相机坐标系下齐次坐标与世界坐标系下齐次坐标的转换关系，是一个包含了旋转矩阵和平移矩阵的变换矩阵。
$$
RP_w+t=TP_w
$$
值的注意的是，等式左侧是非齐次坐标，右侧是齐次坐标。

而$T$为
$$
T=\begin{pmatrix}R&t\\0^T&1\\ \end{pmatrix}
$$
我们设$P$在像素坐标下坐标为$P_{uv}$，世界坐标系下坐标为$P_w$，有转换关系：
$$
ZP_{uv}=Z\begin{pmatrix}u\\v\\1\\ \end{pmatrix}=K(RP_w+t)=KTP_w
$$
