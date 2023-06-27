# Kalman Filters

> 卡尔曼最重要的发明是卡尔曼滤波算法，该算法成就了过去50年间的许多基本技术，如把阿波罗号宇航员送上月球的航天计算机系统、把人类送去探索深海和星球的机器人载体，以及几乎所有需要从噪声数据去估算实际状态的项目。有人甚至把包括环绕地球的卫星系统、卫星地面站及各类计算机系统在内的整个GPS系统合称为一个巨大无比的卡尔曼滤波器                                                                                            ——Frank Moss

### 参考文献

建议按照顺序阅读

【1】[【易懂教程】我是如何十分钟理解与推导贝叶斯滤波(Bayes Filter)算法？](https://blog.csdn.net/varyshare/article/details/97642209)

【2】[如何通俗并尽可能详细地解释卡尔曼滤波？](https://www.zhihu.com/question/23971601/answer/770830003)

【3】[卡尔曼滤波器开发实践之一: 五大公式详解](https://blog.csdn.net/okgwf/article/details/119940610)

【4】[详解卡尔曼滤波器](https://blog.csdn.net/u012411498/article/details/82887417?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522166799040216800186514626%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=166799040216800186514626&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-82887417-null-null.142^v63^control,201^v3^add_ask,213^v2^t3_esquery_v1&utm_term=%E5%8D%A1%E5%B0%94%E6%9B%BC%E6%BB%A4%E6%B3%A2%E5%99%A8&spm=1018.2226.3001.4187)

## 卡尔曼滤波的作用

- 卡尔曼滤波用于有优化我们感兴趣的量，当这些量无法直接测量但可以间接测量时。

- 用于估算系统状态，通过组合各种受噪音的传感器测量值

![image-20221109185339905](/home/suyu/.config/Typora/typora-user-images/image-20221109185339905.png)

## 状态观测器

$$
\hat{X}表示观测值
$$

 ![image-20221109191220782](/home/suyu/.config/Typora/typora-user-images/image-20221109191220782.png)





![image-20221109191401664](/home/suyu/.config/Typora/typora-user-images/image-20221109191401664.png)

![image-20221109191426324](/home/suyu/.config/Typora/typora-user-images/image-20221109191426324.png)

怎么样选择控制增益器，使测量和估算的外部温度之间的误差最小化？





![image-20221109191816412](/home/suyu/.config/Typora/typora-user-images/image-20221109191816412.png)





![image-20221109191938452](/home/suyu/.config/Typora/typora-user-images/image-20221109191938452.png)

如果小于零，错误会趋向0，最后观测值收敛于真实值



## 最优状态估计

平均值和方差

![image-20221109192856403](/home/suyu/.config/Typora/typora-user-images/image-20221109192856403.png)



![image-20221109193445435](/home/suyu/.config/Typora/typora-user-images/image-20221109193445435.png)

![image-20221109193535683](/home/suyu/.config/Typora/typora-user-images/image-20221109193535683.png)



## 最优状态估算算法和方程

![image-20221109193736265](/home/suyu/.config/Typora/typora-user-images/image-20221109193736265.png)

![image-20221109193803755](/home/suyu/.config/Typora/typora-user-images/image-20221109193803755.png)



![image-20221109193837800](/home/suyu/.config/Typora/typora-user-images/image-20221109193837800.png)

![image-20221109193903833](/home/suyu/.config/Typora/typora-user-images/image-20221109193903833.png)

![image-20221109193926720](/home/suyu/.config/Typora/typora-user-images/image-20221109193926720.png)

![image-20221109193940848](/home/suyu/.config/Typora/typora-user-images/image-20221109193940848.png)

![image-20221109194008298](/home/suyu/.config/Typora/typora-user-images/image-20221109194008298.png)

卡尔曼滤波有两部分

- 预测部分 系统莫新用于计算状态预估值和协方差P

![image-20221109194200603](/home/suyu/.config/Typora/typora-user-images/image-20221109194200603.png)

![image-20221109194212883](/home/suyu/.config/Typora/typora-user-images/image-20221109194212883.png)

P状态预估值的方差，并把他当作预测状态中的不确定性的，这种不确定性来自过程噪声，和预估值的不确定性的影响  

在算法最开始，预估值$x(k-1)$和$P(k-1)$来自初始估计值

![image-20221109194837622](/home/suyu/.config/Typora/typora-user-images/image-20221109194837622.png)

- 算法第二步，利用预测步骤中的计算得到的预估值，更新后的状态值及其误差协方差调整卡尔曼增益，时更新后的状态值误差协方差最小

![image-20221109194938032](/home/suyu/.config/Typora/typora-user-images/image-20221109194938032.png)

![image-20221109195022863](/home/suyu/.config/Typora/typora-user-images/image-20221109195022863.png)

![image-20221109195306361](/home/suyu/.config/Typora/typora-user-images/image-20221109195306361.png)

 R测量噪声的协方差

通过调整修正项，卡尔曼增益确定测量值和预估值对计算$\hat{x}(k)$的影响程度,如果测量噪音很小，则测量值更可靠，应对$\hat{x}(k)$的计算贡献更大；相反情况下，预估值的误差很小，则预估值更可信，所以$\hat{x}(k)$的计算更多取决于预估值

极端情况，R趋近0，及测量没有误差

![image-20221109200511932](/home/suyu/.config/Typora/typora-user-images/image-20221109200511932.png)

![image-20221109200705158](/home/suyu/.config/Typora/typora-user-images/image-20221109200705158.png)

如果预估误差的协方差趋近0，$x(k)$的计算完全来自预估值

![image-20221109200824273](/home/suyu/.config/Typora/typora-user-images/image-20221109200824273.png)

![image-20221109201109416](/home/suyu/.config/Typora/typora-user-images/image-20221109201109416.png)

修正值用于预测新的预估值

![image-20221109201408173](/home/suyu/.config/Typora/typora-user-images/image-20221109201408173.png)

要估计当前状态，算法只需要预估状态值和前一时间步的误差协方差矩阵，以及当前测量值

![image-20221109201733403](/home/suyu/.config/Typora/typora-user-images/image-20221109201733403.png)





## 先验与后验

在卡尔曼滤波中，先验（prior）和后验（posterior）是指根据系统的动态模型和观测数据得出的状态估计的两个阶段。

1. 先验（Prior）：先验是指在观测到新数据之前，基于系统的动态模型和上一时刻的状态估计，对当前状态的预测。先验是通过使用状态转移方程（系统的动态模型）来推断当前状态，并考虑到控制输入（如果有的话）。在卡尔曼滤波中，先验状态估计通常由以下方程给出：
   $$
   \hat{x}_{-} = F\hat{x}_{t-1} + Bu
   $$
   其中，x̂<sub>-</sub> 是先验状态估计，F 是状态转移矩阵，x̂<sub>t-1</sub> 是上一时刻的状态估计，B 是控制输入的影响矩阵，u 是控制输入。

2. 后验（Posterior）：后验是指在观测到新数据后，根据先验和观测数据进行更新得到的更准确的状态估计。后验状态估计通过融合先验估计和观测数据，得出对当前状态的修正。在卡尔曼滤波中，后验状态估计通常由以下方程给出：
   $$
   \hat{x}_{t} = \hat{x}_{-} + K(z_{t} - H\hat{x}_{-})
   $$
   其中，x̂<sub>t</sub> 是后验状态估计，x̂<sub>-</sub> 是先验状态估计，K 是卡尔曼增益，z<sub>t</sub> 是观测数据，H 是观测矩阵。卡尔曼增益根据先验估计的不确定性和观测数据的噪声协方差来调整状态修正的权重。

通过先验和后验的迭代更新，卡尔曼滤波能够递归地估计系统的状态，并提供最优的状态估计结果。
