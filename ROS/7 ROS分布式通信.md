# ROS分布式通信

ROS是一种分布式软件框架，节点之间通过松耦合的方式进行组合

## 实现分布式多机通信

### 设置IP地址，确保底层链路的联通

![image-20221215163912495](/home/suyu/.config/Typora/typora-user-images/image-20221215163912495.png)

```shell
ifconfig #查看ip地址，indet addr

sudo vi /etc/hosts#把ip地址设置进入对方的hosts文件
```

![image-20221215164313204](/home/suyu/.config/Typora/typora-user-images/image-20221215164313204.png)

```shell
ping hcx-pc#ping + 计算机名，查看通信
```

### 在从机端设置`ROS-MASTER_URI`，让从机找到ROS Master

```shell
export ROS_MASTER_URI=http://hcx-pc:11311#当前终端有效；11311是ROSMaster的默认端口号

echo "ROS_MASTER_URI=http://hcx-pc:11311">>~/.bashrc#所有终端有效
```

