# Tmux

## 基本概念

在使用tmux之前我们先了解关于tmux的几个名词：

session，会话（任务）
windows，窗口
pane，窗格
关于session，很多人把session成为会话，但我觉得叫任务更适合一些。

在普通的终端中，窗口和其中由于session（任务）而启动的进程是连在一起的，关闭窗口，session就结束了，session内部的进程也会终止，不管是否运行完。但是在具体使用中，我们希望当前的session隐藏起来，在终端中做其他事情，但是又不希望session及其进程被关闭。这样就需要用到tmux，对session进行解绑。之后再想继续出来这个session的时候，再次绑定就可以回到之前的工作状态。

对于window可以理解为一个工作区，一个窗口。

对于一个session，可以创建好几个window，对于每一个窗口，都可以将其分解为几个pane小窗格。

所以，关于session、window、pane的关系是：

$$
[pane∈window]∈session
$$

## session操作

### 启动

新建session，可以在terminal上输入`tmux`命令，会自动生成一个id为0的session

```shell
tmux
```

也可以在建立时显式地说明session的名字，这个名字可以用于解绑后快速的重新进入该session：

```shell
tmux new -s your-session-name
```

### 分离

在tmux窗口中，按下`ctrl+b d`或者输入以下命令，就会将当前session与窗口分离，session转到后台执行：

```shell
tmux detach
```

### 退出

如果你想退出该session，可以杀死session：

```bash
tmux kill-session -t your-session-name
```

当然，也可以使用`ctrl+d`关闭该session的所有窗口来退出该session。

### 绑定、解绑、切换session

假设现在正处于session1，使用分离操作就是将session1进行解绑:

```bash
tmux detach
```

而如果你想再次绑定session1，可以使用命令：

```bash
tmux attach -t your-session-name
```

切换到指定session：

```bash
tmux switch -t your-session-name
```

### 重命名session

```bash
tmux rename-session -t old-session new-session
```



## window操作

一个session可以有好几个window窗口。

### 新建窗口`tmux new-window`

```bash
# 新建一个指定名称的窗口
tmux new-window -n your-window-name
```

### 切换窗口

- ctrl+b c: 创建一个新窗口（状态栏会显示多个窗口的信息）
- ctrl+b p: 切换到上一个窗口（按照状态栏的顺序）
- ctrl+b n: 切换到下一个窗口
- ctrl+b w: 从列表中选择窗口（这个最好用）

### 重命名窗口

```bash
tmux rename-window -t old_name new_name
```



## pane操作

tmux可以将一个窗口分为几个窗格（pane），每个窗格运行不同的命令。

### 划分窗口

```bash
# 划分为上下两个窗格
tmux split-window

# 划分左右两个窗格
tmux split-window -h

```

其实**划分窗格**pane使用快捷键更方便，如果你当前pane正在运行程序不就没法使用命令了嘛。

- 左右划分`：`ctrl+b %
- 上下划分 ： ctrl+b "

### 光标位置

使用语句太麻烦了，使用快捷键最好：ctrl+b arrow-key（方向键）：光标切换到其他窗格

### 交换窗格位置

```bash
# 当前窗格往上移
tmux swap-pane -U

# 当前窗格往下移
tmux swap-pane -D

```

### 关闭窗格

`ctrl+d`，记住如果只有一个窗格就是关闭window哦

### 窗格操作其他快捷键

- `Ctrl+b %`：划分左右两个窗格。
- `Ctrl+b "`：划分上下两个窗格。
- `Ctrl+b <arrow key>`：光标切换到其他窗格。`<arrow key>`是指向要切换到的窗格的方向键，比如切换到下方窗格，就按方向键`↓`。
- `Ctrl+b ;`：光标切换到上一个窗格。
- `Ctrl+b o`：光标切换到下一个窗格。
- `Ctrl+b {`：当前窗格与上一个窗格交换位置。
- `Ctrl+b }`：当前窗格与下一个窗格交换位置。
- `Ctrl+b Ctrl+o`：所有窗格向前移动一个位置，第一个窗格变成最后一个窗格。
- `Ctrl+b Alt+o`：所有窗格向后移动一个位置，最后一个窗格变成第一个窗格。
- `Ctrl+b x`：关闭当前窗格。
- `Ctrl+b !`：将当前窗格拆分为一个独立窗口。
- `Ctrl+b z`：当前窗格全屏显示，再使用一次会变回原来大小。
- `Ctrl+b Ctrl+<arrow key>`：按箭头方向调整窗格大小。
- `Ctrl+b q`：显示窗格编号。

## 其他操作

```cpp
# 列出所有快捷键，及其对应的 Tmux 命令
$ tmux list-keys

# 列出所有 Tmux 命令及其参数
$ tmux list-commands

# 列出当前所有 Tmux 会话的信息
$ tmux info

# 重新加载当前的 Tmux 配置
$ tmux source-file ~/.tmux.conf

```

### tmux上下翻屏

使用快捷键`ctrl+b [ `，就可以通过方向键上下移动使用`PageUp`和`PageDown`可以实现上下翻页



## tmux基本操作

基本的操作无非就是对会话、窗口、窗格进行管理，包括创建、关闭、重命名、连接、分离、选择等等。

一般使用命令和快捷键进行操作，可在系统shell终端和tmux命令模式（类似vim的命令模式）下使用命令，或者在tmux终端使用快捷键。

tmux默认的快捷键前缀是***\*Ctrl+b\****(下文用***\*prefix\****指代)，按下前缀组合键后松开，再按下命令键进行快捷操作，比如使用***\*prefix d\****分离会话（应该写作**prefix d**而不是**prefix+d，**因为**d**键不需要与**prefix**同时按下）。

快捷键可以自定义，比如将前缀改为**Ctrl+a**，但需要保留shell默认的**Ctrl+a**快捷键，按如下所示修改~/.tmux.conf文件：

```
1 set-option -g prefix C-a
2 unbind-key C-b
3 bind-key C-a send-prefix
4 bind-key R source-file ~/.tmux.conf \; display-message "~/.tmux.conf reloaded."
```

现在已将原先的**Ctrl+a**用**prefix Ctrl+a**取代，即需要按两次**Ctrl+a**生效。

第4行的作用是使用**prefix r**重新加载配置文件，并输出提示，否则需要关闭会话后配置文件才能生效，也可手动加载配置文件，在tmux终端输入"**prefix :"**进入命令模式，用**source-file**命令加载配置文件。

**注意，将多个命令写在一起作为命令序列时，命令之间要用空格和分号分隔。** 

 

### 会话管理　

***\*常用命令\****

***\*tmux new\****　　创建默认名称的会话（在tmux命令模式使用**new**命令可实现同样的功能，其他命令同理，后文不再列出tmux终端命令）

**tmux new -s mysession**　　创建名为mysession的会话

**tmux ls**　　显示会话列表

**tmux a**　　连接上一个会话

**tmux a -t mysession**　　连接指定会话

**tmux rename -t s1 s2**　　重命名会话s1为s2

**tmux kill-session**　　关闭上次打开的会话

**tmux kill-session -t s1**　　关闭会话s1

**tmux kill-session -a -t s1**　　关闭除s1外的所有会话

**tmux kill-server**　　关闭所有会话

**常用快捷键**

**prefix s**　　列出会话，可进行切换

**prefix $**　　重命名会话

**prefix d**　　分离当前会话

**prefix** **D**　　分离指定会话

　　

### 窗口管理

**prefix c**　　创建一个新窗口

***\*prefix ,\****　　重命名当前窗口

**prefix w**　　列出所有窗口，可进行切换

**prefix n**　　进入下一个窗口

**prefix p**　　进入上一个窗口

**prefix l**　　进入之前操作的窗口

**prefix 0~9**　　选择编号0~9对应的窗口

***\*prefix .\****　　修改当前窗口索引编号

***\*prefix '\****　　切换至指定编号（可大于9）的窗口

**prefix f**　　根据显示的内容搜索窗格

**prefix &**　　关闭当前窗口

　

### **窗格管理**

**prefix %**　　水平方向创建窗格

**prefix "**　　垂直方向创建窗格

**prefix Up|Down|Left|Right**　　根据箭头方向切换窗格

**prefix q**　　显示窗格编号

**prefix o**　　顺时针切换窗格

**prefix }**　　与下一个窗格交换位置

**prefix {**　　与上一个窗格交换位置

**prefix x**　　关闭当前窗格

**prefix space(空格键)**　　重新排列当前窗口下的所有窗格

**prefix !**　　将当前窗格置于新窗口

**prefix Ctrl+o**　　逆时针旋转当前窗口的窗格

**prefix t**　　在当前窗格显示时间

**prefix z**　　放大当前窗格(再次按下将还原)

**prefix i**　　显示当前窗格信息

 

　　

### **其他命令**

**tmux list-key**　　列出所有绑定的键，等同于**prefix ?**

**tmux list-command**　　列出所有命令























