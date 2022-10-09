# Qt

## 1.  Qt 概述

### 1.1  什么是 Qt

>不论我们学习什么样的知识点首先第一步都需要搞明白它是什么，这样才能明确当前学习的方向是否正确，下面给大家介绍一下什么是 Qt。

1. 是一个跨平台的 C++ 应用程序开发框架

- 具有短平快的优秀特质：投资少、周期短、见效快、效益高
- 几乎支持所有的平台，可用于桌面程序开发以及嵌入式开发
- 有属于自己的事件处理机制
- 可以搞效率的开发基于窗口的应用程序。

2. Qt 是标准 C++ 的扩展，C++ 的语法在 Qt 中都是支持的

- 良好封装机制使得 Qt 的模块化程度非常高，可重用性较好，可以快速上手。
- Qt 提供了一种称为 signals/slots 的安全类型来替代 callback（回调函数），这使得各个元件 之间的协同工作变得十分简单。

### 1.2  Qt 的特点

1. 广泛用于开发 GUI 程序，也可用于开发非 GUI 程序。

- GUI = Graphical User Interface
- 也就是基于窗口的应用程序开发

2. 有丰富的 API

- Qt 包括多达 250 个以上的 C++ 类
- 可以处理正则表达式

3. 支持 2D/3D 图形渲染，支持 OpenGL
4. Qt 给程序猿提供了非常详细的官方文档
5. 支持 XML，Json
6. 框架底层模块化， 使用者可以根据需求选择相应的模块来使用
7. 可以轻松跨平台

- 和 Java 的跨平台方式不同
- 在不同的平台使用的是相同的上层接口，但是在底层封装了不同平台对应的 API（暗度陈仓）。

### 1.3 Qt 中的模块

>Qt 类库里大量的类根据功能分为各种模块，这些模块又分为以下几大类：

- Qt 基本模块（Qt Essentials)：**提供了 Qt 在所有平台上的基本功能。**
- Qt 附加模块（Qt Add-Ons)：实现一些特定功能的提供附加价值的模块。
- 增值模块（Value-AddModules)：单独发布的提供额外价值的模块或工具。
- 技术预览模块（Technology Preview Modules）：一些处于开发阶段，但是可以作为技术预览使用的模块。
- Qt 工具（Qt Tools)：帮助应用程序开发的一些工具。

<All Modules>

| 模块                                         | 描述                                               |
| -------------------------------------------- | -------------------------------------------------- |
| <font color = SandyBrown>`Qt Core`</font>    | Qt 类库的核心，所有其他模块都依赖于此模块          |
| <font color = SandyBrown>`Qt GUI`</font>     | 设计 GUI 界面的基础类，包括 OpenGL                 |
| Qt Multimedia                                | 音频、视频、摄像头和广播功能的类                   |
| Qt Multimedia Widgets                        | 实现多媒体功能的界面组件类                         |
| Qt Network                                   | 使网络编程更简单和轻便的类                         |
| Qt QML                                       | 用于 QML 和 JavaScript 语言的类                    |
| Qt Quick                                     | 用于构建具有定制用户界面的动态应用程序的声明框架   |
| Qt Quick Controls                            | 创建桌面样式用户界面，基于 Qt Quick 的用户界面控件 |
| Qt Quick Dialogs                             | 用于 Qt Quick 的系统对话框类型                     |
| Qt Quick Layouts                             | 用于 Qt Quick 2 界面元素的布局项                   |
| Qt SQL                                       | 使用 SQL 用于数据库操作的类                        |
| Qt Test                                      | 用于应用程序和库进行单元测试的类                   |
| <font color = SandyBrown>`Qt Widgets`</font> | 用于构建 GUI 界面的 C++ 图形组件类                 |

## 2.  QtCreator

1. QtCreator 是编写 Qt 程序默认使用的一款 IDE，使用 VS 写 Qt 程序也是可以的，在此不做介绍。
2. 使用 QtCreator 创建的项目目录中不能包含中文
3. QtCreator 默认使用 Utf8 格式编码对文件字符进行编码

## 3.  实例

```cpp
HelloQt
|----HelloQt.pro //项目文件，一般不修改
|--Headers
|--Sources
  |--main.cpp//入口函数
  |--mianwindow.cpp
|--Forms
```

```cpp
#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    //应用程序类，一个qt应用程序中有且只有一个
    QApplication a(argc, argv);//QApplication类名，对应同名头文件
    //窗口对象，一个mainwindow对应三个文件.h  .cpp  .ui
    MainWindow w;
    //显示窗口
    w.show();
    //阻塞函数，程序不会退出，进入事件循环
    return a.exec();
}
```

```cpp
//mainwindow.h 

ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
    
    //对应的是ui文件中的类，和C++中的类不同
namespace Ui { class MainWindow; }//在Ui这个命名空间中，有一个类 MainWindow
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H

```

```cpp
//mainwindow.cpp

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)//把指针实例化，即我们要看到的窗口要实例化
{
    ui->setupUi(this);//将两个类捆绑起来
}

MainWindow::~MainWindow()
{
    delete ui;
}
```

