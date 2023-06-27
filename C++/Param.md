# Param

## 静态变量和静态函数

重构了之前的代码，功能未完全恢复；现在还有以下修改未完成：1. 把所有全局变量修改成静态变量，并从param.yaml中调用并初始化；2. main函数只保留加载参数和线程启动的入口；3. 部分函数，类，变量不符合命名规范，需要修改。目前规划是：有电脑可以调车时先复现功能；没有电脑调车时重构代码

## 其他

### ::

[C++中的.和::和：和->的区别](https://blog.csdn.net/hxlawf/article/details/99937248?ops_request_misc=&request_id=&biz_id=102&utm_term=::&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-1-99937248.nonecase&spm=1018.2226.3001.4187)

### cv::FileStorage和cv::FileNode

cv::FileStorage的构造函数：

```cpp
cv::FileStorage::FileStorage();
cv::FileStorage::FileStorage(string filename, int flag);//flag的参数可以为cv::FileStorage::WRITE、cv::FileStorage::READ或者cv::FileStorage::APPEND。
cv::FileStorage::open(string file int flag);
```

cv::FileStorage对象代表着一个XML或者YAML格式的数据文件。可以使用默认构造函数创建一个未打开的cv::FileStorage对象，稍后再使用cv::FileStorage::open()函数打开，也可以给定文件名参数创建一个cv::FileStorage对象，flag的参数可以为cv::FileStorage::WRITE、cv::FileStorage::READ或者cv::FileStorage::APPEND。

成功打开文件后，可以使用cv::FileStorage::operator<<()进行写入操作。


**注意，读取结束后，需要调用cv::FileStorage::release()关闭文件。**

```cpp
cv::FileStorage fs("test.yml", cv::FileStorage::WRITE);
	fs << "frameCount" << 5;
fs.release();
```

