# OpenCV

*参考*

*[OpenCV教程(C++)](https://blog.csdn.net/weixin_42715287/article/details/106442830)*

## 前置

Gray Scale Image

8 bits or 256levels

for a colored image we have three grayscale images representing the intensities of red green and blue,in short rgb adding these images together gives us a full color image this means a colored vga image.

RGB VGA   640 X 450 X 3

Mat

kjdhfiuwahfiuajhgsrjuhgeuwijhfurjghrshgjg

## 1.  读取图片/视频/摄像头

### 从文件读取图片

| 模块                                 | 功能                           |
| ------------------------------------ | ------------------------------ |
| imgcodecs                            | Image file reading and writing |
| imgproc                              | Image Procssing                |
| highgui                              | High-level GUI                 |
| <font color=sandybrown>opencv</font> |                                |

```cpp
Mat cv::imread(const String &filename, int flags = IMREAD_COLOR)
```

**从文件加载图像**。函数`imread`从指定文件（路径）加载图像并返回。 如果无法读取图像（由于缺少文件、权限不正确、格式不受支持或无效），该函数将返回一个空矩阵（` Mat::data==NULL` ）。在彩色图像的情况下，解码图像的通道将以 B G R 顺序存储。

- flags>0返回一个3通道彩色图像（解码后的图像以BGR存储）

- flags=0返回灰度图像

- flags<0返回包含Alpha通道的加载图像

```cpp
void cv::imshow(cosnst String &windowsname, InputArray mat)
```

在指定窗口中显示图像。这个函数后面应该是 cv::waitKey 函数，它显示指定毫秒的图像。否则，它不会显示图像。如：

- waitKey(0) 将无限显示窗口，直到有任何按键（适用于图像显示）。 
- waitKey(25) 将显示一帧 25 毫秒，之后显示将自动关闭。（如果你把它放在一个循环中读取视频，它会逐帧显示视频）

```cpp
int cv::waitKey(int delay = 0)
```

等待按下的键。函数 waitKey 无限等待按键事件（当 delay≤0 时）或延迟毫秒，当它为正时。由于操作系统在切换线程之间有最短时间，因此该函数不会完全等待延迟毫秒，它会至少等待延迟毫秒，具体取决于当时您计算机上正在运行的其他内容。如果在指定的时间过去之前没有按下任何键，则返回被按下键的代码或 -1。


图像实际上就是图片，我们需要捕捉所有图片所以用到while

```cpp
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
	string path = "Resources/test.png";
	Mat img = imread(path);
	imshow("Image", img);
	waitKey(0); //增加延时，0表示无穷

	return 0;
}

```

### 从文件读取视频

<font color = powderblue>要捕获视频，需要创建一个VideoCapture对象。</font>font它的参数可以是视频文件的名称或设备索引。
OpenCV3.4.6中VideoCapture类构造函数及成员函数

- cv::VideoCapture::VideoCapture()
- cv::VideoCapture::VideoCapture(const String &filename)
- cv::VideoCapture::VideoCapture(const String &filename, int apiPreference)
- cv::VideoCapture::VideoCapture(int index)
- cv::VideoCapture::VideoCapture(int index, int apiPreference)

打开视频文件或捕获设备或 IP 视频流进行视频捕获。



```cpp
virtual bool cv::VideoCapture::isOpened() const
```

如果视频捕获已经初始化，则返回true。如果先前对 `VideoCapture `构造函数或`VideoCapture::open()`的调用成功，则该方法返回 true。

```cpp
`virtual bool cv::VideoCapture::read(OutputArray image)`
```

抓取、解码并返回下一个视频帧。

```cpp
`virtual double cv::VideoCapture::get(int proId) const`
```

返回指定的**`VideoCapture`**属性

```cpp
`virtual double cv::VideoCapture::set(int proId, double value)`
```

在**`VideoCapture`**中设置一个属性。

```cpp
  if (cv::waitKey(1) == 27)  // 27是键盘摁下esc时，计算机接收到的ascii码值
        {
            break;
        }
```



```cpp
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
	string path = "Resources/test_video.mp4";
	VideoCapture cap(path); //视频捕捉对象
	Mat img;
	while (true) {

		cap.read(img);

		imshow("Image", img);
		waitKey(1);
	}
	return 0;
}
```

### 读摄像头

```cpp
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
	VideoCapture cap(0);//注意摄像头的id地址
	Mat img;

	while (true) {

		cap.read(img);

		imshow("Image", img);
		waitKey(1);
	}

	return 0;
}

```



## 2.  基础函数

### 2.1  颜色空间转换

```cpp
void cv::cvtColor(InputArray src, OutputArray dst, int code, int dstCn = 0)
```

将图像从一种颜色空间转换为另一种颜色空间。该函数将输入图像从一种颜色空间转换为另一种颜色空间。在从 RGB 颜色空间转换的情况下，应明确指定通道的顺序（RGB 或 BGR）。man请注意，OpenCV 中的默认颜色格式通常称为 RGB，但实际上是 BGR（字节反转）。因此，标准（24 位）彩色图像中的第一个字节将是 8 位蓝色分量，第二个字节将是绿色，第三个字节将是红色。 然后第四、第五和第六个字节将是第二个像素（蓝色，然后是绿色，然后是红色），依此类推。


### 2.2  滤波

```cpp
void cv::GaussianBlur(InputArray src, OutputArray dst, Size ksize, double sigmaX, doube sigmaY = 0, int borderType = BORDER_DEFAULT)
```

使用高斯滤波器模糊图像。该函数将源图像与指定的高斯核进行卷积。

### 2.3  

```cpp
void cv::Canny(InputArray image, OutputArray edges, double threshold1, double threshold2, int apertureSize = 3, bool L2gradient = false)
```

使用 Canny 算法在图像中查找边缘

### 2.4  图像操作

```cpp
Mat cv::getStructuringElement(int shape, Size ksize, Point anchor = Point(-1, -1))
```

**返回指定大小和形状的结构元素，用于形态学操作**。该函数构造并返回可以进一步传递给腐蚀、扩张或形态学的结构元素。 但是您也可以自己构建任意二进制掩码并将其用作结构元素。

```cpp
void cv::dilate(InputArray src, OutputArray dst, InuputArray kernel, Point anchor = Point(-1, -1), int iterations = 1, int borderType = BORDER_CONSTANT, const Scalar &borderValue = morphologyDefaultBorderValue())
```

**使用特定的结构元素膨胀图像**。

```cpp
void cv::erode(InputArray src, OutputArray dst, InuputArray kernel, Point anchor = Point(-1, -1), int iterations = 1, int borderType = BORDER_CONSTANT, const Scalar &borderValue = morphologyDefaultBorderValue())
```

**使用特定的结构元素腐蚀图像**。

```cpp
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
	string path = "resources/test.png";
	Mat img = imread(path);
	Mat imgGray, imgBlur, imgCanny, imgDil, imgErode;

    //cvt是convert的缩写，将图像从一种颜色空间转换为另一种颜色空间。
	cvtColor(img, imgGray, COLOR_BGR2GRAY); //灰度化
    //使用高斯滤波器模糊图像。该函数将源图像与指定的高斯核进行卷积,Size(7,7)是核大小,数字越大越模糊
	GaussianBlur(img, imgBlur, Size(3, 3), 3, 0); //高斯模糊
	
    Canny(imgBlur, imgCanny, 25, 75); //边缘检测

    //创建一个核，增加Size（只能是奇数）会扩张/侵蚀更多
	Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
	
    dilate(imgCanny, imgDil, kernel);//扩张边缘（增加边缘厚度）
	erode(imgDil, imgErode, kernel);//侵蚀边缘（减小边缘厚度）

	imshow("Image", img);
	imshow("ImageGray", imgGray);
	imshow("ImageBlur", imgBlur);
	imshow("ImageCanny", imgCanny);
	imshow("ImageDilation", imgDil);
	imshow("ImageErode", imgErode);
	waitKey(0);

	return 0;
}

```

## 3.  调整和裁减

```cpp
void cv::resize(InputArray src, OutputArray dst, Size dsize, double fx=0, double fy=0, int interpolation = INTER_LINEAR)
```

**调整图像的大小**。函数 `resize` 将图像 `src` 的大小缩小到或最大到指定的大小。请注意，不考虑初始 `dst `类型或大小。相反，大小和类型是从 `src、dsize、fx 和 fy` 派生的。

```cpp
typedef struct CvRect   
　　{   
　　int x; /* 方形的左上角的x-坐标 */   
　　int y; /* 方形的左上角的y-坐标*/   
　　int width; /* 宽 */   
　　int height; /* 高 */   
　　}  

```



```cpp
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
	string path = "resources/test.png";
	Mat img = imread(path);
	Mat imgResize, imgCrop;

	cout << img.size() << endl;//打印图像尺寸
    resize(img, imgResize, Size(640, 480));//指定图片尺寸缩放
	resize(img, imgResize, Size(), 0.5, 0.5);//指定缩放比例，不指定图片尺寸

	Rect roi(200, 100, 300, 300);//以左上角为坐标原点，（200，100）为矩形的左上角坐标，300,300为矩形长宽
	imgCrop = img(roi);//裁剪图像，为了找到特定区域 添加更多处理

	imshow("Image", img);
	imshow("ImageResieze", imgResize);
	imshow("ImageCrop", imgCrop);
	waitKey(0);

	return 0;
}

```

## 4.  绘制形状和文字

```cpp
Mat(int rows, int cols, int type, const Scalar &s)
```

重载的构造函数

```cpp
void cv::circle(InputOutputArray img, Point center, int radius, const Scalar &color, int thickness=1, int lineType=LINE_8, int shift=0)
```

函数 `cv::circle` 用给定的中心和半径绘制一个简单的或实心圆。

```cpp
void cv::rectangle(InputOutputArray img, Point pt1, Point pt2, const Scalar &color, int thickness=1, int lineType=LINE_8, int shift=0)
```

绘制一个简单的、粗的或填充的右上矩形。函数 `cv::rectangle` 绘制一个矩形轮廓或两个对角为 pt1 和 pt2 的填充矩形。

```cpp
void cv::line (InputOutputArray img, Point pt1, Point pt2, const Scalar &color, int thickness=1, int lineType=LINE_8, int shift=0)
```

绘制连接两点的线段。函数`line`绘制图像中 pt1 和 pt2 点之间的线段。

```cpp
void cv::putText (InputOutputArray img, const String &text, Point org, int fontFace, double fontScale, Scalar color, int thickness=1, int lineType=LINE_8, bool bottomLeftOrigin=false)
```

**绘制一个文本字符串**。函数 `cv::putText` 在图像中呈现指定的文本字符串。无法使用指定字体呈现的符号将替换为问号。

```cpp
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
	//Blank Image
	Mat img(512, 512, CV_8UC3, Scalar(255, 255, 255));//(512,512)为图片大小，CV8UC3中8表示每个像素的值从0到255，3表示3个颜色通道BGR,Scalar(255, 0, 0)表示图像将具有的颜色

	circle(img, Point(256, 256), 155, Scalar(0, 69, 255), FILLED);//第一个参数：输出的目标图片，第二个参数是圆心坐标，第三个参数是圆大小，第四个参数是颜色，第五个参数是厚度（可以不写），想要填充可以填FILLED
	rectangle(img, Point(130, 226), Point(382, 286), Scalar(255, 255, 255), -1);//第一个Point给矩形左上角坐标，第二个Point给矩形右下角坐标
	line(img, Point(130, 296), Point(382, 296), Scalar(255, 255, 255), 2);

	putText(img, "SJN's Workshop", Point(137, 262), FONT_HERSHEY_DUPLEX, 0.95, Scalar(0, 69, 255), 2);

	imshow("Image", img);
	waitKey(0);

	return 0;
}

```

## 5.  透视变换

```cpp
Mat cv::getPerspectiveTransform (const Point2f src[], const Point2f dst[])
```

**返回相应 4 个点对的 3x3 透视变换**。

```cpp
void cv::warpPerspective (InputArray src, OutputArray dst, InputArray M, Size dsize, int flags=INTER_LINEAR, int borderMode=BORDER_CONSTANT, const Scalar &borderValue=Scalar())
```

**对图像应用透视变换**

```cpp
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

float w = 250, h = 350;
Mat matrix, imgWarp;

int main()
{
	string path = "Resources/cards.jpg";
	Mat img = imread(path);

    //当前位姿
	Point2f src[4] = { {529, 142}, {771, 190}, {405, 395}, {674, 457} };//Point2f表示浮点数
    //目标位姿
	Point2f dst[4] = { {0.0f, 0.0f}, {w, 0.0f}, {0.0f, h}, {w, h} };

    //转换矩阵
	matrix = getPerspectiveTransform(src, dst);
	warpPerspective(img, imgWarp, matrix, Point(w, h));

    //标识四角
	for (int i = 0; i < 4; i++) {
		circle(img, src[i], 10, Scalar(0, 0, 255), FILLED);
	}

	imshow("Image", img);
	imshow("ImageWarp", imgWarp);
	waitKey(0);

	return 0;
}
```

## 6.  颜色检测

```cpp
void cv::inRange (InputArray src, InputArray lowerb, InputArray upperb, OutputArray dst)
```

检查数组元素是否位于其他两个数组的元素之间。

```cpp
void cv::namedWindow (const String &winname, int flags = WINDOW_AUTOSIZE)
```

**创建一个窗口**。函数`namedWindow`创建一个可用作图像和轨迹栏占位符的窗口。创建的窗口由它们的名称引用。如果同名的窗口已经存在，则该函数不执行任何操作。

```cpp
int cv::createTrackbar (const String &trackbarname, const String &winname, int *value, int count, TrackbarCallback onChange = 0, void *userdata = 0)
```

创建一个trackbar并将其附加到指定窗口。函数createTrackbar创建一个具有指定名称和范围的trackbar（滑块或范围控件），分配一个变量值作为与trackbar同步的位置，并指定回调函数onChange为 在跟踪栏位置变化时被调用。创建的轨迹栏显示在指定的窗口winname中。

```cpp
//学习检测图片中的颜色，来创建特定对象的对象检测器
#include<opencv2/imgcodecs.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include<iostream>

using namespace std;
using namespace cv;

/// <summary>
/// Color Detection
/// </summary>


int hmin = 0, smin = 0, vmin = 0;
int hmax = 179, smax = 255, vmax = 255;//如何确定这6个值，每次都更改所有这些再次运行很痛苦 -->创建跟踪栏（使我们可以实时更改这些值）
void main() {
	string path = "Resources/shapes.png";
Mat img=imread(path);
Mat imgHSV,mask;
cvtColor(img, imgHSV, COLOR_BGR2HSV);//转换图像到HSV空间，在其中查找颜色更加容易
namedWindow("Trackbars", (640, 200));//(640,200)是尺寸
	//运行时，把3个min的都移到最小值，把3个max的都移到最大值，然后移动使其保持为白色
createTrackbar("Hue Min", "Trackbars", &hmin, 179);//对于hue色相饱和度最大180,对于另外两个色相饱和度最大255
createTrackbar("Hue Max", "Trackbars", &hmax, 179);
createTrackbar("Sat Min", "Trackbars", &smin, 255);
createTrackbar("Sat Max", "Trackbars", &smax, 255);
createTrackbar("Val Min", "Trackbars", &vmin, 255);
createTrackbar("Val Max", "Trackbars", &vmax, 255);

	while (true) {
//		//检查数组元素是否位于其他两个数组的元素之间。
//		//imgHSV为输入图像，mask为输出图像

	Scalar lower(hmin, smin, vmin);
	Scalar upper(hmax, smax, vmax);
//		inRange(imgHSV, lower, upper, mask);//定义颜色下限和上限，因为由于照明和不同的阴影，颜色的值将不完全相同，会是一个值的范围
	imshow("Image", img);
	imshow("Image HSV", imgHSV);
	 imshow("Image mask", mask);
	waitKey(1);//增加延时
}
}
```

## 7.  形状/轮廓检测

```cpp
void cv::findContours(InputOutputArray image, OutputArrayOfArrays contours, OutputArray hierarchy, int mode, int method, Point offset = Point())
```

**在二值图像中查找轮廓**

| 参数      | 含义                                                         |
| --------- | ------------------------------------------------------------ |
| image     | 二值输入图像                                                 |
| contours  | 检测到的轮廓，每个轮廓都存储为点向量（例如 `std::vector<std::vector<cv::Point> >`） |
| hierarchy | 可选的输出向量（例如` std::vector<cv::Vec4i>`），包含有关图像拓扑的信息 |
| mode      | 轮廓检索模式                                                 |
| method    | 轮廓近似方式                                                 |
| offset    | 每个轮廓点移动的可选偏移量                                   |

```cpp
double cv::contourArea(InputArray contour, bool oriented=false)
```

计算轮廓区域

```cpp
double cv::arcLength(InputArray curve, bool closed)
```

计算曲线长度或闭合轮廓周长

```cpp
void cv::approxPolyDP(InputArray curve, OutputArray approxCurve, double epsilon, bool closed)
```

函数`cv::approxPolyDP`用另一个具有较少顶点的曲线/多边形来逼近一条曲线或多边形，以使它们之间的距离小于或等于指定的精度。

```cpp
Rect cv::boundingRect(InputArray array)
```

计算并返回指定点集或灰度图像非零像素的最小上边界矩形。

```cpp
void cv::drawContours(InputOutputArray image, InputArrayOfArrays contours, int contourIdx, const Scalar &color, int thickness = 1, int lineType = LINE_8, InputArray hierarchy = noArray(), int maxLevel = INT_MAX, Point offset = Point())
```

绘制轮廓轮廓或填充轮廓。如果厚度≥0，该函数在图像中绘制轮廓轮廓，如果厚度<0，则填充轮廓所包围的区域。



```cpp
//学习如何检测形状或图像中的轮廓
#include<opencv2/imgcodecs.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include<iostream>

using namespace std;
using namespace cv;

/// <summary>
/// Shapes
/// </summary>


//! 获取轮廓
void getContours(Mat imgDil,Mat img) {//imgDil是传入的扩张边缘的图像用来查找轮廓，img是要在其上绘制轮廓的图像
	vector<vector<Point>> contours;//轮廓检测到的轮廓。每个轮廓线存储为一个点的向量
	
	vector<Vec4i> hierarchy;//包含关于映像拓扑的信息  typedef Vec<int, 4> Vec4i;具有4个整数值
	
	//在二值图像中查找轮廓。该函数利用该算法从二值图像中提取轮廓
	findContours(imgDil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	//drawContours(img, contours, -1, Scalar(255, 0, 255), 2);//img：要绘制轮廓在什么图片上，contours：要绘制的轮廓，-1定义要绘制的轮廓号（-1表示所有轮廓），Saclar表示轮廓颜色，2表示厚度
	vector<vector<Point>> conPoly(contours.size());//conploy的数量应小于contours
	vector<Rect> boundRect(contours.size());
	//过滤器：通过轮廓面积来过滤噪声
	for (int i = 0; i < contours.size(); i++) {//遍历检测到的轮廓
		int area = contourArea(contours[i]);
		
		//cout << area << endl;
		
		string objectType;
		if (area > 1000) {//轮廓面积＞1000才绘制
			//计算轮廓周长或凹坑长度。该函数计算了曲线长度和封闭的周长。
			float peri = arcLength(contours[i], true);//计算封闭轮廓周长
			approxPolyDP(contours[i], conPoly[i],0.02*peri,true);//以指定的精度近似多边形曲线。第二个参数conPloy[i]存储近似的结果，是输出。
			
			
			boundRect[i]=boundingRect(conPoly[i]);//计算边界矩形
			
			int objCor = (int)conPoly[i].size();//找近似多边形的角点,三角形有3个角点，矩形/正方形有4个角点，圆形>4个角点
			cout << objCor << endl;
			if (objCor == 3) {objectType = "Tri";}
			else if (objCor == 4) {
				float aspRatio = (float)boundRect[i].width / (float)boundRect[i].height;//宽高比
				if (aspRatio > 0.95 && aspRatio < 1.05) { objectType = "Square";}//矩形的宽高比不会正好等于1
				else objectType = "Rect";
			}
			else if (objCor > 4) { objectType = "Circle";}
			
			drawContours(img, conPoly, i, Scalar(255, 0, 255), 2);
			rectangle/*绘制边界矩形*/(img, boundRect[i].tl()/*tl()：topleft矩形左上角坐标*/, boundRect[i].br()/*br()：bottom right矩形右下角坐标*/, Scalar(0, 255, 0), 5);
			putText(img, objectType, {boundRect[i].x,boundRect[i].y-5}/*文字坐标*/, FONT_HERSHEY_PLAIN, 1, Scalar(0, 69, 255), 2);
		}
	}
}

void main() {
	string path = "Resources/shapes.png";
	Mat img=imread(path);
	
	//在检测形状前，对图片预处理：转换为灰度、添加高斯模糊、使用Canny边缘检测器、扩张边缘
	Mat imgGray, imgBlur, imgCanny, imgDil, imgErode;
	cvtColor(img, imgGray, COLOR_BGR2GRAY);//cvt是convert的缩写，将图像从一种颜色空间转换为另一种颜色空间。
	GaussianBlur(imgGray, imgBlur,Size(3,3),3,0);//使用高斯滤波器模糊图像。该函数将源图像与指定的高斯核进行卷积,Size(7,7)是核大小,数字越大越模糊
	Canny(imgBlur, imgCanny, 25, 75);//边缘检测，阈值1，2可调，目的：显示更多的边缘
	Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));//创建一个核，增加Size（只能是奇数）会扩张/侵蚀更多
	dilate(imgCanny, imgDil, kernel);//扩张边缘（增加边缘厚度）
	
	getContours(imgDil,img);//img是在其上绘轮廓的图片

	imshow("Image", img);
	imshow("Image Gray", imgGray);
	imshow("Image Blur", imgBlur);
	imshow("Image Canny", imgCanny);
	imshow("Image Dil", imgDil);*/
	waitKey(0);//增加延时，0表示无穷
}
```



## 8.  人脸检测

```cpp
#include <opencv2/objdetect.hpp>
```



```cpp
class cv::CascadeClassifier
```

**用于对象检测的级联分类器类**。

```cpp
bool load (const String &filename)
```

从文件加载分类器。

```cpp
bool empty() const
```

检查分类器是否已加载

```cpp
void detectMultiScale(InputArray image, std::vector<Rect> &objects, double scaleFactor=1.1, int minNeighbors=3, int flags=0, Size minSize=Size(), Size maxSize=Size())
```

检测输入图像中不同大小的对象。检测到的对象作为矩形列表返回。

```cpp
int main() {
    std::string path = "Resources/test.png"；
    cv::Mat img = cv::imread(path);
    //加载级联分类器
    cv::CascadeClassifier faceCascade;
  
    //加载训练模型
    faceCascade.load("Resources/haarcascade_frontalface_default.xml");
//判断是否加载成功
    if (faceCascade.empty()) {
        std::cout << "empty" << std::endl;
    }
    
    //存放人脸
    std::vector<cv::Rect> faces;
    faceCascade.detectMultiScale(img, faces, 1.1, 10);
//标识人脸
    for (int i = 0; i < faces.size(); i++) {
        cv::rectangle(img, faces[i].tl(), faces[i].br(), cv::Scalar(255, 255, 255), 3);
    }

    imshow("Image", img);
    cv::waitKey(0);
}
```

  # OpenCV 库

## finContours

[知乎专栏](https://zhuanlan.zhihu.com/p/144807771)

```cpp
findContours(InputOutputArray image, OutputArrayOfArrays contours, 
             OutputArray hierarchy, int mode, int method, Point offset = Point());
```

| 参数        | 含义                                                         |
| ----------- | ------------------------------------------------------------ |
| image       | 单通道图像矩阵，可以是灰度图，但更常用的是**二值图像**，一般是经过Canny、拉普拉斯等边缘检测算子处理过的二值图像； |
| contours    | contours定义为“**vector<vector\<Point>> contours**”，是一个双重向量（**向量内每个元素保存了一组由连续的Point构成的点的集合的向量**），每一组点集就是一个轮廓，有多少轮廓，contours就有多少元素； |
| hierarchy   | <font color = SandyBrown>层级关系</font>，hierarchy定义为“**vector\<Vec4i> hierarchy**”，Vec4i的定义：typedef Vec<int, 4> Vec4i;（向量内每个元素都包含了4个int型变量），所以从定义上看，**hierarchy是一个向量，向量内每个元素都是一个包含4个int型的数组**。向量hierarchy内的元素和轮廓向量contours内的元素是一一对应的，向量的容量相同。hierarchy内每个元素的4个int型变量是hierarchy\[i][0] ~ hierarchy\[i][3]，分别表示当前轮廓 i 的后一个轮廓、前一个轮廓、父轮廓和内嵌轮廓的编号索引。如果当前轮廓没有对应的后一个轮廓、前一个轮廓、父轮廓和内嵌轮廓，则相应的hierarchy[i][*]被置为-1。 |
| mode        | 检索模式                                                     |
| method      | 近似方法                                                     |
| Point偏移量 | 所有的轮廓信息相对于原始图像对应点的偏移量，**相当于在每一个检测出的轮廓点上加上该偏移量，并且Point还可以是负值**！ |



| 检索方法         | 内容                                                         |
| ---------------- | ------------------------------------------------------------ |
| CV_RETR_EXTERNAL | **只检测最外围轮廓**，包含在外围轮廓内的内围轮廓被忽略；     |
| CV_RETR_LIST     | **检测所有的轮廓**，包括内围、外围轮廓，但是检测到的轮廓不建立等级关系，彼此之间独立，没有等级关系，这就意味着**这个检索模式下不存在父轮廓或内嵌轮廓**，所以hierarchy向量内所有元素的第3、第4个分量都会被置为-1，具体下文会讲到； |
| CV_RETR_CCOMP    | 检测所有的轮廓，但所有轮廓只建立两个等级关系，外围为顶层，若外围内的内围轮廓还包含了其他的轮廓信息，则内围内的所有轮廓均归属于顶层； |
| CV_RETR_TREE     | 检测所有轮廓，所有轮廓建立一个等级树结构。外层轮廓包含内层轮廓，内层轮廓还可以继续包含内嵌轮廓。 |



| 近似方法                  | 内容                                                         |
| ------------------------- | ------------------------------------------------------------ |
| CV_CHAIN_APPROX_NONE      | 保存物体边界上所有连续的轮廓点到contours向量内；             |
| CV_CHAIN_APPROX_SIMPLE    | **仅保存轮廓的拐点信息**，把所有轮廓拐点处的点保存入contours向量内，拐点与拐点之间直线段上的信息点不予保留； |
| CV_CHAIN_APPROX_TC89_L1   | 使用teh-Chinl chain 近似算法;                                |
| CV_CHAIN_APPROX_TC89_KCOS | 使用teh-Chinl chain 近似算法。                               |

```cpp
//hierachy
[Next, Previous, First_Child, Parent]
```

**`vector<Vec4i> hierarchy`**，存储轮廓的层级关系

![](/home/suyu/Typora/View/image/OpenCV/a150569a11b74ab8bf5735251b77e95b.png)

- Next：表示属于同一个层级`hierarchy`的下一个轮廓
- Previous：表示同一层级中之前的的那个轮廓
- First_Child：表示当前轮廓的第一个子轮廓
- Parent：表示当前轮廓的父轮廓

**`vector<vector\<Point>> contours`**，存储轮廓

```cpp
contours.size()        //表示轮廓的个数
contours[i].size()   //表示第i个轮廓中点的个数
    
    for (int i = 0; i < Contours.size(); i++)
	{
	数
		for (int j = 0;j < Contours[i].size();j++)
		{
		cout << "输出轮廓第"<<j+1<<"个点的坐标  " << Contours[i][j] << endl;
	    cout<< "输出轮廓i的第j个点的x坐标  " <<Contours[i][j].x << endl;  //试试看输出轮廓第一个点的横坐标 x
        cout<< "输出轮廓i的第j个点的y坐标  " <<Contours[i][j].y << endl;  //试试看输出轮廓第一个点的纵坐标 y
        }
	  }

```



## arcLength

```cpp
double arcLength( InputArray curve, bool closed );
```



## approxPolyDP

```cpp
void approxPolyDP(InputArray curve, OutputArray approxCurve, double epsilon, bool closed)
```

| 参数        | 内容                                           |
| ----------- | ---------------------------------------------- |
| curve       | 一般是由图像的轮廓点组成的点集                 |
| approxCurve | 表示输出的多边形点集                           |
| epsilon     | 近似值的精确度，即原始曲线和近似曲线的最大距离 |
| closed      | 表示输出的多边形是否封闭                       |





## cv::threshold

```cpp
double cv::threshold	(	InputArray 	src,
OutputArray 	dst,
double 	thresh,
double 	maxval,
int 	type 
)		
```



| 参数   | 含义                                              |
| ------ | ------------------------------------------------- |
| src    | 源图像，可以为8位的灰度图，也可以为32位的彩色图像 |
| dst    | 输出图像                                          |
| thresh | 阈值                                              |
| maxval | 二值图像中灰度最大值                              |
| type   | 阈值操作类型，具体的阈值操作实现如下图所示        |

![](/home/suyu/默认/图片/20170810122741046.png)

![](/home/suyu/默认/图片/20170810122752738.png)



hhh
