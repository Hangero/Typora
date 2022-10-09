# Image_processing

## BMP

### 文件结构

BMP文件由4部分组成：

1. 位图文件头(bitmap-file header)

2. 位图信息头(bitmap-informationheader)

3. 颜色表(color table)

4. 颜色点阵数据(bits data)

| **BMP文件组成**                                              | **数据结构表示** |
| ------------------------------------------------------------ | ---------------- |
| [位图](https://so.csdn.net/so/search?q=位图&spm=1001.2101.3001.7020)文件头 | BITMAPFILEDEADER |
| 位图信息头                                                   | BITMAPINFOHEADER |
| 颜色信息                                                     | RGBQUAD          |
| 位图数据                                                     | BYTE             |

####  位图文件头

位图文件头主要包括文件大小、文件类型、图像数据偏移文件头的长度等信息，其结构占14个字节，定义如下

```cpp
typedef struct tagBITMAPFILEHEADER{
 
WORD bfType;          //占2个字节，文件类型，一般为“BM”
 
DWORD bfSize;         //占4个字节，位图文件的大小
 
WORD bfReserved1;     //占2个字节，文件保留字1，0（一般用不到）
 
WORD bfReserved2;     //占2个字节，文件保留字2，0（一般用不到）
 
DWORD bfOffBits;      //占4个字节，图像数据偏移文件头的长度
 
} BITMAPFILEHEADER;
```

[typedef struct](https://blog.csdn.net/qq_41848006/article/details/81321883)

#### 位图信息头

位图信息头主要包括位图图像的大小、像素深度、图像是否压缩、图像所用眼色数等，其结构占用40个字节，定义如下：

```cpp
typedef struct tagBITMAPINFOHEADER{ 
 
DWORD biSize;         //该结构所需的字节数
 
LONG biWidth;         //图像宽度，以像素为单位
 
LONG biHeight;        //图像高度，以像素为单位
 
WORD biPlanes;        //目标设备的平面数，设置为1
 
WORD biBitCount       //每个像素所需的bit数。“1”表示单色图像；设置“4”，表示有16种颜色；设置为“8”，表示有256种颜色；设置为“24”就是真彩色图像，表示的颜色有16 777 216种颜色，且一个像素由3个像素表色，分别代表R，G，B分量。
 
DWORD biCompression;  //位图的压缩类型，“0”表示未压缩
 
DWORD biSizeImage;    //位图大小，以字节为单位
 
LONG biXPelsPerMeter; //水平分辨率
 
LONG biYPelsPerMeter; //垂直分辨率
 
DWORD biClrUsed;      //实际使用颜色表中的颜色数
 
DWORD biClrImportant; //显示过程中主要的颜色数
 
} BITMAPINFOHEADER; 
```

#### 颜色信息

颜色信息包含所要用到的颜色表，显示图像时需要这个表来生成调色板。但如果是一幅真彩色的图像，则没有这一块信息。颜色表中有若干个表项，每一项都是RGBQUAD类型的结构，定义一种颜色。其结构如下：

```cpp
typedef struct tagRGBQUAD { 
 
BYTE rgbBlue;        //蓝色分量（0-255）
 
BYTE rgbGreen;       //绿色分量（0-255）
 
BYTE rgbRed;         //红色分量（0-255）
 
BYTE rgbReserved;}   //保留，“0”
 
RGBQUAD;
```

