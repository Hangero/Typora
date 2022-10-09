# PCL





## 数据类型

### PointCloud

- 定义

我们定义一个点云类型可能类似于以下形式：

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);//点云指针类型
pcl::PointCloud<pcl::PointXYZ> cloud;//点云对象
pcl::PointXYZ  point;//单个点云
```

- 访问形式

```cpp
cloud.points[0].x;//访问第一个点的x
cloudPtr->points[0].x;//访问第一个点的x
point.x;//访问单个点的x
```

- 常用的成员变量和方法的使用与说明

| 属性                 | 说明                                                         |
| -------------------- | ------------------------------------------------------------ |
| cloud.size()         | 点云的个数                                                   |
| cloud.height         | 有序列点云：点云行数(类似于图像)；`无序点云：height=1`       |
| cloud.width          | 点云个数标识width*height，如果height=1，则size()=width       |
| cloud.points         | PointT的点，它是vector形式，`方便点云访问`cloud.points       |
| cloud.is_dense       | 如果为false，则表示点云中包含inf/NaN这样的点，`使用的时候注意滤除这些点` |
| cloud.isOrganized () | `判断点云是否有序`(一般不使用height=1判断)                   |

###  PointXYZ——x,y,z

| 成员变量 | 成员类型     | 访问形式                       |
| -------- | ------------ | ------------------------------ |
| xyz      | xyz均为float | points[i].data[0]或points[i].x |

xyz三个浮点数附加一个浮点数来`满足存储对齐`，具体类型如下：

```cpp
union
{
    float data[4];
    struct
    {
        float x;
        float y;
        float z;
    };
};
```

###  PointXYZRGBA——x,y,z,r,g,b,a

| 成员变量              | 成员类型                       | 访问形式           |
| --------------------- | ------------------------------ | ------------------ |
| x、y、z、r、g、b和a等 | xyz为float；rgba为std::int32_t | 请查看以下数据结构 |

`因为使用了union，所以可以很方便地单独访问每个颜色通道`。具体数据类型如下：

```cpp
union
{
    float data[4];
    struct
    {
        float x;
        float y;
        float z;
    };
};
union
{
    union
    {
        struct
        {
            std::uint8_t b;
            std::uint8_t g;
            std::uint8_t r;
            std::uint8_t a;
        };
        float rgb;
    };
    std::uint32_t rgba;
};

```

###  InterestPoint——x, y, z, strength

| 成员变量          | 成员类型      | 访问形式           |
| ----------------- | ------------- | ------------------ |
| x, y, z, strength | 都是float类型 | 请查看以下数据结构 |

### PointNormal——x, y, z,normal,curvature

| 成员变量                 | 成员类型      | 访问形式           |
| ------------------------ | ------------- | ------------------ |
| x, y, z,normal,curvature | 都是float类型 | 请查看以下数据结构 |

这里包含了3个独立的union，其中normal为曲面的法向量，curvature为曲率

## 实例

```cpp
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;
// 定义点云类型

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;
// 主函数 

int main(int argc, char** argv)
{
	// 读取rgb.png和depth.png到图像矩阵里
	cv::Mat rgb, depth;
	// 使用cv::imread()来读取图像
	rgb = cv::imread("color.png");
	cout << "read rgb" << endl;
	// rgb 图像是8UC3的彩色图像
	// depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改
	depth = cv::imread("depth.png");
	cout << "read depth" << endl;

	// 使用智能指针，创建一个空点云。这种指针用完会自动释放。
	PointCloud::Ptr cloud(new PointCloud);
	// 遍历深度图
	for (int m = 0; m < depth.rows; m++)
		for (int n = 0; n < depth.cols; n++)
		{
			// 获取深度图中(m,n)处的值
			ushort d = depth.ptr<ushort>(m)[n];
			// d 可能没有值，若如此，跳过此点
			if (d == 0)
				continue;
			// d 存在值，则向点云增加一个点

			PointT p;
			// 计算这个点的空间坐标
			p.z = double(d) / camera_factor;
			p.x = (n - camera_cx) * p.z / camera_fx;
			p.y = (m - camera_cy) * p.z / camera_fy;
			// 从rgb图像中获取它的颜色
			// rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
			p.b = rgb.ptr<uchar>(m)[n * 3];
			p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
			p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

			// 把p加入到点云中
			cloud->points.push_back(p);
			cout << cloud->points.size() << endl;
		}

	// 设置并保存点云
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cout << "point cloud size = " << cloud->points.size() << endl;
	cloud->is_dense = false;
	try {
		pcl::io::savePCDFile("pcd.pcd", *cloud);
	}
	//异常处理
	catch (pcl::IOException &e) {
		cout << e.what() << endl;
	}

	//显示点云图
	pcl::visualization::CloudViewer view("Simple Cloud Viewer");//直接创造一个显示窗口
	view.showCloud(cloud);//再这个窗口显示点云

	pcl::io::savePCDFileASCII("projectpointcloud.pcd", *cloud);
	// 清除数据并退出
	cloud->points.clear();
	cout << "Point cloud saved." << endl;
	return 0;
}

```

## 基础

### PCDReader

```cpp
  pcl::PCDReader reader;
  //创建一个`PointCloud<PointXYZ>` boost共享指针并进行实例化
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new     pcl::PointCloud<pcl::PointXYZ>);
  //读取pcd文件，用指针传递给cloud
  reader.read ("xcmg-test1.pcd", *cloud);
std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;

```



## Filtering

### PassThrough

```cpp
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    // define the Point Clouds structures, 
    //fill in the input cloud, and display its content to screen.
    cloud->width = 5;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (auto& point : *cloud) {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    std::cerr << "Cloud before filtering: " << std::endl;
    for (const auto& point : *cloud)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);// 传入点云数据
    pass.setFilterFieldName("z");// 设置操作的坐标轴
    pass.setFilterLimits(0.0, 1.0); // 设置坐标范围
    pass.setFilterLimitsNegative(true); //反向取
    pass.filter(*cloud_filtered); // 进行滤波输出

    std::cerr << "Cloud after filtering: " << std::endl;
    for (const auto& point : *cloud_filtered)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;
  
    
    //通过重复使用直通滤波就可以进行三维区间的滤波。
    pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-5.0, 5.0);
	// pass.setFilterLimitsNegative(true);
	pass.filter(*cloud_filtered2);
	// filter range Y-axis
	pass.setInputCloud(cloud_filtered2);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-5.0, 5.0);
	pass.filter(*cloud_filtered3);
    // filter range Z-axis
	pass.setInputCloud(cloud_filtered3);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-0.5, 3.0);
	pass.filter(*cloud_filtered);
```

- cout：写到标准输出的ostream对象；
- cerr：输出到标准错误的ostream对象，通常用来输出警告和错误信息给程序的使用者；

1. cout经过缓冲后输出，默认情况下是显示器。这是一个被缓冲的输出，是标准输出；它在内存中对应开辟了一个缓冲区,用来存放流中的数据,当向cout流插入一个endl,不论缓冲区是否漫了,都立即输出流中所有数据,然后插入一个换行符. 可以被输出到文件，即可以重定向输出。
2. cerr不经过缓冲而直接输出，一般用于迅速输出出错信息，是标准错误，默认情况下被关联到标准输出流，但它不被缓冲，也就说错误消息可以直接发送到显示器，而无需等到缓冲区或者新的换行符时，才被显示。
   

### VoxelGrid

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


int main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read ("table_scene_lms400.pcd", *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

  pcl::PCDWriter writer;
  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
```

### StatisticalOutlierRemoval

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>


int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZ>("Resources/table_scene_lms400.pcd", *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

    sor.setNegative(true);//////////
    sor.filter(*cloud_filtered);
    writer.write<pcl::PointXYZ>("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
```



```cpp
std::cerr << *cloud << std::endl;

points[]: 460400
width: 460400
height: 1
is_dense: 1
sensor origin (xyz): [0, 0, 0] / orientation (xyzw): [0, 0, 0, 1]
```

输出的并非图像，而是`cloud`的属性

| 属性                 | 说明                                                         |
| -------------------- | ------------------------------------------------------------ |
| cloud.size()         | 点云的个数                                                   |
| cloud.height         | 有序列点云：点云行数(类似于图像)；`无序点云：height=1`       |
| cloud.width          | 点云个数标识width*height，如果height=1，则size()=width       |
| cloud.points         | PointT的点，它是vector形式，`方便点云访问`cloud.points       |
| cloud.is_dense       | 如果为false，则表示点云中包含inf/NaN这样的点，`使用的时候注意滤除这些点` |
| cloud.isOrganized () | `判断点云是否有序`(一般不使用height=1判断)                   |

### RadiusOutlierRemoval

```cpp
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

int
 main (int argc, char** argv)
{
    //ensures that the user has specified a command line argument:
  if (argc != 2)
  {
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 1;
  cloud->resize (cloud->width * cloud->height);

  for (auto& point: *cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  if (strcmp(argv[1], "-r") == 0){
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius (2);
    outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter (*cloud_filtered);
  }
  else if (strcmp(argv[1], "-c") == 0){
      //ConditionalRemoval，它可以一次删除满足对输入的点云设定的一个或多个条件指标的所有数据点。
    // build the condition
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
      pcl::ConditionAnd<pcl::PointXYZ> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);
    condrem.setKeepOrganized(true);
    // apply filter
    condrem.filter (*cloud_filtered);
  }
  else{
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
  std::cerr << "Cloud before filtering: " << std::endl;
  for (const auto& point: *cloud)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;
  // display pointcloud after filtering
  std::cerr << "Cloud after filtering: " << std::endl;
  for (const auto& point: *cloud_filtered)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;
  return (0);
}

```



```cpp
int strcmp(const char s1,const char s2);

//自左向右逐个按照ASCII码值进行比较，直到出现不同的字符或遇’\0’为止。
//如果返回值 < 0，则表示 s1 小于 s2。
//如果返回值 > 0，则表示 s1 大于 s2。
//如果返回值 = 0，则表示 s1 等于 s2。
```

After you have made the executable, you can run it. If you would like to use ConditionalRemoval then simply do:

```
$ ./remove_outliers -c
```

Otherwise, if you would like to use RadiusOutlierRemoval, simply do:

```
$ ./remove_outliers -r
```



### Projecting points using a parametric model

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (auto& point: *cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);//(-1,1)间的随机数
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before projection: " << std::endl;
  for (const auto& point: *cloud)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;

  // Create a set of planar coefficients with X=Y=0,Z=1
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;

  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  std::cerr << "Cloud after projection: " << std::endl;
  for (const auto& point: *cloud_projected)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;

  return (0);
}
```

- 想要拷贝元素：for(auto x:range)

- 想要修改元素 : for(auto &&x:range)

- 想要只读元素：for(const auto& x:range)

## I/O

### Reading Point Cloud data from PCD

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  for (const auto& point: *cloud)
    std::cout << "    " << point.x
              << " "    << point.y
              << " "    << point.z << std::endl;

  return (0);
}
```

### Writing Point Cloud data to PCD files

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 5;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for (auto& point: cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.size () << " data points to test_pcd.pcd." << std::endl;

  for (const auto& point: cloud)
    std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

  return (0);
}
```

