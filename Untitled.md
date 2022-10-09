```cmake
# cmake version
CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

# project name
PROJECT(opencvcourse)

# find OpenCV PCL
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(PCL 1.8 REQUIRED) 
# show the message of OpenCV
MESSAGE(STATUS "OpenCV library status:")
MESSAGE(STATUS "    version: 	${OpenCV_VERSION}")
MESSAGE(STATUS "    headers: 	${OpenCV_INCLUDE_DIRS}")
MESSAGE(STATUS "    libraries: 	${OpenCV_LIBS}")

# show the message of PCL
MESSAGE(STATUS "PCL library status:")
MESSAGE(STATUS "    version: 	${PCL_VERSION}")
MESSAGE(STATUS "    headers: 	${PCL_INCLUDE_DIRS}")
MESSAGE(STATUS "    libraries: 	${PCL_LIBS}")

# link headers
INCLUDE_DIRECTORIES({OpenCV_INCLUDE_DIRS}  ./include)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS} ./include) 

#设置输出路径
SET (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(PCL_INCLUDE_DIRS /usr/include/pcl-1.8)

link_directories(${PCL_LIBRARY_DIRS})    
#SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_definitions(${PCL_DEFINITIONS})  

# 添加源代码文件到SRC_LIST变量中
AUX_SOURCE_DIRECTORY(src SRC_LIST)

# 生成可执行文件
ADD_EXECUTABLE(pcltest ${SRC_LIST})



# after ADD_EXECUTABLE，为生成文件target添加库
TARGET_LINK_LIBRARIES(pcltest ${OpenCV_LIBS})
target_link_libraries(pcltest ${PCL_LIBRARIES})
target_link_libraries(pcltest ${PCL_COMMON_LIBRARY})
target_link_libraries(pcltest ${PCL_OCTREE_LIBRARY})
target_link_libraries(pcltest ${PCL_IO_LIBRARY})




```

```cpp
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>

double fx = 611.85382, fy = 611.67426, cx = 638.15198, cy = 366.2175;
double camera_factor = 100;
int main() {
    cv::Mat img, depth;
    std::string path_img = "Resources/1.png";
    std::string path_depth = "Resources/1_depth.png";
    img = cv::imread(path_img);
    depth = cv::imread(path_depth);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (int m = 0; m < depth.rows; m++) {
        for (int n = 0; n < depth.cols; n++) {
            ushort d = depth.ptr<ushort>(m)[n];
            if (d == 0) {
                continue;
            }
            pcl::PointXYZRGBA p;
            p.z = double(d) / camera_factor;
            p.x = (n - cx) * p.z / fx;
            p.y = (m - cy) * p.z / fy;

            p.b = img.ptr<uchar>(m)[n * 3];
            p.g = img.ptr<uchar>(m)[n * 3 + 1];
            p.r = img.ptr<uchar>(m)[n * 3 + 2];

            cloud->points.push_back(p);
        }
    }

    cloud->height = 1;
    cloud->width = cloud->points.size();

    std::cout << "point cloud size = " << cloud->points.size() << std::endl;

    cloud->is_dense = false;

    try {
        pcl::io::savePCDFile("Resources/1.pcd", *cloud);
    } catch (pcl::IOException &e) {
        std::cout << e.what() << std::endl;
    }

    pcl::visualization::CloudViewer viewer("1");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {
    }
    cloud->points.clear();
    cout << "Point cloud saved." << endl;

    // cv::imshow("img", img);
    // cv::imshow("depth", depth);
    // cv::waitKey(0);
    return 0;
}
```

