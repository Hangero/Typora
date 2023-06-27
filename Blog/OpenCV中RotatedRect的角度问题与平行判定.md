# OpenCV中RotatedRect的角度问题与平行判定

在运用OpenCV的过程中，想对两个细窄长方形进行平行判定，但是因为`RotatedRct`的角度问题，走了很多弯路。

## RotatedRect

**OpenCV中，以左上角角点为O点，竖直向下为Y正方向，水平向右为X正方向**

OpenCV的`RotatedRct`在标志四个角点时，以Y值最大(最下方)的点为p[0]，依次**顺时针**标记p[1]，p[2]，p[3]。其中，p[0]与p[3]，p[1]与p[2]之间的线段为`width` ，而p[0]与p[1]，p[2]与p[3]之间的线段为`height` 。取**顺时针为角度正方向**，`angle`正是由X轴正方向转向`width`的角度（负值）。而在我们朴素的认知中，一般直觉是较长一边与X轴正方向的夹角。所以在出现图4情况时，很容易理解成`-120`，而不是`-30`。



![](/home/suyu/默认/图片/20201105204818434.png)

​                                                                                (网图，侵删)

## 平行的判定

进行平行判定时，可以考虑方向向量与叉乘

1. 获得单位方向向量

```cpp
 //vertices为存放旋转矩形角点的数组
if (width < height) {
     //即图4这种情况
     //注意除以长度获得单位向量
            DirectionVector.x = (vertices[1].x - vertices[0].x) / height;
            DirectionVector.y = (vertices[1].y - vertices[0].y) / height;
        } else {
     //即图123情况
            DirectionVector.x = (vertices[3].x - vertices[0].x) / width;
            DirectionVector.y = (vertices[3].y - vertices[0].y) / width;
        }
```

2. 求叉乘（外积）

```cpp
 //判断是否平行
            float CrossProduct = LeftRect.DirectionVector.x * RightRect.DirectionVector.y -
                                 LeftRect.DirectionVector.y * RightRect.DirectionVector.x;
            if (asin(abs(CrossProduct)) > 15) continue;
            float MeanAngle = std::min(LeftRect.angle, RightRect.angle);
```



## 寻找左上角点

笔者的应用场景是：检测左右灯条，并以两灯条为边拟合成一个矩形。。在这个场景下，矩形的边近似平行与两个坐标轴，x+y的最小值99%以上都是左上角点，因此找到x+y的最小值，再重排序。

```cpp
//寻找左上角点，并重新排序
            cv::Point2f temp[4];
            Armor.points(temp);
            int flag=0;
            for(int i =0;i<4;i++){
                float Point0=temp[0].x+temp[0].y;

                if(temp[i].x+temp[i].y>Point0)continue;
                else{
                    Point0=temp[i].x+temp[i].y;
                    flag=i;
                }
            }
            cv::Point2f vertices[4];
            for(int i =0;i<4;i++){

                vertices[i]=temp[(flag+i)%4];
            }
```

