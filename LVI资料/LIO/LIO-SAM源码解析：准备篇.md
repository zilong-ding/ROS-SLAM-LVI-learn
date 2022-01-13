> 源码详细注释：[https://github.com/smilefacehh/LIO-SAM-DetailedNote](https://link.zhihu.com/?target=https%3A//github.com/smilefacehh/LIO-SAM-DetailedNote)

LIO-SAM的代码十分轻量，只有四个cpp文件，很值得读一读呢。

关于LIO-SAM的论文解读，网上已经有很多文章啦，同系列的LOAM、A-LOAM、LEGO-LOAM等，在网上都可以找到相关的解读文章。所以本文旨在对源代码进行阅读学习，积累一些工程上的经验。这里记录下来，希望可以帮到有需要的同学，如有错误的地方，请您批评指正。

## 目录

-   [LIO-SAM源码解析：准备篇](https://zhuanlan.zhihu.com/p/352039509)
-   [LIO-SAM源码解析(一)：ImageProjection](https://zhuanlan.zhihu.com/p/352120054)
-   [LIO-SAM源码解析(二)：FeatureExtraction](https://zhuanlan.zhihu.com/p/352144126)
-   [LIO-SAM源码解析(三)：IMUPreintegratio](https://zhuanlan.zhihu.com/p/352146800)n
-   [LIO-SAM源码解析(四)：MapOptimization](https://zhuanlan.zhihu.com/p/352148894)

## 一、代码运行

LIO-SAM作者源码地址：[TixiaoShan/LIO-SAM](https://link.zhihu.com/?target=https%3A//github.com/TixiaoShan/LIO-SAM)

LIO-SAM论文：[https://github.com/TixiaoShan/LIO-SAM/blob/master/config/doc/paper.pdf](https://link.zhihu.com/?target=https%3A//github.com/TixiaoShan/LIO-SAM/blob/master/config/doc/paper.pdf)

数据集：Github上作者提供的数据集存放在Google Drive上，由于需要翻墙，这里提供一个百度网盘上的park.bag数据集（带GPS数据），其他的数据集在网上也可以搜到哈。

链接: [https://pan.baidu.com/s/1fMEhAhQwpiESekYAxQYWww](https://link.zhihu.com/?target=https%3A//pan.baidu.com/s/1fMEhAhQwpiESekYAxQYWww) 密码: knf3

## 安装依赖

ROS（注：本文使用Ubuntu20.04 + noetic）

```
sudo apt-get install -y ros-noetic-navigation
sudo apt-get install -y ros-noetic-robot-localization
sudo apt-get install -y ros-noetic-robot-state-publisher
```

GTSAM（注：本文使用GTSAM4.1版本）

```
git clone https://github.com/borglab/gtsam
cd gtsam
mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
sudo make install -j8
```

## 运行LIO-SAM

下载和编译LIO-SAM

```
cd ~/catkin_ws/src
git clone https://github.com/TixiaoShan/LIO-SAM
cd ..
catkin_make
```

参数配置，本文以park.bag为例，该数据集带GPS数据，需要修改如下参数

```
imuTopic: "imu_raw"
gpsTopic: "odometry/gps"
useImuHeadingInitialization: true
```

运行LIO-SAM

```
roslaunch lio_sam run.launch
rosbag play park.bag
```

然后就可以跑起来啦，放一张运行截图。如果发现有报错，不要紧张，下面列几条可能遇到的问题和解决办法。

![](https://pic1.zhimg.com/v2-c1f3fd39d3e426872b7ae27236bf3ea0_b.jpg)

LIO-SAM运行截图，park.bag数据集

## 可能遇到的问题

1.  运行时报错 \[lio\_sam\_mapOptmization-5\] process has died \[pid 260348, exit code -11

```
解决: gtsam编译时带上这个参数，cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
```

2\. 运行时报错 error while loading shared libraries: [libmetis-gtsam.so](https://link.zhihu.com/?target=http%3A//libmetis-gtsam.so/): cannot open shared object file: No such file or directory

```
解决: sudo ln -s /usr/local/lib/libmetis-gtsam.so /usr/lib/libmetis-gtsam.so
```

3\. 保存地图数据

```
savePCDDirectory: "/Downloads/LOAM/" 
注意！！代码会删除这个路径重新创建，不要用已有的路径哦，最好改一下代码把删除路径的逻辑去掉。

修改_TIMEOUT_SIGINT值，默认15s，改为60s。
sudo gedit /opt/ros/noetic/lib/python3/dist-packages/roslaunch/nodeprocess.py
```

4\. Rviz展示

## 二、代码整体框架

首先看看工程目录结构，只有四个cpp文件，从名字看也是简介明了，后面会分开对这四个文件进行源码的阅读。

```
LIO-SAM/
  config/
    params.yaml             # 参数配置
  include/
    utility.h               # 读取参数，提供一些工具方法
  launch/
    run.launch              # 启动文件
  src/
    featureExtraction.cpp   # 点云计算曲率，提取特征（角点、平面点）
    imageProjection.cpp     # 激光点云运动畸变校正
    imuPreintegration.cpp   # imu预积分因子图优化，计算每时刻imu里程计
    mapOptmization.cpp      # scan-to-map匹配，因子图优化，闭环优化
```

对照LIO-SAM作者给出的下面这张系统架构图，我们首先获取一个整体上的印象。

![](https://pic3.zhimg.com/v2-7b18a0fb9e81155c822d1270c130dd16_b.jpg)

LIO-SAM代码结构，数据流向图

首先对于一个SLAM系统，后端优化是一个核心模块，有较早的卡尔曼滤波器、现在流行的图优化、因子图优化。LIO-SAM则采用因子图优化方法，包含四种因子。

LIO-SAM因子：IMU预积分因子，激光里程计因子，GPS因子，闭环因子。

下图是LIO-SAM的因子图结构，变量节点是关键帧。相邻的关键帧之间，通过IMU数据计算预积分，获得位姿变换，构建IMU预积分因子。每个关键帧还有对应的GPS数据参与校正。如果有闭环出现，闭环帧之间可以构建约束。关键帧之间有若干普通帧，这些帧不参与图优化，但是会执行scan-to-map的配准，优化每帧位姿。

![](https://pic4.zhimg.com/v2-2088c446a867f8c267f39d20c4b4c8e3_b.jpg)

LIO-SAM因子图

## 整体流程

1、激光运动畸变校正。利用当前帧起止时刻之间的IMU数据、IMU里程计数据计算预积分，得到每一时刻的激光点位姿，从而变换到初始时刻激光点坐标系下，实现校正。

2、提取特征。对经过运动畸变校正之后的当前帧激光点云，计算每个点的曲率，进而提取角点、平面点特征。

3、scan-to-map匹配。提取局部关键帧map的特征点，与当前帧特征点执行scan-to-map匹配，更新当前帧的位姿。

4、因子图优化。添加激光里程计因子、GPS因子、闭环因子，执行因子图优化，更新所有关键帧位姿。

5、闭环检测。在历史关键帧中找候选闭环匹配帧，执行scan-to-map匹配，得到位姿变换，构建闭环因子，加入到因子图中一并优化。

下面在整体上给出这四个文件对应模块的功能，以及模块之间数据的发布-订阅关系。

## (1) ImageProjection 激光运动畸变校正

**功能简介**

1.  利用当前激光帧起止时刻间的imu数据计算旋转增量，IMU里程计数据（来自ImuPreintegration）计算平移增量，进而对该帧激光每一时刻的激光点进行运动畸变校正（利用相对于激光帧起始时刻的位姿增量，变换当前激光点到起始时刻激光点的坐标系下，实现校正）；
2.  同时用IMU数据的姿态角（RPY，roll、pitch、yaw）、IMU里程计数据的的位姿，对当前帧激光位姿进行粗略初始化。

**订阅**

1.  订阅原始IMU数据；
2.  订阅IMU里程计数据，来自ImuPreintegration，表示每一时刻对应的位姿；
3.  订阅原始激光点云数据。

**发布**

1.  发布当前帧激光运动畸变校正之后的有效点云，用于rviz展示；
2.  发布当前帧激光运动畸变校正之后的点云信息，包括点云数据、初始位姿、姿态角、有效点云数据等，发布给FeatureExtraction进行特征提取。

## (2) FeatureExtraction 点云特征提取

**功能简介**

对经过运动畸变校正之后的当前帧激光点云，计算每个点的曲率，进而提取角点、平面点（用曲率的大小进行判定）。

**订阅**

1.  订阅当前激光帧运动畸变校正后的点云信息，来自ImageProjection。

**发布**

1.  发布当前激光帧提取特征之后的点云信息，包括的历史数据有：运动畸变校正，点云数据，初始位姿，姿态角，有效点云数据，角点点云，平面点点云等，发布给MapOptimization；
2.  发布当前激光帧提取的角点点云，用于rviz展示；
3.  发布当前激光帧提取的平面点点云，用于rviz展示。

## (3) ImuPreintegration IMU预积分

1.  **TransformFusion类**

**功能简介**

主要功能是订阅激光里程计（来自MapOptimization）和IMU里程计，根据前一时刻激光里程计，和该时刻到当前时刻的IMU里程计变换增量，计算当前时刻IMU里程计；rviz展示IMU里程计轨迹（局部）。

**订阅**

1.  订阅激光里程计，来自MapOptimization；
2.  订阅imu里程计，来自ImuPreintegration。

**发布**

1.  发布IMU里程计，用于rviz展示；
2.  发布IMU里程计轨迹，仅展示最近一帧激光里程计时刻到当前时刻之间的轨迹。

2\. **ImuPreintegration类**

**功能简介**

1.  用激光里程计，两帧激光里程计之间的IMU预计分量构建因子图，优化当前帧的状态（包括位姿、速度、偏置）;
2.  以优化后的状态为基础，施加IMU预计分量，得到每一时刻的IMU里程计。

**订阅**

1.  订阅IMU原始数据，以因子图优化后的激光里程计为基础，施加两帧之间的IMU预计分量，预测每一时刻（IMU频率）的IMU里程计；
2.  订阅激光里程计（来自MapOptimization），用两帧之间的IMU预计分量构建因子图，优化当前帧位姿（这个位姿仅用于更新每时刻的IMU里程计，以及下一次因子图优化）。

**发布**

1.  发布imu里程计；

## (4) MapOptimization 因子图优化

**功能简介**

1.  scan-to-map匹配：提取当前激光帧特征点（角点、平面点），局部关键帧map的特征点，执行scan-to-map迭代优化，更新当前帧位姿；
2.  关键帧因子图优化：关键帧加入因子图，添加激光里程计因子、GPS因子、闭环因子，执行因子图优化，更新所有关键帧位姿；
3.  闭环检测：在历史关键帧中找距离相近，时间相隔较远的帧设为匹配帧，匹配帧周围提取局部关键帧map，同样执行scan-to-map匹配，得到位姿变换，构建闭环因子数据，加入因子图优化。

**订阅**

1.  订阅当前激光帧点云信息，来自FeatureExtraction；
2.  订阅GPS里程计；
3.  订阅来自外部闭环检测程序提供的闭环数据，本程序没有提供，这里实际没用上。

**发布**

1.  发布历史关键帧里程计；
2.  发布局部关键帧map的特征点云；
3.  发布激光里程计，rviz中表现为坐标轴；
4.  发布激光里程计；
5.  发布激光里程计路径，rviz中表现为载体的运行轨迹；
6.  发布地图保存服务；
7.  发布闭环匹配局部关键帧map；
8.  发布当前关键帧经过闭环优化后的位姿变换之后的特征点云；
9.  发布闭环边，rviz中表现为闭环帧之间的连线；
10.  发布局部map的降采样平面点集合；
11.  发布历史帧（累加的）的角点、平面点降采样集合；
12.  发布当前帧原始点云配准之后的点云；

下一篇将对ImageProjection（[LIO-SAM源码解析(一)：ImageProjection](https://zhuanlan.zhihu.com/p/352120054)）的代码内容进行详细介绍。

## 参考

-   [运行LIO-SAM踩坑问题\_zhaodeming000的博客-CSDN博客](https://link.zhihu.com/?target=https%3A//blog.csdn.net/zhaodeming000/article/details/111477923)
-   [LIO-SAM：配置环境、安装测试、适配自己采集数据集\_有待成长的小学生-CSDN博客](https://link.zhihu.com/?target=https%3A//blog.csdn.net/qq_42938987/article/details/108434290)
-   [LIO-SAM-scan2MapOptimization()优化原理及代码解析\_chennuo0125的博客-CSDN博客](https://link.zhihu.com/?target=https%3A//blog.csdn.net/weixin_37835423/article/details/111587379)
-   [运行LIO-SAM踩坑问题\_zhaodeming000的博客-CSDN博客](https://link.zhihu.com/?target=https%3A//blog.csdn.net/zhaodeming000/article/details/111477923)
-   [robot L：【论文阅读38】LIO-SAM](https://zhuanlan.zhihu.com/p/153394930?from_voters_page=true)