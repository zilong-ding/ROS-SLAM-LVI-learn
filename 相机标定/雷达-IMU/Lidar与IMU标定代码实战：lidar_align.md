> Date：2020-9-14  
> 作者：小L  
> 来源：公众号【[3D视觉工坊](https://link.zhihu.com/?target=https%3A//mp.weixin.qq.com/s/KoJwpSFkpDR5yTA88HhyzQ)】  
> 欢迎加入国内最大的[3D视觉交流社区](https://link.zhihu.com/?target=https%3A//mp.weixin.qq.com/s/KoJwpSFkpDR5yTA88HhyzQ)

**注1：文末附有【多传感器融合、SLAM】交流群加入方式哦~**

**注2：多传感器数据融合系统教程：**[自动驾驶中的多传感器融合](https://link.zhihu.com/?target=https%3A//mp.weixin.qq.com/s%3F__biz%3DMzU1MjY4MTA1MQ%3D%3D%26mid%3D2247562103%26idx%3D1%26sn%3D55fbd9538a6e9961dff1c3860705f114%26chksm%3Dfbfdd843cc8a5155b6c5b9239612a1c48266188f339f1b7399ef761819bdafd79433c505157e%26token%3D991507855%26lang%3Dzh_CN%23rd)

## **0\. 前言**

对于Lidar+IMU系统，往往需要标定Lidar与IMU的外参\[4\]，即从Lidar到IMU的6个位姿参数。ETH开源的lidar\_align代码\[0\]，用于标定Lidar和里程计Odom之间的位姿参数。本文将对代码进行初步介绍，并总结一些使用中可能会遇到的问题。

## **1\. 代码整体一览**

## **1.1 代码结构**

代码主要包括：头文件、cpp、以及ROS的launch启动文件等。其中头文件包括：**aligner.h**：Lidar和Odom对齐（外参计算）时用到的类**loader.h**：从ROS的Bag或CSV格式载入数据的相关函数**sensors.h**：主要包括：里程计Odom，以及雷达Lidar相关接口**transform.h**：一些SO3变化的计算以及转换，在插值、优化时使用

## **1.2 方法基本思想**

方法本质解决的是一个优化问题，即在外参参数(6DoF)如何选择时，使Lidar采集到的数据转化到Odom系下后，前后两次scan的数据点能够尽可能的重合。详细一点儿来说，方法将Lidar数据根据当前假设的状态变量（6DoF参数）变换到Odom系下，构成点云PointCloud，之后对每一次scan时的数据，在下一次scan中通过kdtree的方式寻找最近邻的数据点并计算距离，当总距离最小时可以认为完全匹配，即计算的外参参数正确。

## **1.3 主要流程**

代码主要有两部分：**载入数据**与**优化计算**。载入数据部分从Bag数据载入雷达的数据(sensor\_msgs/PointCloud2)，并从CSV或Bag数据中载入Odom获得的6DoF位置信息。具体的位置信息如何获得将在后面进行介绍。优化计算部分将在第2.2小节详细展开。

## **2\. 详细介绍**

## **2.1 主要数据类型**

**Odom数据**：主要包括两个数据：时间戳timestamp\_us\_与从当前时刻到初始时刻的变换T\_o0\_ot\_。**Lidar数据**：主要包括两个参数：从Lidar到Odom的外参T\_o\_l\_与每次扫描的数据scans\_，而每次的扫描scan数据类型主要包括：扫描起始时间timestamp\_us\_，本次扫描的点云raw\_points\_，某个点在Odom的变换（用于去畸变）T\_o0\_ot\_，以及相关配置参数等。**Aligner数据**：Aligner首先包含了需要优化的数据OptData（其中包括Lidar、Odom等数据），以及相应的配置参数（数据载入路径、初值、优化参数、KNN相关参数等），以及优化计算的相关参数。

## **2.2 优化过程详细介绍**

在载入了Odom和Lidar数据之后，进行优化求解6个位姿参数。主要求解函数为：lidarOdomTransform**Aligner::lidarOdomTransform()**首先进行相关的优化配置。默认优化参数是6个，但可以考虑两个传感器传输造成的时间差，如果考虑这个因素，参数数量将变为7。优化时，采用NLOPT优化库\[3\]，默认首先全局优化这三个参数。如果提供的初值与真值相差较大，或完全没有设置初值（默认为全0），则需要进行全局优化获得旋转参数。在局部优化这6个参数，局部优化开始时的初值就是3个为0的平移参数，以及全局优化计算出来的旋转参数。全局优化、局部优化，都是调用的optimize函数。**Aligner::optimize()**在这个函数设置了NLOPT优化的相关参数，包括：是全局优化还是局部优化、优化问题的上下界、最大迭代次数、求解精度以及目标函数等。最重要的是目标函数LidarOdomMinimizer**LidarOdomMinimizer()**这个函数在优化中会不断调用，迭代计算。首先会通过上一时刻的状态，计算新的从Lidar到Odom的变换（这里用到了Transform.h中定义的一些变换），误差是由lidarOdomKNNError函数获得。**lidarOdomKNNError()**这个是一个重载函数，具有两种重载类型。首先调用的是lidarOdomKNNError(const Lidar)，处理的是Lidar的数据，首先根据估计的Lidar到Odom的变化，对完整的scans\_数据计算出每次scan时每个点在Odom下的坐标（getTimeAlignedPointcloud函数，相当于点云去畸变），得到一个结合的点云(CombinedPointcloud)，之后从这个点云中寻找每个点的最近邻，在利用另一个重载类型的lidarOdomKNNError(const Pointcloud, const Pointcloud)函数进行计算最近邻误差。计算最近邻误差时，构建了一个KD-Tree，并行计算kNNError函数，利用pcl库的nearestKSearch函数搜索一定范围（全局优化时是1m，局部优化时是0.1m）的最近邻，计算最近2个点的误差。**小结**优化的目标函数是每次scan的每个点在完整点云中的最近邻的距离，首先通过粗的全局优化估计一部分参数，再局部优化求解精细的6DoF参数。

## **3\. 配置与运行**

## **3.1 安装**

首先在安装时需要安装NLOPT：sudo apt-get install libnlopt-dev。之后把代码拷贝到ros的工作空间，使用 catkin\_make进行编译。

## **3.2 编译可能遇到的问题**

这个代码是个人编写使用，没有在大多数的ubuntu和ros版本进行测试，所以可能会遇到各种各样的问题。以Ubuntu18与ROS-melodic为例，首先会遇到一个定义冲突的报错：**1\. 定义冲突问题**_error: conflicting declaration ‘typedef struct LZ4\_stream\_t LZ4\_stream\_t’ typedef struct { long long table\[LZ4\_STREAMSIZE\_U64\]; } LZ4\_stream\_t;_这个原因是ROS版本下有两个头文件定义发生冲突，github的issue中给出了两种解决办法，之一是重命名头文件避免冲突：sudo mv /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4.h.baksudo mv /usr/include/flann/ext/lz4hc.h /usr/include/flann/ext/lz4.h.baksudo ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.hsudo ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h（详见：[https://github.com/ethz-asl/lidar\_align/issues/16](https://link.zhihu.com/?target=https%3A//github.com/ethz-asl/lidar_align/issues/16)）**2\. 找不到"FindNLOPT.cmake"**_By not providing "FindNLOPT.cmake" in CMAKE\_MODULE\_PATH this project has asked CMake to find a package configuration file provided by "NLOPT", but CMake did not find one. Solutions: Move "NLOPTConfig.cmake" file to src directory._解决方法：将"\\lidar\_align\\FindNLOPT.cmake"文件移动到工作路径下中的"\\src"文件夹下，再次编译即可。

## **3.3 测试运行**

在github的issue中，由于存在准备数据（尤其是Odom数据）有错误的问题，造成运行失败。作者上传了测试数据[https://drive.google.com/open?id=11fUwbVnvej4NZ\_0Mntk7XJ2YrZ5Dk3Ub](https://link.zhihu.com/?target=https%3A//drive.google.com/open%3Fid%3D11fUwbVnvej4NZ_0Mntk7XJ2YrZ5Dk3Ub)可以运行测试。

## **3.4 数据准备**

Lidar的数据直接是ros中的sensor\_msgs/PointCloud2即可，但位置数据需要提供CSV格式或者ROS下的geometry\_msgs/TransformStamped消息类型。后者如何获得？如果是IMU直接进行积分就好，但这样积分势必会不准确，作者也在issue中提到没有考虑noise的问题([https://github.com/ethz-asl/lidar\_align/issues/5#issuecomment-432232087](https://link.zhihu.com/?target=https%3A//github.com/ethz-asl/lidar_align/issues/5%23issuecomment-432232087))，所以目前看来对IMU进行积分，凑合使用就好。\[1\]给出了一种IMU计算Odom的实现。

## **3.5 数据采集要求**

作者在issue和readme中指出，该方法存在的局限性是，必须要求采集数据时系统进行**非平面运动**，对平移要求不高但要求旋转必须充分。但对数据量、运动范围没有经过严格的测试。这个局限性也限制了不能用于给无人车这种系统标定。

## **参考资料**

\[0\]. 原版代码github：[https://github.com/ethz-asl/lidar\_align](https://link.zhihu.com/?target=https%3A//github.com/ethz-asl/lidar_align)

\[1\]. IMU数据计算Odom的实现：[https://www.cnblogs.com/gangyin/p/13366683.html](https://link.zhihu.com/?target=https%3A//www.cnblogs.com/gangyin/p/13366683.html)

\[2\]. lidarr\_align原理简要介绍：[https://blog.csdn.net/miracle629/article/details/87854450](https://link.zhihu.com/?target=https%3A//blog.csdn.net/miracle629/article/details/87854450)

\[3\]. NLOPT优化库介绍：[https://nlopt.readthedocs.io/en/latest](https://link.zhihu.com/?target=https%3A//nlopt.readthedocs.io/en/latest)

\[4\]. Lidar和IMU标定需要标什么？[https://blog.csdn.net/tfb760/article/details/108532974](https://link.zhihu.com/?target=https%3A//blog.csdn.net/tfb760/article/details/108532974)

**更多干货资源：**

[汇总 | 国内最全的3D视觉学习资源，涉及计算机视觉、SLAM、三维重建、点云处理、姿态估计、深度估计、3D检测、自动驾驶、深度学习（3D+2D）、图像处理、立体视觉、结构光等方向！](https://link.zhihu.com/?target=https%3A//mp.weixin.qq.com/s/xyGndcupuK1Zzmv1AJA5CQ)

[汇总 | 3D目标检测（基于点云、双目、单目）](https://link.zhihu.com/?target=https%3A//mp.weixin.qq.com/mp/appmsgalbum%3Faction%3Dgetalbum%26album_id%3D1433712471084466177%26__biz%3DMzU1MjY4MTA1MQ%3D%3D%23wechat_redirect)

[汇总 | 6D姿态估计算法（基于点云、单目、投票方式）](https://link.zhihu.com/?target=https%3A//mp.weixin.qq.com/mp/appmsgalbum%3Faction%3Dgetalbum%26album_id%3D1433707278687109122%26__biz%3DMzU1MjY4MTA1MQ%3D%3D%23wechat_redirect)

[汇总 | 三维重建算法实战（单目重建、立体视觉、多视图几何）](https://link.zhihu.com/?target=https%3A//mp.weixin.qq.com/mp/appmsgalbum%3Faction%3Dgetalbum%26album_id%3D1433700656199860224%26__biz%3DMzU1MjY4MTA1MQ%3D%3D%23wechat_redirect)

[汇总 | 3D点云后处理算法（匹配、检索、滤波、识别）](https://link.zhihu.com/?target=https%3A//mp.weixin.qq.com/mp/appmsgalbum%3Faction%3Dgetalbum%26album_id%3D1329868938683187201%26__biz%3DMzU1MjY4MTA1MQ%3D%3D%23wechat_redirect)

[汇总 | SLAM算法（视觉里程计、后端优化、回环检测）](https://link.zhihu.com/?target=https%3A//mp.weixin.qq.com/mp/appmsgalbum%3Faction%3Dgetalbum%26album_id%3D1329870989060308993%26__biz%3DMzU1MjY4MTA1MQ%3D%3D%23wechat_redirect)

[汇总 | 深度学习&自动驾驶前沿算法研究（检测、分割、多传感器融合）](https://link.zhihu.com/?target=https%3A//mp.weixin.qq.com/mp/appmsgalbum%3Faction%3Dgetalbum%26album_id%3D1329874254812512256%26__biz%3DMzU1MjY4MTA1MQ%3D%3D%23wechat_redirect)

[汇总 | 相机标定算法](https://link.zhihu.com/?target=https%3A//mp.weixin.qq.com/mp/appmsgalbum%3Faction%3Dgetalbum%26album_id%3D1319739406642937857%26__biz%3DMzU1MjY4MTA1MQ%3D%3D%23wechat_redirect)

[汇总 | 事件相机原理](https://link.zhihu.com/?target=https%3A//mp.weixin.qq.com/mp/appmsgalbum%3Faction%3Dgetalbum%26album_id%3D1329866645053210625%26__biz%3DMzU1MjY4MTA1MQ%3D%3D%23wechat_redirect)

[汇总 | 结构光经典算法](https://link.zhihu.com/?target=https%3A//mp.weixin.qq.com/mp/appmsgalbum%3Faction%3Dgetalbum%26album_id%3D1319744372111671296%26__biz%3DMzU1MjY4MTA1MQ%3D%3D%23wechat_redirect)

[汇总 | 缺陷检测常用算法与实战技巧](https://link.zhihu.com/?target=https%3A//mp.weixin.qq.com/mp/appmsgalbum%3Faction%3Dgetalbum%26album_id%3D1433687238369705986%26__biz%3DMzU1MjY4MTA1MQ%3D%3D%23wechat_redirect)

欢迎加入【3D视觉工坊】交流群，方向涉及3D视觉、计算机视觉、深度学习、vSLAM、激光SLAM、立体视觉、自动驾驶、点云处理、三维重建、多视图几何、结构光、多传感器融合、VR/AR、学术交流、求职交流等。工坊致力于干货输出，为3D领域贡献自己的力量！欢迎大家一起交流成长~

添加小助手微信：CV\_LAB，备注学校/公司+姓名+研究方向即可加入工坊一起学习进步。

编辑于 2021-05-16 · 著作权归作者所有