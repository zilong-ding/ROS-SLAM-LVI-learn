原论文信息

> @inproceedings{liosam2020shan,  
> title={LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping},  
> author={Shan, Tixiao and Englot, Brendan and Meyers, Drew and Wang, Wei and Ratti, Carlo and Rus Daniela},  
> booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},  
> pages={5135-5142},  
> year={2020},  
> organization={IEEE}  
> }

**【Paper】**[https://github.com/TixiaoShan/LIO-SAM/blob/master/config/doc/paper.pdf](https://link.zhihu.com/?target=https%3A//github.com/TixiaoShan/LIO-SAM/blob/master/config/doc/paper.pdf)

【**Code**】[TixiaoShan/LIO-SAM](https://link.zhihu.com/?target=https%3A//github.com/TixiaoShan/LIO-SAM)

**【Dataset】百度网盘：**[https://pan.baidu.com/s/1ezIFR25NRhCVmeCIDfi2Bg](https://link.zhihu.com/?target=https%3A//pan.baidu.com/s/1ezIFR25NRhCVmeCIDfi2Bg) 提取码：8ko1

## 0.摘要 Abstract

提出一种**紧耦合的平滑建图激光惯导里程计框架**，LIO-SAM，完成高准确度、实时的移动机器人轨迹估计和地图构建。LIO-SAM在因子图上制定了一个激光惯导里程计，允许许多相对和绝对测量，包括闭环，从不同的来源作为因子图的输入纳入系统中。I**MU预积分的运动估计对点云进行去畸变并产生激光里程计优化的初始估计**。**激光雷达里程计方案用于估计IMU的零偏。为了确保在实时环境下得到较好的性能，在姿态优化的时候边缘化掉旧的激光雷达帧**，而不是将激光雷达点云帧匹配到全局地图中。局部尺度而不是全局尺度上的扫描匹配可以显著高系统的实时性能，**选择性引入关键帧以及有效的滑动窗口方法，将新的关键帧注册到固定大小的“子关键帧”中**。提出的方法，在不同尺度和环境下，从三个平台搜集的数据集上，进行了广泛的评估。

## 1.引言 Introduction

状态估计、定位与建图是一个成功的智能移动机器人的基础前提，这是反馈控制、避障和规划等许多其他功能所必需的。利用基于视觉和基于激光雷达的感知，人们已经付出了巨大的努力致力于实现移动机器人的高性能实时同步定位与建图，它可以支持移动叫人的六自由度的状态估计。基于视觉的方法通常采用单目或者深度相机，并通过连续图像的三角化特征来确定相机运动。尽管基于视觉的方法特别适合位置识别，但是视觉方法对初始化、光照、距离的敏感性，使得它们在单独用于支持自动导航系统时不是很可靠。另一方面，基于激光雷达的方法在很大程度上不受光照变化的影响。特别是随着最近提出的远距离、高分辨率三维激光雷达，比如Velodyne128线以及Ouster128线，激光雷达变得更适合直接捕捉三维空间环境中的细节。因此，本文着重研究基于激光雷达的状态估计和建图方法。

最近二十年，许多基于激光雷达的状态估计和建图方法提出。其中，LOAM方法，在低偏移和实时状态估计和建图中，是应用最广泛的。LOAM采用激光雷达和IMU达到了最好的性能，并自从KITTI里程计Benchmark榜单发布以来一直被列为最好的激光雷达方法。尽管LOAM取得了成功，但还是存在一些限制：通过将数据保存在全局体素地图中，通常很难执行回环检测以及结合其他，比如GPS，绝对测量值用于姿态校正。**当体素地图在特征丰富的环境中变得稠密时，LOAM的在线优化过程就会变得效率很低**。**LOAM在大规模测试中也存在漂移问题，因为它的核心是一种基于scan-match扫描匹配的方法**。

本文中，提出一个通过平滑和建图的紧耦合激光惯导里程计框架，LIO-SAM，用来解决上面提到的问题。我们假设一个点云去畸变后的非线性运动模型，**使用原始IMU测量数据估计激光雷达扫描过程中的传感器运动**。**除了点云去畸变，运动估计作为激光雷达里程计优化的初始估计。得到的激光雷达里程计解决方案用于估计因子图中IMU零**偏。通过引入机器人轨迹估计的全局因子图，我们**利用激光雷达和IMU测量有效地进行传感器融合，结合机器人姿态之间的位置识别，并在可用的时候，引入GPS定位或者指南针航向等绝对测量**。不同来源因子的集合用于因子图的联合优化。此外，我们边缘化旧的激光雷达扫描来优化姿态，而不是类似LOAM那样将扫描匹配到全局地图中。局部尺度而不是全局尺度上的扫描匹配scan-match可以显著提高系统的实时性能，以及选择性地引入关键帧和一种有效的滑动窗口方法，将新的关键帧注册到之前的“子关键帧”集合中。

本文工作的主要贡献总结如下：

-   在因子图上建立一个紧耦合的激光惯导里程计框架，适用于多传感器融合和全局优化。
-   提出一种高效的，基于局部滑动窗口的扫描匹配方法，通过有选择性地挑选新关键帧注册到固定大小的先前的子关键帧集合上，来达到实时的性能。
-   提出的框架在不同尺度、车辆以及环境中的测试得到了广泛的验证。

## 2.相关工作 Related Work

激光雷达里程计通常是通过ICP【3】或者GICP【4】等scan-match扫描匹配算法查找两个连续帧之间的相对转换来进行的。除了匹配一整片点云，基于特征的匹配方法已经成为一种流行的替代方法。例如，文献【5】中提出的基于平面的实时激光雷达匹配方法。假设在结构化环境中进行操作，它从点云中提取平面，并通过解决最小二乘问题对它们进行匹配。文献【6】中提出一种基于项圈线的里程计估计方法。在该方法中，从原始点云中随机生成线段，并用于之后的点云配准。然而，由于现代三维激光雷达的旋转机制和传感器的运动，扫描的点云通常都是倾斜的。仅仅使用激光雷达进行姿态估计并不是理想的，因为使用点云或特征进行配准最终会导致大的漂移。

因此，激光雷达通常与其他传感器联合使用，比如IMU或者GPS，用于状态估计以及建图。这种利用传感器融合的设计方案，通常可以分为两类：松耦合融合以及紧耦合融合。\*\*在LOAM中，引入IMU用来激光雷达扫描数据的去畸变，并给出扫描匹配的运动先验信息。然而，IMU并没有涉及到算法的优化过程。因此LOAM可以归类为松耦合方法。文献【7】中提出一种用于地面车辆建图任务的轻量级地面优化激光雷达里程计与建图算法LeGO-LOAM。其对IMU数据的融合过程与LOAM中一致。EKF扩展卡尔曼滤波是另一种更加流行的松耦合融合方法。例如，文献【9】到文献【13】在优化阶段使用EKF结合激光雷达和IMU/GPS观测，来用于机器人状态估计。

紧耦合系统通常提供更高的精度，并且是目前正在研究的主要热点。文献【15】中，IMU预积分被用到点云去畸变中。在文献【16】中提出一种机器人激光惯导状态估计器RLINS。它以一种紧耦合的方式递归地采用错误状态卡尔曼滤波器去矫正机器人的状态估计。由于缺乏其他用于状态估计的传感器，它在长期导航过程中容易出现漂移。文献【17】中引入一种紧耦合的激光惯导里程计与建图框架，LIOM。其是LIO-Mapping的缩写，联合优化了激光雷达和IMU的测量结果，与LOAM相比获得了差不多甚至更好的精度。因为LIOM方法被设计用来处理所有的传感器测量值，因此不能达到实时性--在我们的测试中大约跑了六成的实时速度。

## 3.基于平滑和建图的激光惯导里程计 Lidar Inertial Odometry Via Smoothing and Mapping

### A. 系统总览 System Overview

首先我们定义整个论文中使用的坐标系与变量符号。我们将世界坐标系定义为$W$，机器人坐标系为$B$。为了方便起见，我们还假设\*\*IMU坐标系与机器人本体坐标系重合\*\*。机器人的状态$x$可以写为：

![[公式]](https://www.zhihu.com/equation?tex=x%3D%5BR%5E%7B%5Cintercal%7D%2Cp%5E%7B%5Cintercal%7D%2Cv%5E%7B%5Cintercal%7D%2Cb%5E%7B%5Cintercal%7D%5D%5E%7B%5Cintercal%7D)

其中 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+R+%5Cin+SO%283%29) 是定义在特殊正交群上的旋转矩阵， ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+p+%5Cin+%5Cmathbb+R%5E3) 是平移向量， ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+v) 是速度，$b$是IMU的零偏。变换矩阵 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+T+%5Cin+SE%283%29) 表示从机器人机体坐标系 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+B) 到世界坐标系 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+W) 的特殊欧式群，表示为 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+T+%3D+%5B%5Cmathbf+%7BR+%7C+p%7D%5D) 。

![](https://pic2.zhimg.com/v2-e2d83fdde8fe07827abfd947ca6c2515_b.jpg)

图1. LIO-SAM的系统结构。接收3D激光雷达、IMU以及GPS作为输入。引入四种因子去构建因子图。a)IMU预积分因子，b) 激光里程计因子，c) GPS因子，d) 闭环因子。这些因子如何产生描述在第3节中。

该系统从激光雷达、IMU以及可选的GPS中接收传感器数据。我们试图利用这些传感器的观测来估计机器人的状态及其轨迹。状态估计问题可以表示为一个最大后验MAP问题。我们使用因子图去建模这个问题，因为与贝叶斯网络相比，因子图更适合执行推理任务。在高斯噪声模型的假设下，我们问题的最大后验估计等价于求解一个非线性最小二乘问题。注意，如果不失一般性，我们提出的系统还可以包含来自其他传感器的测量，比如高度计的海拔高度以及指南针的航向等。我们介绍四种类型的因子factors，以及一种因子图构造的变量类型variable。该变量表示机器人状态在特定时间的状态归属于因子图中的节点nodes。

这四种因子分别是：

1.  **IMU预积分因子**
2.  **激光雷达里程计因子**
3.  **GPS因子**
4.  **闭环因子**

当机器人姿态的变化超过用户定义的阈值时，图中会添加一个新的机器人状态节点 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+x) 。在使用贝叶斯树（iSAM2）增量式平滑和建图过程插入一个新的节点时，因子图会随之进行优化。

### B. IMU预积分因子 IMU Preintegration Factor

IMU角速度和加速度的观测公式定义如下：

![[公式]](https://www.zhihu.com/equation?tex=%5Chat%7B%5Cmathbf++%5Comega%7D_t+%3D+%5Cmathbf++%5Comega_t+%2B+%5Cmathbf+++b_t%5E%7B%5Comega%7D+%2B+%5Cmathbf++n_t%5E%7B%5Comega%7D)

![[公式]](https://www.zhihu.com/equation?tex=%5Chat%7B%5Cmathbf+a%7D_%7Bt%7D++%3D+%5Cmathbf+R_%7Bt%7D%5E%7B%5Cmathbf+%7BBW%7D%7D%28+%5Cmathbf+a_t+-+%5Cmathbf+g%29+%2B+%5Cmathbf++b_t%5E%7B%5Cmathbf+a%7D+%2B+%5Cmathbf++n%5E%7B%5Cmathbf+a%7D_t)

其中 ![[公式]](https://www.zhihu.com/equation?tex=%5Chat%7B%5Comega%7D_t)和 ![[公式]](https://www.zhihu.com/equation?tex=%5Chat%7B%5Cmathbf%7Ba%7D%7D_t)是机体坐标系下 ![[公式]](https://www.zhihu.com/equation?tex=t) 时刻的IMU观测源数据。 ![[公式]](https://www.zhihu.com/equation?tex=%5Chat+%5Comega_t) 和 ![[公式]](https://www.zhihu.com/equation?tex=%5Chat%7B%5Cmathbf%7Ba%7D%7D_t) 均受到缓慢变化的IMU零偏 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+b_t) 以及白噪声 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+n_t) 的影响。 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+R_t%5E%7B%5Cmathbf%7BBW%7D%7D) 是从世界坐标系到机体坐标系的旋转矩阵。 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7Bg%7D) 是世界坐标系 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+W) 下的重力常数向量。

现在我们可以采用IMU测量去推断出机器人的运动。机器人在 ![[公式]](https://www.zhihu.com/equation?tex=t%2B%5CDelta+t) 时刻的速度、位置以及旋转计算如下：

![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+v_%7Bt%2B%7B%5CDelta+t%7D%7D+%3D+%5Cmathbf+v_t+%2B+%5Cmathbf+g+%5CDelta+t+%2B+%5Cmathbf+R_t%28%5Chat%7B%5Cmathbf+a+%7D_t+-+%5Cmathbf+b_t%5E%7B%5Cmathbf+a%7D+-+%5Cmathbf+n_t%5E%7B%5Cmathbf+a%7D+%29+%5CDelta+t)

![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+p_%7Bt%2B%5CDelta+t%7D+%3D+%5Cmathbf+p_t+%2B+%5Cmathbf+v+%5CDelta+t+%2B+%5Cfrac%7B1%7D%7B2%7D+%5Cmathbf+g+%5CDelta+t%5E2+%2B+%5Cfrac%7B1%7D%7B2%7D+%5Cmathbf+R_t%28%5Chat%7B%5Cmathbf+a%7D_t+-+%5Cmathbf+b_t%5E%7B%5Cmathbf+a%7D+-+%5Cmathbf+n_t%5E%7B%5Cmathbf+a%7D%29+%5CDelta+t%5E2+)

![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+R_%7Bt%2B%5CDelta+t%7D+%3D+%5Cmathbf+R_t+%5Cmathbf%7Bexp%7D%28%28%5Chat+%5Comega_t+-+%5Cmathbf+b_t%5E%7B%5Cmathbf+%5Comega+%7D+-+%5Cmathbf+n_t%5E%7B%5Cmathbf+%5Comega%7D%29+%5CDelta+t+%29)

其中 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+R_t+%3D+%5Cmathbf+R_t%5E%7B%5Cmathbf%7BWB%7D%7D%3D%7B%5Cmathbf+%7BR_t%5E%7B%5Cmathbf%7BBW%7D%7D%7D%7D%5E%5Cintercal) 。这里我们假设机器人角速度和加速度在积分过程中保持匀速不变。

然后我们采用文献【20】**提出的IMU预积分去获取两个时间戳之间的相对运动**。预积分得到的第 ![[公式]](https://www.zhihu.com/equation?tex=i) 和第 ![[公式]](https://www.zhihu.com/equation?tex=j) 时刻之间的速度测量 ![[公式]](https://www.zhihu.com/equation?tex=%5CDelta+%5Cmathbf+v_%7Bij%7D)、位置测量 ![[公式]](https://www.zhihu.com/equation?tex=%5CDelta+%5Cmathbf+p_%7Bij%7D)以及旋转测量 ![[公式]](https://www.zhihu.com/equation?tex=%5CDelta+%5Cmathbf+R_%7Bij%7D)计算如下：

![[公式]](https://www.zhihu.com/equation?tex=%5CDelta+%5Cmathbf+v_%7Bij%7D+%3D+%5Cmathbf+R_i%5E%5Cintercal%28%5Cmathbf+v_j+-+%5Cmathbf+v_i+-+%5Cmathbf+g+%5CDelta+t_%7Bij%7D%29+)

![[公式]](https://www.zhihu.com/equation?tex=%5CDelta+%5Cmathbf+p_%7Bij%7D+%3D+%5Cmathbf+R_i%5E%5Cintercal+%28%5Cmathbf+p_j+-+%5Cmathbf+p_i+-+%5Cmathbf+v_i+%5CDelta+t_%7Bij%7D+-+%5Cfrac%7B1%7D%7B2%7D+%5Cmathbf%7Bg%7D+%5CDelta+t_%7Bij%7D%5E2%29)

![[公式]](https://www.zhihu.com/equation?tex=%5CDelta+%5Cmathbf+R_%7Bij%7D+%3D+%5Cmathbf+R_i%5E%7B%5Cintercal%7D+%5Cmathbf+R_j)

由于篇幅限制，我们让读者参考来自文献【20】的描述，来得到公式【2-9】的详细推导。**除了IMU预积分的效率之外，应用IMU预积分也可以很自然地给出IMU预积分因子在因子图中的约束条件。IMU零偏因子与激光雷达里程计因子在因子图中一起进行联合优化。**

### C.激光雷达里程计因子 Lidar Odometry Factor

当新的一圈激光雷达扫描到来时，**首先执行特征提取**。**通过评估局部区域上点的粗糙度/曲率来提取边缘和平面特征。粗糙度/曲率较大的点被划分为边缘特征。类似地，一个平面特征也可以按一个小的粗糙度/曲率进行分类**。 我们将从激光雷达在 ![[公式]](https://www.zhihu.com/equation?tex=t) 时刻扫描点云中提取的边缘特征以及平面特征分别定义为 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+F_i%5E%7Be%7D) 以及 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+F_i%5Ep) 。在 ![[公式]](https://www.zhihu.com/equation?tex=t) 时刻提取的所有特征构成了一个激光雷达点云帧 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbb+F_i) ，其中 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbb+F_i%3D%5Clbrace%5Cmathbf%7BF%7D_i%5E%7Be%7D%2C%5Cmathbf%7BF%7D_i%5Ep%5Crbrace) 。注意一个激光雷达点云帧 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbb+F) 是表示在机器人坐标系下的。特征提取步骤的更多详细描述可以在文献【1】或如果用到距离图像也可以参考文献【7】。

**如果将每一帧激光雷达点云都拿来计算因子并添加到因子图中是难以计算的**，因此我们采用了**关键帧**选择 的概念，其被广泛应用于视觉SLAM领域。使用一种简单但有效的启发式方法，**当机器人姿态的变化与先前的状态 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+x_i)相比，如果超过了用户定义的阈值时，我们选择此时的激光雷达帧 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbb+R_%7Bi%2B1%7D)作为关键帧。新保存的关键帧 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbb+F_%7Bi%2B1%7D) 与因子图的中新的机器人状态节点 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+x_%7Bi%2B1%7D)相关联。然后丢弃掉两个关键帧之间的其他激光雷达帧**。这样添加的关键帧不仅可以实现地图密度和内存消耗之间的平衡，而且有助于保持一个相对稀疏的因子图结构，适用于实时的非线性优化。在我们的工作中，添加新关键帧的平移和旋转的变化阈值选择为1米和10度。

假设我们希望在因子图中添加一个新的状态节点 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+x_%7Bi%2B1%7D) 。那么与该状态相关联的激光雷达关键帧为 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbb+F_%7Bi%2B1%7D) 。激光雷达里程计因子的生成过程描述如下：

-   **1）建立子关键帧的体素地图 sub-keyframes for voxel map**

我们实现了一种**滑动窗口方法**去构建点云地图，其中包含了固定数目的最近激光雷达的扫描点云。我们没有优化两个连续的激光雷达扫描点云帧之间的变换关系，而是提取了n个最近的关键帧，**sub-keyframes子关键帧**，来进行估计。**采用变换关系 ![[公式]](https://www.zhihu.com/equation?tex=%5Clbrace+%5Cmathbb+T_%7Bi-n%7D%2C+%5Ccdots+%2C%5Cmathbb+T_i%5Crbrace) 将关联的子关键帧集合 ![[公式]](https://www.zhihu.com/equation?tex=%5Clbrace+%5Cmathbb+F_%7Bi-n%7D%2C+%5Ccdots+%2C%5Cmathbb+F_i%5Crbrace) 转换到世界坐标系中。转换后的子关键帧会被合并成一个体素地图 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+M_%7Bi%7D)** 。由于我们在之前的特征提取步骤中提取了两种特征：边缘特征以及平面特征，因此体素地图 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+M_%7Bi%7D)由两个子体素地图构成，定义边缘特征体素地图为 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+M_i%5Ee) ，平面特征体素地图为 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+M_%7Bi%7D%5E%7Bp%7D)。激光雷达帧以及体素地图之间的关系如下：

![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+M_i+%3D+%5Clbrace+%5Cmathbf+M_i%5Ee%2C+%5Cmathbf+M_i%5Ep+%5Crbrace)

其中：

![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+M_i%5Ee+%3D+%5Cmathbf%7B%5Chat+F%7D_i%5Ee+%5Ccup+%5Cmathbf%7B%5Chat+F%7D%5Ee_%7Bi%2B1%7D+%5Ccup+%5Ccdots+%5Ccup+%5Cmathbf%7B%5Chat+F%7D_%7Bi-n%7D%5Ee)

![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+M_i%5Ep+%3D+%5Cmathbf%7B%5Chat+F%7D_i%5Ep+%5Ccup+%5Cmathbf%7B%5Chat+F%7D%5Ep_%7Bi%2B1%7D+%5Ccup+%5Ccdots+%5Ccup+%5Cmathbf%7B%5Chat+F%7D_%7Bi-n%7D%5Ep)

而 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7B%5Chat+F%7D_%7Bi%7D%5E%7Be%7D) 和 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7B%5Chat+F%7D_%7Bi%7D%5Ep) 是转换到世界坐标系下的边缘特征和平面特征。然后对子体素地图 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+M_i%5Ee+) 和 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+M_%7Bi%7D%5Ep) 进行下采样，消除落在同一个体素单元中的重复冗余特征。本文中， ![[公式]](https://www.zhihu.com/equation?tex=n) 选择25。![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+M_i%5Ee+)和 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+M_%7Bi%7D%5Ep) 下采样的分辨率分别设为0.2m和0.4m。

-   **2）扫描匹配 scan-matching**

我们将新获得的激光雷达关键帧 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbb+F_%7Bi%2B1%7D+%3D+%5Clbrace+%5Cmathbf+F_%7Bi%2B1%7D%5Ee%2C+%5Cmathbf+F_%7Bi%2B1%7D%5Ep+%5Crbrace) 通过扫描匹配到世界坐标系下的体素地图 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+M_%7Bi%7D) 中。为此，可以使用各种扫描匹配scan-match方法，比如ICP【3】和GICP【4】。这里，我们选择使用文献【1】中提出的方法，因为它具备出色计算效率和在各种有挑战性的环境中有很好的鲁棒性。

我们首先将激光雷达关键帧 ![[公式]](https://www.zhihu.com/equation?tex=%5Clbrace+%5Cmathbf+F_%7Bi%2B1%7D%5Ee%2C+%5Cmathbf+F%5Ep_%7Bi%2B1%7D+%5Crbrace) 从机器人坐标系转换到世界坐标系中，得到转换后的关键帧集合为 ![[公式]](https://www.zhihu.com/equation?tex=%5Clbrace+%5Cmathbf%7B%5Chat%7BF%7D%7D_%7Bi%2B1%7D%5E%7Be%7D%2C+%5Cmathbf%7B%5Chat+F%7D_%7Bi%2B1%7D%5E%7Bp%7D+%5Crbrace) 。初始的变换关系是通过从IMU预测的机器人运动 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7B%5Coverline%7BT%7D%7D_%7Bi%2B1%7D) 中得到的。对于每帧 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7B%5Chat+F%7D_%7Bi%2B1%7D%5E%7Be%7D) 或者 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7B%5Chat+%7BF%7D%7D_%7Bi%2B1%7D%5E%7Bp%7D) 中的边缘特征和平面特征，我们然后找到它们对应的子体素地图 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+M_i%5Ee) 和 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+M_%7Bi%7D%5E%7Bp%7D) 中的边缘和平面。为了简洁起见，找到对应关系的详细过程在这里省略，文献【1】中给出了详细的描述。

-   **3）相对变换关系 relative transformation**

**边缘特征与平面特征和它们对应的边缘或平面块之间的距离**可以通过如下的公式进行计算：

![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+d_%7Be_%7Bk%7D%7D+%3D+%5Cfrac%7B%7C%28%5Cmathbf%7Bp%7D_%7Bi%2B1%2Ck%7D%5Ee-%5Cmathbf%7Bp%7D_%7Bi%2Cu%7D%5Ee%29+%5Ctimes+%28%5Cmathbf%7Bp%7D_%7Bi%2B1%2Ck%7D%5Ee-%5Cmathbf%7Bp%7D_%7Bi%2Cv%7D%5E%7Be%7D%29%7C%7D%7B%7C%5Cmathbf%7Bp%7D_%7Bi%2Cu%7D%5E%7Be%7D+-+%5Cmathbf%7Bp%7D_%7Bi%2Cv%7D%5E%7Be%7D%7C%7D)

![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+d_%7Bp_%7Bk%7D%7D+%3D+%5Cfrac%7B%5Cbegin%7Bvmatrix%7D%28%5Cmathbf%7Bp%7D_%7Bi%2B1%2Ck%7D%5E%7Bp%7D+-+%5Cmathbf%7Bp%7D_%7Bi%2Cu%7D%5Ep%29++%5C%5C++%28%5Cmathbf%7Bp%7D_%7Bi%2Cu%7D%5E%7Bp%7D+-+%5Cmathbf%7Bp%7D_%7Bi%2Cv%7D%5E%7Bp%7D%29+%5Ctimes++%28%5Cmathbf%7Bp%7D_%7Bi%2Cu%7D%5E%7Bp%7D+-+%5Cmathbf%7Bp%7D_%7Bi%2Cw%7D%5E%7Bp%7D%29%5C%5C+%5Cend%7Bvmatrix%7D%7D%7B%7C%28%5Cmathbf%7Bp%7D_%7Bi%2Cu%7D%5E%7Bp%7D+-+%5Cmathbf%7Bp%7D_%7Bi%2Cv%7D%5E%7Bp%7D%29+%5Ctimes++%28%5Cmathbf%7Bp%7D_%7Bi%2Cu%7D%5E%7Bp%7D+-+%5Cmathbf%7Bp%7D_%7Bi%2Cw%7D%5Ep%29%7C%7D)

其中， ![[公式]](https://www.zhihu.com/equation?tex=k%2Cu%2Cv%2Cw) 是在边缘特征或平面特征对应集合中的索引号。对于在转换后的边缘关键帧 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7B%5Chat+F%7D_%7Bi%2B1%7D%5E%7Be%7D) 中的边缘特征 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+p%5Ee_%7Bi%2B1%2Ck%7D) 来说， ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+p%5E%7Be%7D_%7Bi%2Cu%7D) 和 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+p_%7Bi%2Cv%7D%5E%7Be%7D) 是形成边缘子体素地图 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+M_%7Bi%7D%5Ee) 中相对应的边缘线上的点。而对于在转换后的平面关键帧 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7B%5Chat+F%7D_%7Bi%2B1%7D%5E%7Bp%7D)中的平面特征 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+p_%7Bi%2B1%2Ck%7D%5E%7Bp%7D)来说， ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7Bp%7D_%7Bi%2Cu%7D%5Ep) ， ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7Bp%7D_%7Bi%2Cv%7D%5E%7Bp%7D) 以及 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7Bp%7D_%7Bi%2Cw%7D%5Ep) 形成了平面子体素地图 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+M_%7Bi%7D%5Ep) 中对应的平面块。然后采用高斯牛顿法来求解最优的变换矩阵关系，通过最小化下述公式：

![[公式]](https://www.zhihu.com/equation?tex=%5Cmin_%7B%5Cmathbf+T_%7Bi%2B1%7D%7D+%5Clbrace+%5Csum_%7B%5Cmathbf+p_%7Bi%2B1%7D%5Ee+%5Cin+%5Cmathbf%7B%5Chat+F%7D_%7Bi%2B1%7D%5Ee%7D+%5Cmathbf+d_%7Be_k%7D+%2B+%5Csum_%7B%5Cmathbf+p_%7Bi%2B1%7D%5E%7Bp%7D+%5Cin+%5Cmathbf%7B%5Chat+F%7D_%7Bi%2B1%7D%5Ep%7D+%5Cmathbf+d_%7B%7Bp%7D_k%7D+%5Crbrace)

最后，我们可以得到状态节点 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+x_i) 与状态节点 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+x_%7Bi%2B1%7D) 之间的相对变换关系 ![[公式]](https://www.zhihu.com/equation?tex=%5CDelta+%5Cmathbf%7BT%7D_%7Bi%2Ci%2B1%7D) ，它是连接这两种位姿的激光里程计因子：

![[公式]](https://www.zhihu.com/equation?tex=%5CDelta+%5Cmathbf%7BT%7D_%7Bi%2Ci%2B1%7D+%3D+%5Cmathbf%7BT%7D_%7Bi%7D%5E%7B%5Cintercal%7D+%5Cmathbf%7BT%7D_%7Bi%2B1%7D)

我们注意到，获得 ![[公式]](https://www.zhihu.com/equation?tex=%5CDelta+%5Cmathbf%7BT%7D_%7Bi%2Ci%2B1%7D) 的另一种方法是将子关键帧转换到 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+x_i) 状态节点的坐标系下。换句话说，就是将 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbb+F_%7Bi%2B1%7D) 与在 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+x_i) 状态节点坐标系下表示的体素地图进行匹配。这种方式下，可以直接得到实施相对变换关系 ![[公式]](https://www.zhihu.com/equation?tex=%5CDelta+%5Cmathbf%7BT%7D_%7Bi%2Ci%2B1%7D) 。由于转换后的特征 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7B%5Chat+F%7D_i%5E%7Be%7D) 和 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7B%5Chat%7BF%7D%7D_i%5E%7Bp%7D) 可以多次重复使用，所以我们选择采用第III节-C.1中描述的方法，因为它的计算效率很高。

### **D. GPS 因子 GPS Factor**

**虽然仅仅利用IMU预积分和激光里程计因子就可以获得可靠的状态估计以及建图，但是这个系统在长期的导航任务中，仍然存在漂移问题**。为了解决这个问题，我们可以**引入提供消除漂移的绝对测量值的传感器**。这些传感器包括高度计、指南针罗盘以及GPS全球定位系统。这里为了阐述清楚，我们讨论了其中GPS，因为它在现实导航系统中被广泛使用。

当我们接收GPS测量时，首先使用文献【21】中提到的方法**将GPS坐标转换到局部笛卡尔坐标系下**。在因子图中添加一个新的节点后，我们将新的GPS因子和这个新节点关联起来。**如果GPS信号与激光雷达没有做硬件同步，那么我们根据激光雷达的时间戳在GPS中进行线性插值。**

我们注意到，当GPS接收可用时，不断添加GPS因子是没有必要的，因为激光惯导里程计漂移增加是非常缓慢的。在实践中，我们只需要在估计的位置协方差大于接收到的GPS位置协方差的时候，才去增加一个GPS因子。

### E. 闭环因子

由于利用了因子图，闭环也可以无缝地插入到提出的系统中，而不是跟LOAM和LIO Mapping那样。为了说明，我们描述并实现了一种**朴素但有效的基于欧氏距离的闭环检测方法**。我们还注意到，我们提出的框架与其他的闭环检测方法是兼容的，例如文献【22】【23】提到的方法，他们生产了一个点云描述符并将它用于位置识别。

当一个新的状态 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+x_%7Bi%2B1%7D) 被加入到因子图中，我们首先在因子图中进行搜索，找到欧式空间中接近状态 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+x_%7Bi%2B1%7D) 的先验状态。如图1所示，状态节点 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+x_3) 是其中一个返回的候选对象。然后我们尝试用扫描匹配scan-match方式将关键帧 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbb+F_%7Bi%2B1%7D) 与子关键帧集 ![[公式]](https://www.zhihu.com/equation?tex=%5Clbrace+%5Cmathbb%7BF%7D_%7B3-m%7D%2C+%5Ccdots%2C+%5Cmathbb%7BF%7D_3%2C%5Ccdots%2C+%5Cmathbb%7BF%7D_%7B3%2Bm%7D+%5Crbrace) 进行匹配。注意，关键帧 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbb+F_%7Bi%2B1%7D) 和过去的子关键帧在扫描匹配scan-match之前已经先被转换到世界坐标系W中 。然后我们就得到了相对变换 ![[公式]](https://www.zhihu.com/equation?tex=%5CDelta+%5Cmathbf%7BT%7D_%7B3%2C+i%2B1%7D) ，并将其作为一个闭环因子添加到因子图中。在本文中，我们选择索引数m为12，闭环与新状态 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf+x_%7Bi%2B1%7D) 的搜索范围设为15m。

实际中，我们发现，当GPS是唯一可用的绝对位置测量传感器的时候，添加闭环因子对于修正机器人高程上的漂移特别有用。这是由于GPS的高程测量是非常不准确的，在我们的测试中，没有闭环情况下，会导致接近100米的高程误差。

## 4.实验 Experiments

我们现在描述一系列的实验来定性和定量地分析我们提出的框架。本文采用的传感器套件包括**一个Velodyne 16线激光雷达，一个MicroStrain 3DM-GX5-25的IMU以及一个Reach M的GPS**。为了进行实验验证，我们在不同尺度、平台和环境中搜集了5个不同的数据集。这些数据集分别为**Rotation，Walking，Campus，Park，Amsterdam**。传感器安装平台如图2所示。

![](https://pic4.zhimg.com/v2-03c53371888b2627486ee7aa99f1c893_b.jpg)

图2. 在三个平台上搜集的数据集：a) 一个定制的手持式设备；b) 一辆无人驾驶的地面汽车-ClearPath Jackal；c) 一艘电动船-Duffy

前三个数据集是使用MIT校园里定制的手持式设备搜集的。Park数据集是在一个植被覆盖的公园里采集的，使用了一辆无人地面车辆（UGV）- Clearpath Jackal。最后一个数据集，Amsterdam，是通过将传感器安装在一艘电动船上，并在阿姆斯特丹运河中巡航采集的。这些数据集的详细信息显示在下表中。

数据集

扫描数

高程变化(米)

轨迹长度(米)

最大旋转速度(度/秒)

Rotation

582

0

0

213.9

Walking

6502

0.3

801

133.7

Campus

9865

1.0

1437

124.8

Park

24691

19.0

2898

217.4

Amsterdam

107656

0

19065

17.2

将我们提出的LIO-SAM框架与LOAM和LIO Mapping进行比较。在所有实验中，LOAM和LIO-SAM实时的运行了。而LIO-Mapping相反，给了无限多的时间来处理每个传感器的观测值。所有的方法都在C++中实现，并在配备了Intel i7-10710U CPU的笔记本上运行，使用的平台是是Ubuntu中的ROS机器人操作系统。我们注意到，只有CPU适用于计算，没有启用并行计算。我们的LIO-SAM的实现可以在我们的[Github](https://link.zhihu.com/?target=https%3A//github.com/TixiaoShan/LIO-SAM)中进行下载。执行实验的补充细节，包括所有测试的完整视频，可以在我们的[YouTube频道](https://link.zhihu.com/?target=https%3A//youtu.be/A0H8CoORZJU)中找到

### **A. Rotation数据集**

在这个测试中，当只使用IMU预积分以及激光里程计因子被添加到因子图中时，重点评估我们提出的LIO-SAM框架的鲁棒性。Rotation数据集是人手持传感器套件采集的，在原地静止不移动条件下，只进行纯旋转操作。本实验中最大的旋转速度为133.7度每秒。测试环境如图3(a)所示。LOAM和LIO-SAM中生成的地图如图3(b)，图3(c)所示。

![](https://pic3.zhimg.com/v2-19604f698161e2b6c83679a284c99b7e_b.jpg)

图3. LOAM和LIO-SAM在Rotation纯旋转数据集上的建图结果。LIO-Mapping没有建出好结果。

由于LIO-Mapping使用了文献【25】中相同的初始化过程，继承了视觉惯导SLAM类似的对初始化的敏感性，并且无法使用这个Rotation数据完成正确的初始化。由于LIO-Maping没能产生有意义的结果，它的地图就没有显示出来。从图中可以看出，与LOAM算法相比，LIO-SAM构建的地图保留了更精细的环境结构细节。这是由于LIO-SAM能够在SO(3)特殊正交群中精确的注册每个激光雷达帧点云，即使当机器人发生了快速旋转的情况下。

### B. Walking数据集

这个测试设计的目的在于评估我们提出的方法在SE(3)特殊欧式群中系统发生剧烈平移和旋转的情况下的性能。这个数据集上遇到的最大平移速度和旋转速度分别为1.8米每秒和213.9度每秒。在数据采集期间，人拿着图2(a)那样的手持式传感器，然后快速行走在MIT校园中，如图4(a)中。这个实验中，LOAM建的地图，如图4(b)所示，当遇到主动旋转时候，会在多个位置出现发散。LIO-Mapping在这个数据集上要比LOAM表现的出色。然而LIO-Mapping建的地图，如图4(c)所示，在不同位置仍然存在不一样，并且由很多模糊的结构组成。由于LIO-Mapping被设计用来处理所有传感器的测量，因此它旨在0.56倍的实时条件下运行，而其他两种方法都可以实时运行。最后，LIO-SAM的性能要优于其他两种方法，并构建了一张与可用的谷歌地图一致的地图。

![](https://pic1.zhimg.com/v2-1bbbe462775b8f14946412b49508eed8_b.jpg)

4\. LOAM, LIO-Mapping和LIO-SAM在Walking数据集上的建图结果。当遇到主动旋转时，图b的LOAM建的图会多次发散。LIO-Mapping建图性能要优于LOAM。但是，LIO-Mapping建的地图由于点云配准的不准确，造成了许多模糊结构。LIO-SAM在未使用GPS的情况下，生成了一张与谷歌地球图像一致的地图

### C. Campus数据集

本数据集上的测试是设计用来展示引入GPS和闭环因子后的建图效果。为了做到这一点，我们故意未添加GPS和闭环因子到因子图中。当GPS和闭环因子禁用后，我们提出的方法命名为LIO-Odom，也就是只利用IMU预积分和激光里程计因子。当使用GPS因子时，我们提出的方法定义为LIO-GPS，其使用IMU预积分，激光里程计以及GPS因子用于建图。LIO-SAM则是在图中添加了所有的因子。为了收集这个数据集，研究人员使用手持式传感器设备在MIT校园周围走动，并返回到相同位置。由于在建图区域存在许多建筑物和树木，GPS接收几乎不可用，而且大多数时候也不准确。在过滤掉不一致的GPS观测后，可用的GPS如图5(a)绿色部分所示。这些区域对应于少数没有建筑物或树木包围的区域。

![](https://pic1.zhimg.com/v2-3086ffc59ac37a1fc6d298b75f7d7690_b.jpg)

图5. 各种方法在MIT校园中采集的Campus数据集上的结果。红点表示起始位置和结束位置。轨迹的方向是按照时钟排列的。LIO-Mapping方法由于没有产生有意义的轨迹结果，没有显示在里面

LOAM、LIO-Odom、LIO-GPS以及LIO-SAM估计出来的轨迹如图5(a)所示。由于LIO-Mapping没能正确地初始化，并且未能产生有意义的结果，因此LIO-Mapping的结果没有展示出来。如图上所示，和其他其他方法相比，LOAM的轨迹有明显的漂移。由于没有对GPS数据进行修正，在地图的右下角LIO-Odom的轨迹开始出现明显的漂移。加入了GPS数据后，LIO-GPS可以在GPS可用时修正漂移。但是，在数据集的后半部分GPS都是不可用的。因此，当由于漂移导致机器人返回到起始位置时，LIO-GPS无法检测到闭环。另一方面，LIO-SAM可以通过向因子图中添加闭环因子来消除轨迹漂移。LIO-SAM建的地图与谷歌地球上的图可以很好的对齐，如图5(b)所示。当机器人回到原点时，所有方法的相对平移误差如下表所示。

数据集

LOAM

LIOM

LIO-Odom

LIO-GPS

LIO-SAM

Campus

192.43

Fail

9.44

6.87

0.12

Park

121.74

34.60

36.36

2.93

0.04

Amsterdam

Fail

Fail

Fail

1.21

0.17

### D. Park 数据集

在本次测试中，我们将传感器安装在一辆UGV无人车上，沿着森林徒步路线行驶。机器人在行驶40分钟后回到出发位置。无人车在三种路面上行驶：沥青路面、草坪以及泥地。由于缺乏悬挂系统，这个机器人在非沥青路面行驶过程中，会出现低幅度高频率的振动。

为了模拟具有挑战性的建图场景，我们只在机器人处于开放的区域时使用GPS测量值，比如图6(a)中的绿色部分。这种建图场景一类任务的代表，即机器人必须在没有GPS的区域进行建图，并定期返回有GPS信号的区域来纠正漂移。

![](https://pic3.zhimg.com/v2-d17ff96fe5eb885c3069f402490906d6_b.jpg)

图6. 使用美国新泽西州的Pleasant Valley Park收集Park数据集上所有方法的轨迹对比结果。红点表示起始位置和结束位置。轨迹方向是顺时针方向。

与前面实验结果类似，LOAM、LIO-Mapping和LIO-Odom都出现了很明显的轨迹漂移，因为他们没有使用绝对校正数据。此外，LIO-Mapping只在0.67倍实时速度下运行，而其他方法是实时运行的。虽然LIO-GPS和LIO-SAM的轨迹在水平面保持一致，但它们的相对平移误差是不一样的如表II所示。由于没有可靠的绝对高度测量值，LIO-GPS在高度上有漂移，并且在返回到机器人的起始位置时无法闭环。LIO-SAM没有出现这样的问题，因为它利用闭环因子来消除轨迹漂移。

### E. Amsterdam 数据集

最后，我们把传感器套件安装到电动船上，沿着阿姆斯特丹的运河巡航3小时。虽然在这次测试中，传感器的移动相对平稳，但由于存在若干个原因，给运河整体建图仍然具有挑战性。运河上的许多桥梁都出现了退化的情况，因为当船出现在桥下时，几乎没有什么有用的特征，类似于穿过一条很长又毫无特色的走廊。由于水里不存在地面，平面特征的数量也明显较少。当阳光直射到传感器的视野范围内时，我们观察到许多激光雷达的错误检测，这发生在数据采集总时长的20%处。由于电动船上方桥梁和城市建筑的存在，我们也只获得了间歇性的GPS信号的接收。

由于存在这样那样的挑战，LOAM、LIO-Mapping和LIO-Odom都未能在这个测试中产生有意义的结果。与Park公园数据集中遇到的问题类似，LIO-GPS由于高度值上出现漂移，在返回机器人的初始位置时无法闭环，这进一步激发我们在LIO-SAM中使用闭环因子来消除轨迹上的漂移。

### F. 基准测试结果 Benchmarking Results

由于完整的GPS覆盖只在Park数据集中可用，因此我们显示了关于GPS测量历史的均方根误差(RMSE)结果，可以把它看作Ground Truth真值。这里均方根误差没有考虑沿z轴发生的误差。如下表所示，LIO-GPS和LIO-SAM在GPS的Ground Truth真值上都达到了差不多的RMSE均方根误差。需要注意的是，我们可以通过让这两种方法使用所有GPS测量值，来使其均RMSE方根误差减少至少一个数量级。然而，在许多建图配置中，完整GPS观测并不总是可用的。我们的目的是设计一个可以在各种具有挑战性的环境中都可运行的鲁棒系统。

数据集

LOAM

LIOM

LIO-Odom

LIO-GPS

LIO-SAM

Park

47.31

28.96

23.96

1.09

0.96

LOAM、LIO-Mapping以及LIO-SAM这三种对比方法，在上述5个数据集上处理一个激光雷达点云帧的平均运行时间如下表所示。在所有测试中，LOAM和LIO-SAM是实时运行的。换句话说，当激光雷达的旋转速率为10Hz时，如果运行时间需要超过100ms，那么一些激光雷达帧点云就会被丢弃。我们给LIO-Mapping方法无限长的时间来处理每个激光雷达帧。如图中所示，LIO-SAM的运行时间明显少于其他两种方法，这使得它更适合部署在低功耗的嵌入式系统上。

数据集

LOAM

LIOM

LIO-SAM

Stress Test

Rotation

83.6

Fail

41.9

13x

Walking

253.6

339.8

58.4

13x

Campus

244.9

Fail

97.8

10x

Park

266.4

245.2

100.5

9x

Amsterdam

Fail

Fail

79.3

11x

我们还通过比实时速度更快的输入数据到LIO-SAM中，对其进行压力测试。当LIO-SAM在加速播放与1倍速回放数据的结果进行比较，可以顺利达到相似的性能时，记录下最大数据回放速度并显示在上表的最后一列中。从图中可以看出，LIO-SAM能够以高达13倍速来处理数据。

我们注意到，LIO-SAM的运行时间明显会受到特征地图密度的影响，但受到因子图中节点数和因子数的影响比较小。例如，Park数据集是在一个特征丰富的环境中采集的，其中的绿植产生了大量的特征，而Amsterdam数据集产生的特征地图相对更稀疏。Park数据集实验中的因子图包含4573个节点和9365个因子，而Amsterdam数据集实验中的因子图则有23304个节点和49617个因子。尽管如此，与Park数据集测试相比，LIO-SAM在Amsterdam数据集测试中使用的时间更少。

## 5.结论与讨论 Conclusion and Discussion

我们提出了**LIO-SAM，一个通过平滑和建图实现紧耦合激光雷达惯性里程计框架，用于复杂环境中执行实时状态估计和建图**。**通过在因子图上建立激光雷达-惯性里程计，LIO-SAM特别适用于多传感器融合**。**额外的传感器测量值可以很容易地作为新的因子加入到框架中。提供绝对测量的传感器**，比如GPS、罗盘指南盘或高度计，可以用来**消除激光雷达惯性里程计在长时间运行或特征比较少的环境中累计的轨迹漂移**。位置识别也可以很容易地融入到系统中。为了提高系统的实时性能，我们**提出了一种滑动窗口方法，边缘化旧的激光雷达帧进行scan-matching扫描匹配**。**关键帧被有选择性地添加到因子图中，并且当同时产生激光雷达里程计因子和闭环因子时，新的关键帧只关联到固定大小的子关键帧集中**。这种局部尺度而不是全局尺度的scan-match扫描匹配方法提高了LIO-SAM算法框架的实时性能。提出的LIO-SAM方法在不同环境的三个移动平台中采集的数据集上进行了全面的评估。结果表明，**与LOAM和LIO-Mapping相比，LIO-SAM可以获得相似或更好的精度**。未来的工作包括在无人机上测试我们提出的LIO-SAM系统。