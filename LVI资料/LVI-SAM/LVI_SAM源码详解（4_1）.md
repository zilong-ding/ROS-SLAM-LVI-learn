## 雷达里程计

_**mapOptimization**_

定义了一个 ![[公式]](https://www.zhihu.com/equation?tex=mapOptimization) 类，由 ![[公式]](https://www.zhihu.com/equation?tex=ParamServer) 类公有继承而来。

成员变量如下所示。

```
// gtsam
NonlinearFactorGraph gtSAMgraph;
Values initialEstimate;
Values optimizedEstimate;
ISAM2 *isam;
Values isamCurrentEstimate;
Eigen::MatrixXd poseCovariance;

ros::Publisher pubLaserCloudSurround;
ros::Publisher pubOdomAftMappedROS;
ros::Publisher pubKeyPoses;
ros::Publisher pubPath;

ros::Publisher pubHistoryKeyFrames;
ros::Publisher pubIcpKeyFrames;
ros::Publisher pubRecentKeyFrames;
ros::Publisher pubRecentKeyFrame;
ros::Publisher pubCloudRegisteredRaw;
ros::Publisher pubLoopConstraintEdge;

ros::Subscriber subLaserCloudInfo;
ros::Subscriber subGPS;
ros::Subscriber subLoopInfo;

std::deque<nav_msgs::Odometry> gpsQueue;
lvi_sam::cloud_info cloudInfo;

// 历史所有关键帧的角点集合（降采样）
vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
// 历史所有关键帧的平面点集合（降采样）
vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

// 历史关键帧位姿（位置）
pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
// 历史关键帧位姿
pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

// 当前激光帧角点集合
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; // corner feature set from odoOptimization
// 当前激光帧平面点集合
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
// 当前激光帧角点集合，降采样，DS: DownSize
pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
// 当前激光帧平面点集合，降采样
pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf featuer set from odoOptimization

// 当前帧与局部 map 匹配上了的角点、平面点，加入同一集合；后面是对应点的参数
pcl::PointCloud<PointType>::Ptr laserCloudOri;
pcl::PointCloud<PointType>::Ptr coeffSel;

// 当前帧与局部 map 匹配上了的角点、参数、标记
std::vector<PointType> laserCloudOriCornerVec; // corner point holder for parallel computation
std::vector<PointType> coeffSelCornerVec;
std::vector<bool> laserCloudOriCornerFlag;
// 当前帧与局部 map 匹配上了的平面点、参数、标记
std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
std::vector<PointType> coeffSelSurfVec;
std::vector<bool> laserCloudOriSurfFlag;

// 局部 map 的角点集合
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
// 局部 map 的平面点集合
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
// 局部 map 的角点集合，降采样
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
// 局部 map 的平面点集合，降采样
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

// 局部关键帧构建的 map 点云，对应 kdtree，用于 scan-to-map 找相邻点
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

pcl::PointCloud<PointType>::Ptr latestKeyFrameCloud;
pcl::PointCloud<PointType>::Ptr nearHistoryKeyFrameCloud;

// 降采样
pcl::VoxelGrid<PointType> downSizeFilterCorner;
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterICP;
pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

ros::Time timeLaserInfoStamp;
double timeLaserInfoCur;

float transformTobeMapped[6];

std::mutex mtx;

bool isDegenerate = false;
cv::Mat matP;

// 当前激光帧角点数量
int laserCloudCornerLastDSNum = 0;
// 当前激光帧平面点数量
int laserCloudSurfLastDSNum = 0;

bool aLoopIsClosed = false;
int imuPreintegrationResetId = 0;

nav_msgs::Path globalPath;

// 当前帧位姿
Eigen::Affine3f transPointAssociateToMap;

map<int, int> loopIndexContainer; // from new to old
vector<pair<int, int>> loopIndexQueue;
vector<gtsam::Pose3> loopPoseQueue;
vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
```

___

首先是构造函数，对一些 subscriber 和 publisher 进行了定义，包括关键帧、点云、路径等，如下所示。

```
mapOptimization()
{
    // ISM2参数
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);

    // 发布历史关键帧里程计
    pubKeyPoses           = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/mapping/trajectory", 1);
    // 发布局部关键帧map的特征点云
    pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/mapping/map_global", 1);
    // 发布激光里程计，rviz 中表现为坐标轴
    pubOdomAftMappedROS   = nh.advertise<nav_msgs::Odometry>      (PROJECT_NAME + "/lidar/mapping/odometry", 1);
    // 发布激光里程计路径，rviz中表现为载体的运行轨迹
    pubPath               = nh.advertise<nav_msgs::Path>          (PROJECT_NAME + "/lidar/mapping/path", 1);

    // 订阅当前激光帧点云信息，来自featureExtraction
    subLaserCloudInfo     = nh.subscribe<lvi_sam::cloud_info>     (PROJECT_NAME + "/lidar/feature/cloud_info", 5, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
    // 订阅GPS里程计
    subGPS                = nh.subscribe<nav_msgs::Odometry>      (gpsTopic,                                   50, &mapOptimization::gpsHandler, this, ros::TransportHints().tcpNoDelay());
    // 订阅来自视觉闭环检测程序提供的闭环数据
    subLoopInfo           = nh.subscribe<std_msgs::Float64MultiArray>(PROJECT_NAME + "/vins/loop/match_frame", 5, &mapOptimization::loopHandler, this, ros::TransportHints().tcpNoDelay());

    // 发布闭环匹配关键帧局部map
    pubHistoryKeyFrames   = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/mapping/loop_closure_history_cloud", 1);
    // 发布当前关键帧经过闭环优化后的位姿变换之后的特征点云
    pubIcpKeyFrames       = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/mapping/loop_closure_corrected_cloud", 1);
    // 发布闭环边，rviz中表现为闭环帧之间的连线
    pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>(PROJECT_NAME + "/lidar/mapping/loop_closure_constraints", 1);

    // 发布局部map的降采样平面点集合
    pubRecentKeyFrames    = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/mapping/map_local", 1);
    // 发布历史帧（累加的）的角点、平面点降采样集合
    pubRecentKeyFrame     = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/mapping/cloud_registered", 1);
    // 发布当前帧原始点云配准之后的点云
    pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/mapping/cloud_registered_raw", 1);

    downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
    downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

    allocateMemory();
}
```

___

首先是接收处理后的点云，并进行一系列计算。

```
void laserCloudInfoHandler(const lvi_sam::cloud_infoConstPtr& msgIn)
{
    // extract time stamp
    // 当前激光帧时间戳
    timeLaserInfoStamp = msgIn->header.stamp;
    timeLaserInfoCur = msgIn->header.stamp.toSec();

    // extract info ana feature cloud
    // 提取当前激光帧角点、平面点集合
    cloudInfo = *msgIn;
    pcl::fromROSMsg(msgIn->cloud_corner,  *laserCloudCornerLast);
    pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);

    std::lock_guard<std::mutex> lock(mtx);

    // mapping执行频率控制
    static double timeLastProcessing = -1;
    if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval) {

        timeLastProcessing = timeLaserInfoCur;

        // 当前帧位姿初始化
        updateInitialGuess();

        // 提取局部角点、平面点云集合，加入局部map
        extractSurroundingKeyFrames();

        // 当前激光帧角点、平面点集合降采样
        downsampleCurrentScan();

        // scan-to-map优化当前帧位姿
        scan2MapOptimization();

        // 设置当前帧为关键帧并执行因子图优化
        saveKeyFramesAndFactor();

        // 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿，更新里程计轨迹
        correctPoses();

        // 发布激光里程计
        publishOdometry();

        // 发布里程计、点云、轨迹
        publishFrames();
    }
}
```

___

第一步是当前帧位姿初始化。如果是第一帧，用原始 imu 数据的 RPY 初始化当前帧位姿（旋转部分）；后续帧，用 imu 里程计计算两帧之间的增量位姿变换，作用于前一帧的激光位姿，得到当前帧激光位姿。

```
void updateInitialGuess()
{
    // 前一帧的位姿，这里指 lidar 的位姿        
    static Eigen::Affine3f lastImuTransformation;
    // system initialization
    // 如果关键帧集合为空，继续进行初始化
    if (cloudKeyPoses3D->points.empty())
    {
        // 当前帧位姿的旋转部分，用激光帧信息中的RPY（来自imu原始数据）初始化
        transformTobeMapped[0] = cloudInfo.imuRollInit;
        transformTobeMapped[1] = cloudInfo.imuPitchInit;
        transformTobeMapped[2] = cloudInfo.imuYawInit;

        if (!useImuHeadingInitialization)
            transformTobeMapped[2] = 0;

        lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
        return;
    }

    // use VINS odometry estimation for pose guess
     // 用当前帧和前一帧对应的imu里程计计算相对位姿变换，再用前一帧的位姿与相对变换，计算当前帧的位姿，存 transformTobeMapped
    static int odomResetId = 0;
    static bool lastVinsTransAvailable = false;
    static Eigen::Affine3f lastVinsTransformation;

    // 回顾之前赋值，cloudInfo.odomResetId = (int)round(startOdomMsg.pose.covariance[0])
    if (cloudInfo.odomAvailable == true && cloudInfo.odomResetId == odomResetId)
    {
        // ROS_INFO("Using VINS initial guess");
        // 如果是首次积分，则将 lastVinsTransformation 赋值为根据 odom 的 xyz，rpy 转换得到的 transform
        if (lastVinsTransAvailable == false)
        {
            // ROS_INFO("Initializing VINS initial guess");
            lastVinsTransformation = pcl::getTransformation(cloudInfo.odomX,    cloudInfo.odomY,     cloudInfo.odomZ, 
                                                            cloudInfo.odomRoll, cloudInfo.odomPitch, cloudInfo.odomYaw);
            lastVinsTransAvailable = true;
        } else {
        // 不是首次积分
            // ROS_INFO("Obtaining VINS incremental guess");
            // 那么首先从 odom 转换成 transform，获得当前 transform 在 lastVinsTransformation 下的位置
            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.odomX,    cloudInfo.odomY,     cloudInfo.odomZ, 
                                                                cloudInfo.odomRoll, cloudInfo.odomPitch, cloudInfo.odomYaw);
            // 当前帧相对于前一帧的位姿变换，imu 里程计计算得到
            Eigen::Affine3f transIncre = lastVinsTransformation.inverse() * transBack;

            // 将上一时态的 imu 的 transform 点乘相邻两个 odom 时态间的位姿变换
            // 将其赋值给 transformTobeMapped 数组，pcl::getTranslationAndEulerAngles 的功能是根据放射矩阵计算x,y,z,raw,pitch,yaw
            // 前一帧的位姿
            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            // 当前帧的位姿
            Eigen::Affine3f transFinal = transTobe * transIncre;
            // 更新当前帧位姿transformTobeMapped
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
            // 将视觉惯性里程计的 transform 赋值为 odom 的位姿
            lastVinsTransformation = pcl::getTransformation(cloudInfo.odomX,    cloudInfo.odomY,     cloudInfo.odomZ, 
                                                            cloudInfo.odomRoll, cloudInfo.odomPitch, cloudInfo.odomYaw);
            // 保存当前时态的 imu 位姿
            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }
    } else {
        // 没有 odom 信息，或者是第一帧进入的时候
        // ROS_WARN("VINS failure detected.");
        lastVinsTransAvailable = false;
        odomResetId = cloudInfo.odomResetId;
    }

    // use imu incremental estimation for pose guess (only rotation)
    // 只在第一帧调用（注意上面的return），用 imu 累计估计位姿（只估计旋转的）
    if (cloudInfo.imuAvailable == true)
    {
        // ROS_INFO("Using IMU initial guess");
        // 首先从 imu 转换成 transform，获得当前 transform 在 lastImuTransformation 下的位置
        // 当前帧的姿态角（来自原始imu数据）
        Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
        // 当前帧相对于前一帧的姿态变换
        Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

        // 将上一时态的 imu 的 transform 点乘相邻两个 imu 时态间的位姿变换，将其赋值给 transformTobeMapped 数组
        // 前一帧的位姿
        Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
        // 当前帧的位姿
        Eigen::Affine3f transFinal = transTobe * transIncre;
        // 更新当前帧位姿transformTobeMapped
        pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                        transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        // 保存当前时态的 imu 位姿
        lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
        return;
    }
}
```

___

第二步是非闭环处理，提取周围关键帧，默认为50米范围内。提取局部角点、平面点云集合，加入局部 map：对最近的一帧关键帧，搜索时空维度上相邻的关键帧集合，降采样；对关键帧集合中的每一帧，提取对应的角点、平面点，加入局部 map 中。  

```
void extractSurroundingKeyFrames()
{
    // 如果接收到的关键帧点云为空
    if (cloudKeyPoses3D->points.empty() == true)
        return; 
        
    extractNearby();
}

void extractNearby()
{
    pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    // extract all the nearby key poses and downsample them
    // kdtree的输入，全局关键帧位姿集合（历史所有关键帧集合）
    kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
    // surroundingKeyframeSearchRadius默认值为 50.0，scan-to-map 优化的距离
    // 对最近的一帧关键帧，在半径区域内搜索空间区域上相邻的关键帧集合
    kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
    // 遍历搜索结果，pointSearchInd 存的是结果在 cloudKeyPoses3D 下面的索引
    for (int i = 0; i < (int)pointSearchInd.size(); ++i)
    {
        int id = pointSearchInd[i];
        // 加入相邻关键帧位姿集合中
        surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
    }

    // 降采样一下
    downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
    downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);

    // also extract some latest key frames in case the robot rotates in one position
    // 提取 10 秒内的关键帧，比如当机器人在原地旋转
    int numPoses = cloudKeyPoses3D->size();
    for (int i = numPoses-1; i >= 0; --i)
    {
        if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
            surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
        else
            break;
    }

    // 将相邻关键帧集合对应的角点、平面点，加入到局部map中，作为scan-to-map匹配的局部点云地图
    extractCloud(surroundingKeyPosesDS);
}

// 相邻关键帧集合对应的角点、平面点，加入到局部 map 中
// 称之为局部 map，后面进行 scan-to-map 匹配，所用到的 map 就是这里的相邻关键帧对应点云集合
void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
{
    std::vector<pcl::PointCloud<PointType>> laserCloudCornerSurroundingVec;
    std::vector<pcl::PointCloud<PointType>> laserCloudSurfSurroundingVec;

    laserCloudCornerSurroundingVec.resize(cloudToExtract->size());
    laserCloudSurfSurroundingVec.resize(cloudToExtract->size());

    // extract surrounding map
    #pragma omp parallel for num_threads(numberOfCores)
    // 遍历当前帧（实际是取最近的一个关键帧来找它相邻的关键帧集合）时空维度上相邻的关键帧集合
    for (int i = 0; i < (int)cloudToExtract->size(); ++i)
    {
        int thisKeyInd = (int)cloudToExtract->points[i].intensity;
        // 提取 50 米范围内的点，transformPointCloud 作用是返回输入点云乘以输入的 transform
        if (pointDistance(cloudKeyPoses3D->points[thisKeyInd], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
            continue;
        // 相邻关键帧对应的角点、平面点云，通过 6D 位姿变换到世界坐标系下
        laserCloudCornerSurroundingVec[i]  = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
        laserCloudSurfSurroundingVec[i]    = *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
    }

    // fuse the map
    // 赋值线和面特征点集，并且进行下采样
    // laserCloudCornerFromMapDS 比较重要
    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear(); 
    for (int i = 0; i < (int)cloudToExtract->size(); ++i)
    {
        // 加入局部map
        *laserCloudCornerFromMap += laserCloudCornerSurroundingVec[i];
        *laserCloudSurfFromMap   += laserCloudSurfSurroundingVec[i];
    }

    // Downsample the surrounding corner key frames (or map)
    // 降采样局部角点map
    downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
    downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
    // Downsample the surrounding surf key frames (or map)
    // 降采样局部平面点map
    downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
    downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
}
```

___

第三步是下采样当前的二维线束，即 scan，不做赘述。

```
void downsampleCurrentScan()
{
    // Downsample cloud from current scan
    laserCloudCornerLastDS->clear();
    downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
    downSizeFilterCorner.filter(*laserCloudCornerLastDS);
    laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

    laserCloudSurfLastDS->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfLastDS);
    laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
}
```

___

然后是 scan-map 优化。

1、要求当前帧特征点数量足够多，且匹配的点数够多，才执行优化。  
2、迭代30次（上限）优化。  
1) 当前激光帧角点寻找局部map匹配点  
a.更新当前帧位姿，将当前帧角点坐标变换到map系下，在局部map中查找5个最近点，距离小于1m，且5个点构成直线（用距离中心点的协方差矩阵，特征值进行判断），则认为匹配上了；  
b.计算当前帧角点到直线的距离、垂线的单位向量，存储为角点参数  
2) 当前激光帧平面点寻找局部map匹配点  
a.更新当前帧位姿，将当前帧平面点坐标变换到map系下，在局部map中查找5个最近点，距离小于1m，且5个点构成平面（最小二乘拟合平面），则认为匹配上了  
b.计算当前帧平面点到平面的距离、垂线的单位向量，存储为平面点参数  
3) 提取当前帧中与局部map匹配上了的角点、平面点，加入同一集合  
4) 对匹配特征点计算Jacobian矩阵，观测值为特征点到直线、平面的距离，构建高斯牛顿方程，迭代优化当前位姿，存transformTobeMapped  
3、用imu原始RPY数据与scan-to-map优化后的位姿进行加权融合，更新当前帧位姿的roll、pitch，约束z坐标。

```
void scan2MapOptimization()
{
    // 如果没有关键帧点云数据
    if (cloudKeyPoses3D->points.empty())
        return;
    // 关键点数量大于阈值，边为10，面为100
    if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
    {
        // kdtree 输入为局部 map 点云
        kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

        for (int iterCount = 0; iterCount < 30; iterCount++)
        {
            // 每次迭代清空特征点集合
            laserCloudOri->clear();
            coeffSel->clear();

            // 当前激光帧角点寻找局部map匹配点
            cornerOptimization();
            // 当前激光帧平面点寻找局部map匹配点
            surfOptimization();
            // 提取当前帧中与局部map匹配上了的角点、平面点，加入同一集合
            combineOptimizationCoeffs();
            // scan-to-map优化
            if (LMOptimization(iterCount) == true)
                break;              
        }
        // 用imu原始RPY数据与scan-to-map优化后的位姿进行加权融合，更新当前帧位姿的roll、pitch，约束z坐标
        transformUpdate();
    } else {
        ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
    }
}

// 当前激光帧角点寻找局部map匹配点
void cornerOptimization()
{
    // 仿射变换，更新当前位姿与地图间位姿变换
    updatePointAssociateToMap();

    // openmp
    #pragma omp parallel for num_threads(numberOfCores)
    // 遍历点云，构建点到直线的约束
    for (int i = 0; i < laserCloudCornerLastDSNum; i++)
    {
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // 角点（坐标还是lidar系）
        pointOri = laserCloudCornerLastDS->points[i];
        // 将点从 lidar 坐标系变换到 map 坐标系
        pointAssociateToMap(&pointOri, &pointSel);
        // 在局部角点 map 中查找当前角点相邻的 5 个角点
        kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
        
        // 只有最近的点都在一定阈值内（1米）才进行计算
        if (pointSearchSqDis[4] < 1.0) {
            // 计算 5 个点的均值坐标，记为中心点
            float cx = 0, cy = 0, cz = 0;
            for (int j = 0; j < 5; j++) {
                cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
            }
            cx /= 5; cy /= 5;  cz /= 5;

            // 根据均值计算协方差
            float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
            for (int j = 0; j < 5; j++) {
                // 计算点与中心点之间的距离
                float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                a22 += ay * ay; a23 += ay * az;
                a33 += az * az;
            }
            a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

            // 构建协方差矩阵
            matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
            matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
            matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

            // 特征值分解，求协方差矩阵的特征值和特征向量, 特征值：matD1，特征向量：matV1
            cv::eigen(matA1, matD1, matV1);
            // 如果最大的特征值相比次大特征值，大很多，认为构成了线，角点是合格的
            if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {
                // 当前帧角点坐标（map 系下）
                float x0 = pointSel.x;
                float y0 = pointSel.y;
                float z0 = pointSel.z;
                // 局部 map 对应中心角点，沿着特征向量（直线方向）方向，前后各取一个点
                float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                // area_012，也就是三个点组成的三角形面积*2，叉积的模|axb|=a*b*sin(theta)
                float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));
                // line_12，底边边长
                float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                // 两次叉积，得到点到直线的垂线段单位向量，x 分量，下面同理
                float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                            + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                            - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                            + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                // 三角形的高，也就是点到直线距离
                float ld2 = a012 / l12;

                // 下面涉及到一个鲁棒核函数，作者简单地设计了这个核函数
                // 距离越大，s 越小，使用距离惩罚因子（权重）
                float s = 1 - 0.9 * fabs(ld2);

                // 点到直线的垂线段单位向量
                coeff.x = s * la;
                coeff.y = s * lb;
                coeff.z = s * lc;
                // 点到直线距离
                coeff.intensity = s * ld2;
                // 程序末尾根据 s 的值来判断是否将点云点放入点云集合 laserCloudOri 以及 coeffSel 中
                if (s > 0.1) {
                    // 当前激光帧角点，加入匹配集合中
                    laserCloudOriCornerVec[i] = pointOri;
                    // 角点的参数
                    coeffSelCornerVec[i] = coeff;
                    laserCloudOriCornerFlag[i] = true;
                }
            }
        }
    }
}

// 当前激光帧平面点寻找局部map匹配点
void surfOptimization()
{
    // 仿射变换，更新当前位姿与地图间位姿变换
    updatePointAssociateToMap();

    #pragma omp parallel for num_threads(numberOfCores)
    // 遍历当前帧平面点集合
    for (int i = 0; i < laserCloudSurfLastDSNum; i++)
    {
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // 平面点（坐标还是 lidar 系）
        pointOri = laserCloudSurfLastDS->points[i];
        // pointSel 为 pointOri 在 map 下的位姿
        pointAssociateToMap(&pointOri, &pointSel);
        // kd 树搜索最近的 5 个点
        kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0;
        Eigen::Vector3f matX0;

        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();

        // 如果最远的点的距离都小于 1 米
        if (pointSearchSqDis[4] < 1.0) {
            // 那么将这五个点存入 matA
            for (int j = 0; j < 5; j++) {
                matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
            }
            // Ax = B，根据这五个点求解平面方程，进行 QR 分解，获得平面方程解
            // 假设平面方程为ax+by+cz+1=0，这里就是求方程的系数abc，d=1
            matX0 = matA0.colPivHouseholderQr().solve(matB0);

            // 平面方程的系数，也是法向量的分量
            float pa = matX0(0, 0);
            float pb = matX0(1, 0);
            float pc = matX0(2, 0);
            float pd = 1;

            // 将 matX 归一化，也就是单位法向量
            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps; pb /= ps; pc /= ps; pd /= ps;

            // 检查平面是否合格，如果5个点中有点到平面的距离超过0.2m，那么认为这些点太分散了，不构成平面
            bool planeValid = true;
            for (int j = 0; j < 5; j++) {
                if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                            pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                            pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                    planeValid = false;
                    break;
                }
            }
            
            // 平面合格
            if (planeValid) {
                // 当前激光帧点到平面距离
                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                        + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                // 点到平面垂线单位法向量（其实等价于平面法向量）
                coeff.x = s * pa;
                coeff.y = s * pb;
                coeff.z = s * pc;
                // 点到平面距离
                coeff.intensity = s * pd2;

                if (s > 0.1) {
                    // 当前激光帧平面点，加入匹配集合中
                    laserCloudOriSurfVec[i] = pointOri;
                    // 系数
                    coeffSelSurfVec[i] = coeff;
                    laserCloudOriSurfFlag[i] = true;
                }
            }
        }
    }
}

// 提取当前帧中与局部map匹配上了的角点、平面点，加入同一集合
void combineOptimizationCoeffs()
{
    // combine corner coeffs
    // 遍历当前帧角点集合，提取出与局部map匹配上了的角点
    for (int i = 0; i < laserCloudCornerLastDSNum; ++i){
        if (laserCloudOriCornerFlag[i] == true){
            laserCloudOri->push_back(laserCloudOriCornerVec[i]);
            coeffSel->push_back(coeffSelCornerVec[i]);
        }
    }
    // combine surf coeffs
    // 遍历当前帧平面点集合，提取出与局部map匹配上了的平面点
    for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
        if (laserCloudOriSurfFlag[i] == true){
            laserCloudOri->push_back(laserCloudOriSurfVec[i]);
            coeffSel->push_back(coeffSelSurfVec[i]);
        }
    }
    // reset flag for next iteration
    // 清空标记
    std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
}

// scan-to-map优化，也就是优化 argmin_x ||Ax-b||^2
bool LMOptimization(int iterCount)
{
    // This optimization is from the original loam_velodyne, need to cope with coordinate transformation
    // lidar <- camera      ---     camera <- lidar
    // x = z                ---     x = y
    // y = x                ---     y = z
    // z = y                ---     z = x
    // roll = yaw           ---     roll = pitch
    // pitch = roll         ---     pitch = yaw
    // yaw = pitch          ---     yaw = roll

    // lidar -> camera
    // 雷达到相机变换的三轴方向的正弦和余弦
    float srx = sin(transformTobeMapped[1]);
    float crx = cos(transformTobeMapped[1]);
    float sry = sin(transformTobeMapped[2]);
    float cry = cos(transformTobeMapped[2]);
    float srz = sin(transformTobeMapped[0]);
    float crz = cos(transformTobeMapped[0]);

    int laserCloudSelNum = laserCloudOri->size();
    // 当前帧匹配特征点数太少
    if (laserCloudSelNum < 50) {
        return false;
    }

    cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

    PointType pointOri, coeff;

    // 遍历匹配特征点，构建Jacobian矩阵
    for (int i = 0; i < laserCloudSelNum; i++) {
        // 雷达和相机的坐标轴不一样，具体看 lvi_sam 作者 github
        // lidar -> camera
        // 特征点位置
        pointOri.x = laserCloudOri->points[i].y;
        pointOri.y = laserCloudOri->points[i].z;
        pointOri.z = laserCloudOri->points[i].x;
        // lidar -> camera
        // coeff 为点到直线/平面距离
        coeff.x = coeffSel->points[i].y;
        coeff.y = coeffSel->points[i].z;
        coeff.z = coeffSel->points[i].x;
        coeff.intensity = coeffSel->points[i].intensity;
        // in camera
        // 求导
        float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                    + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                    + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

        float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                    + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                    + ((-cry*crz - srx*sry*srz)*pointOri.x 
                    + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

        float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                    + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                    + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
        // lidar -> camera
        matA.at<float>(i, 0) = arz;
        matA.at<float>(i, 1) = arx;
        matA.at<float>(i, 2) = ary;
        matA.at<float>(i, 3) = coeff.z;
        matA.at<float>(i, 4) = coeff.x;
        matA.at<float>(i, 5) = coeff.y;
        // 点到直线距离、平面距离，作为观测值
        matB.at<float>(i, 0) = -coeff.intensity;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    // J^T·J·delta_x = -J^T·f 高斯牛顿
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    // 首次迭代，检查近似 Hessian 矩阵（J^T·J）是否退化，或者称为奇异，行列式值=0
    if (iterCount == 0) 
    {
        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

        // 奇异值分解
        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        isDegenerate = false;
        float eignThre[6] = {100, 100, 100, 100, 100, 100};
        for (int i = 5; i >= 0; i--) {
            if (matE.at<float>(0, i) < eignThre[i]) {
                for (int j = 0; j < 6; j++) {
                    matV2.at<float>(i, j) = 0;
                }
                isDegenerate = true;
            } else {
                break;
            }
        }
        matP = matV.inv() * matV2;
    }

    if (isDegenerate) {
        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP * matX2;
    }

    // 更新当前位姿 x = x + delta_x
    transformTobeMapped[0] += matX.at<float>(0, 0);
    transformTobeMapped[1] += matX.at<float>(1, 0);
    transformTobeMapped[2] += matX.at<float>(2, 0);
    transformTobeMapped[3] += matX.at<float>(3, 0);
    transformTobeMapped[4] += matX.at<float>(4, 0);
    transformTobeMapped[5] += matX.at<float>(5, 0);

    float deltaR = sqrt(
                        pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                        pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                        pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
    float deltaT = sqrt(
                        pow(matX.at<float>(3, 0) * 100, 2) +
                        pow(matX.at<float>(4, 0) * 100, 2) +
                        pow(matX.at<float>(5, 0) * 100, 2));

    // delta_x很小，认为收敛
    if (deltaR < 0.05 && deltaT < 0.05) {
        return true; // converged
    }
    return false; // keep optimizing
}

// 用imu原始RPY数据与scan-to-map优化后的位姿进行加权融合，更新当前帧位姿的roll、pitch，约束z坐标
void transformUpdate()
{
    if (cloudInfo.imuAvailable == true)
    {
        // 俯仰角小于1.4
        if (std::abs(cloudInfo.imuPitchInit) < 1.4)
        {
            double imuWeight = 0.01;
            tf::Quaternion imuQuaternion;
            tf::Quaternion transformQuaternion;
            double rollMid, pitchMid, yawMid;

            // slerp roll
            // roll 角求加权均值，用 scan-to-map 优化得到的位姿与 imu 原始 RPY 数据，进行加权平均
            transformQuaternion.setRPY(transformTobeMapped[0], 0, 0); 
            imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
            tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
            transformTobeMapped[0] = rollMid;

            // slerp pitch
            // pitch 角求加权均值，用 scan-to-map 优化得到的位姿与 imu 原始 RPY 数据，进行加权平均
            transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
            imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
            tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
            transformTobeMapped[1] = pitchMid;
        }
    }

    // 更新当前帧位姿的roll, pitch, z坐标
    // 因为是小车，roll、pitch是相对稳定的，不会有很大变动，一定程度上可以信赖imu的数据，z是进行高度约束
    transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
    transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
    transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);
}
```

___

第五步是设置当前帧为关键帧并执行因子图优化。

1、计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧；  
2、添加激光里程计因子、GPS因子、闭环因子；  
3、执行因子图优化；  
4、得到当前帧优化后位姿，位姿协方差；  
5、添加cloudKeyPoses3D，cloudKeyPoses6D，更新transformTobeMapped，添加当前关键帧的角点、平面点集合。

```
// 设置当前帧为关键帧并执行因子图优化
void saveKeyFramesAndFactor()
{
    // 计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
    if (saveFrame() == false)
        return;

    // odom factor
    // 激光里程计因子
    addOdomFactor();

    // gps factor
    // addGPSFactor();

    // loop factor
    // 闭环因子
    addLoopFactor();

    // update iSAM
    // 执行优化
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    
    // update之后要清空一下保存的因子图，注：历史数据不会清掉，ISAM保存起来了
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    //save key poses
    PointType thisPose3D;
    PointTypePose thisPose6D;
    Pose3 latestEstimate;

    // 优化结果
    isamCurrentEstimate = isam->calculateEstimate();
    latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
    // cout << "****************************************************" << endl;
    // isamCurrentEstimate.print("Current estimate: ");

    // cloudKeyPoses3D加入当前帧位姿
    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    // 索引
    thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
    cloudKeyPoses3D->push_back(thisPose3D);

    // cloudKeyPoses6D加入当前帧位姿
    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
    thisPose6D.roll  = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw   = latestEstimate.rotation().yaw();
    thisPose6D.time = timeLaserInfoCur;
    cloudKeyPoses6D->push_back(thisPose6D);

    // cout << "****************************************************" << endl;
    // cout << "Pose covariance:" << endl;
    // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
    // 位姿协方差
    poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

    // save updated transform
    // transformTobeMapped更新当前帧位姿
    transformTobeMapped[0] = latestEstimate.rotation().roll();
    transformTobeMapped[1] = latestEstimate.rotation().pitch();
    transformTobeMapped[2] = latestEstimate.rotation().yaw();
    transformTobeMapped[3] = latestEstimate.translation().x();
    transformTobeMapped[4] = latestEstimate.translation().y();
    transformTobeMapped[5] = latestEstimate.translation().z();

    // save all the received edge and surf points
    // 当前帧激光角点、平面点，降采样集合
    pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
    pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

    // save key frame cloud
    // 保存特征点降采样集合
    cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);

    // save path for visualization
    // 更新里程计轨迹，也就是将 thisPose6D 的位姿加入到 "/lidar/mapping/path" 中去
    updatePath(thisPose6D);
}

void addOdomFactor()
{
    // 检查是否有关键帧点
    if (cloudKeyPoses3D->points.empty())
    {
        noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
        gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
    }else{
        noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
        gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
        // 添加关键帧最近的两个位姿
        gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
        initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        // if (isDegenerate)
        // {
            // adding VINS constraints is deleted as benefits are not obvious, disable for now
            // gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), vinsPoseFrom.between(vinsPoseTo), odometryNoise));
        // }
    }
}

void addLoopFactor()
{
    if (loopIndexQueue.empty())
        return;

    for (size_t i = 0; i < loopIndexQueue.size(); ++i)
    {
        // 添加相邻两个回环位姿
        int indexFrom = loopIndexQueue[i].first;
        int indexTo = loopIndexQueue[i].second;
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }
    // 清空所有的回环数据队列
    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
}
```

___

第六步是修正位姿，后端的任务。

```
// 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿，更新里程计轨迹
void correctPoses()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    if (aLoopIsClosed == true)
    {
        // clear path
        // 清空里程计轨迹
        globalPath.poses.clear();

        // update key poses
        int numPoses = isamCurrentEstimate.size();
        // 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿
        for (int i = 0; i < numPoses; ++i)
        {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

            // 更新里程计轨迹
            updatePath(cloudKeyPoses6D->points[i]);
        }

        aLoopIsClosed = false;
        // ID for reseting IMU pre-integration
        ++imuPreintegrationResetId;
    }
}
```

___

最后两步是发布里程计和关键帧。

```
void publishOdometry()
{
    // Publish odometry for ROS
    nav_msgs::Odometry laserOdometryROS;
    laserOdometryROS.header.stamp = timeLaserInfoStamp;
    laserOdometryROS.header.frame_id = "odom";
    laserOdometryROS.child_frame_id = "odom_mapping";
    laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
    laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
    laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
    laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    laserOdometryROS.pose.covariance[0] = double(imuPreintegrationResetId);
    pubOdomAftMappedROS.publish(laserOdometryROS);
    // Publish TF
    static tf::TransformBroadcaster br;
    tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                                    tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
    tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, "odom", "lidar_link");
    br.sendTransform(trans_odom_to_lidar);
}

void publishFrames()
{
    if (cloudKeyPoses3D->points.empty())
        return;
    // publish key poses
    publishCloud(&pubKeyPoses, cloudKeyPoses6D, timeLaserInfoStamp, "odom");
    // Publish surrounding key frames
    publishCloud(&pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, "odom");
    // publish registered key frame
    if (pubRecentKeyFrame.getNumSubscribers() != 0)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
        *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);
        *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
        publishCloud(&pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, "odom");
    }
    // publish registered high-res raw cloud
    if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
        PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
        *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
        publishCloud(&pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, "odom");
    }
    // publish path
    if (pubPath.getNumSubscribers() != 0)
    {
        globalPath.header.stamp = timeLaserInfoStamp;
        globalPath.header.frame_id = "odom";
        pubPath.publish(globalPath);
    }
}
```

___

## 目录