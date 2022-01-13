## 雷达里程计

_**imageProjection**_

定义了一个 ![[公式]](https://www.zhihu.com/equation?tex=imageProjection) 类，由 ![[公式]](https://www.zhihu.com/equation?tex=ParamServer) 公有继承而来。

作者终于将成员变量定义为私有的了。

```
// imu队列、odom队列互斥锁
std::mutex imuLock;
std::mutex odoLock;

// 订阅原始激光点云
ros::Subscriber subLaserCloud;
ros::Publisher  pubLaserCloud;

// 发布当前帧校正后点云
ros::Publisher pubExtractedCloud;
ros::Publisher pubLaserCloudInfo;

// imu 数据队列（原始数据，转lidar系下）
ros::Subscriber subImu;
std::deque<sensor_msgs::Imu> imuQueue;

// imu里程计队列
ros::Subscriber subOdom;
std::deque<nav_msgs::Odometry> odomQueue;

// 激光点云数据队列
std::deque<sensor_msgs::PointCloud2> cloudQueue;
// 队列头部帧，作为当前处理帧点云
sensor_msgs::PointCloud2 currentCloudMsg;

// 当前激光帧起止时刻间对应的imu数据，计算相对于起始时刻的旋转增量和时间戳
// 用于插值计算当前激光帧起止时间范围内，每一时刻的旋转姿态
double *imuTime = new double[queueLength];
double *imuRotX = new double[queueLength];
double *imuRotY = new double[queueLength];
double *imuRotZ = new double[queueLength];

int imuPointerCur;
bool firstPointFlag;
Eigen::Affine3f transStartInverse;

// 当前帧原始激光点云
pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
// 当期帧运动畸变校正之后的激光点云
pcl::PointCloud<PointType>::Ptr   fullCloud;
// 从 fullCloud 中提取特征点
pcl::PointCloud<PointType>::Ptr   extractedCloud;

int deskewFlag;
cv::Mat rangeMat;

bool odomDeskewFlag;
// 当前激光帧起止时刻对应 imu 里程计位姿变换，该变换对应的平移增量
// 用于插值计算当前激光帧起止时间范围内，每一时刻的位置
float odomIncreX;
float odomIncreY;
float odomIncreZ;

// 当前帧激光点云运动畸变校正之后的数据，包括点云数据、初始位姿、姿态角等，发布给 featureExtraction 进行特征提取
lvi_sam::cloud_info cloudInfo;
// 当前帧起始时刻
double timeScanCur;
// 当前帧结束时刻
double timeScanNext;
// 当前帧header，包含时间戳信息
std_msgs::Header cloudHeader;
```

___

构造函数如下。

```
ImageProjection():
deskewFlag(0)
{
    // 订阅原始imu数据
    subImu        = nh.subscribe<sensor_msgs::Imu>        (imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
    // 订阅imu里程计，由 imuPreintegration 积分计算得到的每时刻 imu 位姿
    subOdom       = nh.subscribe<nav_msgs::Odometry>      (PROJECT_NAME + "/vins/odometry/imu_propagate_ros", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
    // 订阅原始 lidar 数据
    subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());
    // 发布当前激光帧运动畸变校正后的点云，有效点
    pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> (PROJECT_NAME + "/lidar/deskew/cloud_deskewed", 5);
    // 发布当前激光帧运动畸变校正后的点云信息
    pubLaserCloudInfo = nh.advertise<lvi_sam::cloud_info>      (PROJECT_NAME + "/lidar/deskew/cloud_info", 5);

    // 初始化
    allocateMemory();
    // 重置参数
    resetParameters();
    // pcl日志级别，只打ERROR日志
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}
```

___

首先是 imu 和 odom 的回调函数，利用互斥锁将收到的数据信息 push\_back 到成员变量数组中去。

```
void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
{
    sensor_msgs::Imu thisImu = imuConverter(*imuMsg);
    std::lock_guard<std::mutex> lock1(imuLock);
    imuQueue.push_back(thisImu);
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
{
    std::lock_guard<std::mutex> lock2(odoLock);
    odomQueue.push_back(*odometryMsg);
}
```

___

然后是点云的回调函数，显而易见包括六个部分。

1、添加一帧激光点云到队列，取出最早一帧作为当前帧，计算起止时间戳，检查数据有效。  
2、当前帧起止时刻对应的imu数据、imu里程计数据处理：  
imu数据：  
1) 遍历当前激光帧起止时刻之间的imu数据，初始时刻对应imu的姿态角RPY设为当前帧的初始姿态角。  
2) 用角速度、时间积分，计算每一时刻相对于初始时刻的旋转量，初始时刻旋转设为0。  
imu里程计数据：  
1) 遍历当前激光帧起止时刻之间的imu里程计数据，初始时刻对应imu里程计设为当前帧的初始位姿。  
2) 用起始、终止时刻对应imu里程计，计算相对位姿变换，保存平移增量。  
3、当前帧激光点云运动畸变校正：  
1) 检查激光点距离、扫描线是否合规。  
2) 激光运动畸变校正，保存激光点。  
4、提取有效激光点，存extractedCloud。  
5、发布当前帧校正后点云，有效点。  
6、重置参数，接收每帧lidar数据都要重置这些参数。

```
void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    // 添加一帧激光点云到队列，取出最早一帧作为当前帧，计算起止时间戳，检查数据有效性
    if (!cachePointCloud(laserCloudMsg))
        return;

    // 当前帧起止时刻对应的 imu 数据、imu 里程计数据处理
    if (!deskewInfo())
        return;

    // 当前帧激光点云运动畸变校正
    // 1、检查激光点距离、扫描线是否合规
    // 2、激光运动畸变校正，保存激光点
    projectPointCloud();

    // 提取有效激光点，存 extractedCloud
    cloudExtraction();
   
    // 发布当前帧校正后点云，有效点
    publishClouds();

    // 重置参数，接收每帧 lidar 数据都要重置这些参数
    resetParameters();
}
```

第一步是判断点云是否有效。

```
bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    // cache point cloud
    cloudQueue.push_back(*laserCloudMsg);

    if (cloudQueue.size() <= 2)
        return false;
    else
    {
        // 取出激光点云队列中最早的一帧
        currentCloudMsg = cloudQueue.front();
        cloudQueue.pop_front();
        // 当前帧头部
        cloudHeader = currentCloudMsg.header;
        // 当前帧起始时刻
        timeScanCur = cloudHeader.stamp.toSec();
        // 当前帧结束时刻，注：点云中激光点的time记录相对于当前帧第一个激光点的时差，第一个点time=0
        timeScanNext = cloudQueue.front().header.stamp.toSec();
    }

    // convert cloud
    // 转换成pcl点云格式
    pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);

    // check dense flag
    // 存在无效点，Nan或者Inf
    if (laserCloudIn->is_dense == false)
    {
        ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
        ros::shutdown();
    }

    // check ring channel
    // 检查是否存在 ring 通道，注意 static 只检查一次
    static int ringFlag = 0;
    if (ringFlag == 0)
    {
        ringFlag = -1;
        for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
        {
            if (currentCloudMsg.fields[i].name == "ring")
            {
                ringFlag = 1;
                break;
            }
        }
        if (ringFlag == -1)
        {
            ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
            ros::shutdown();
        }
    }     

    // check point time
    // 检查是否存在time通道
    if (deskewFlag == 0)
    {
        deskewFlag = -1;
        for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
        {
            if (currentCloudMsg.fields[i].name == timeField)
            {
                deskewFlag = 1;
                break;
            }
        }
        if (deskewFlag == -1)
            ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
    }

    return true;
}
```

___

第二步是去畸变信息，也就是对 imu 和 odom 数组进行预处理。

```
bool deskewInfo()
{
    std::lock_guard<std::mutex> lock1(imuLock);
    std::lock_guard<std::mutex> lock2(odoLock);

    // make sure IMU data available for the scan
    // 要求imu数据包含激光数据，否则不往下处理了
    if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanNext)
    {
        ROS_DEBUG("Waiting for IMU data ...");
        return false;
    }

    imuDeskewInfo();

    odomDeskewInfo();

    return true;
}

/**
 * 当前帧对应imu数据处理
 * 1、遍历当前激光帧起止时刻之间的imu数据，初始时刻对应imu的姿态角RPY设为当前帧的初始姿态角
 * 2、用角速度、时间积分，计算每一时刻相对于初始时刻的旋转量，初始时刻旋转设为0
 * 注：imu数据都已经转换到lidar系下了
*/
void imuDeskewInfo()
{
    cloudInfo.imuAvailable = false;

    while (!imuQueue.empty())
    {
        // 只保留 0.01s 间隔内的 imu 信息
        if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
            imuQueue.pop_front();
        else
            break;
    }

    if (imuQueue.empty())
        return;
    
    // 指向的是当前积分到的位置
    imuPointerCur = 0;

    // 遍历当前激光帧起止时刻（前后扩展0.01s）之间的imu数据
    for (int i = 0; i < (int)imuQueue.size(); ++i)
    {
        sensor_msgs::Imu thisImuMsg = imuQueue[i];
        double currentImuTime = thisImuMsg.header.stamp.toSec();

        // get roll, pitch, and yaw estimation for this scan
        // 提取imu姿态角RPY，作为当前lidar帧初始姿态角
        if (currentImuTime <= timeScanCur)
            imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);
        
        // 间隔必须不大于 0.01 秒
        if (currentImuTime > timeScanNext + 0.01)
            break;

        // 如果积分指针指向头部，将头部初始化为全0，时间为当前时间
        // 并且将指针向后移一位，当前状态不作积分
        // 第一帧imu旋转角初始化
        if (imuPointerCur == 0){
            imuRotX[0] = 0;
            imuRotY[0] = 0;
            imuRotZ[0] = 0;
            imuTime[0] = currentImuTime;
            ++imuPointerCur;
            continue;
        }

        // get angular velocity
        // 提取imu角速度
        double angular_x, angular_y, angular_z;
        imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

        // integrate rotation
        // 利用当前时间和上一步的时间，乘以当前 imu 的各方向角速度
        // 加上一步的各个位置，得到当前的各个位置数据
        // 并且将指针后移一位
        // imu帧间时差
        double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
        // 当前时刻旋转角 = 前一时刻旋转角 + 角速度 * 时差
        imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
        imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
        imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
        imuTime[imuPointerCur] = currentImuTime;
        ++imuPointerCur;
    }
    // 由于循环结束后指针多加一位，指向数组尾部填充位
    // 因此将指针减一，指向数组实际数据的尾部
    --imuPointerCur;
    // 没有合规的imu数据
    if (imuPointerCur <= 0)
        return;
    
    cloudInfo.imuAvailable = true;
}

/**
 * 当前帧对应imu里程计处理
 * 1、遍历当前激光帧起止时刻之间的imu里程计数据，初始时刻对应imu里程计设为当前帧的初始位姿
 * 2、用起始、终止时刻对应imu里程计，计算相对位姿变换，保存平移增量
 * 注：imu数据都已经转换到lidar系下了
*/
void odomDeskewInfo()
{
    cloudInfo.odomAvailable = false;

    while (!odomQueue.empty())
    {
        // 只要 0.01 秒内的 odom 信息
        if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
            odomQueue.pop_front();
        else
            break;
    }
    // odom 队列中为空，则退出函数
    if (odomQueue.empty())
        return;
    // odom 队列中元素时间都在 scan 开始时间后面，则退出函数
    if (odomQueue.front().header.stamp.toSec() > timeScanCur)
        return;

    // get start odometry at the beinning of the scan
    // 提取当前激光帧起始时刻的imu里程计
    nav_msgs::Odometry startOdomMsg;

    for (int i = 0; i < (int)odomQueue.size(); ++i)
    {
        startOdomMsg = odomQueue[i];
        // 找到队列中第一个时间戳不早于 scan 开始时间的 odom 元素
        if (ROS_TIME(&startOdomMsg) < timeScanCur)
            continue;
        else
            break;
    }
    
    // 提取imu里程计姿态角
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    // Initial guess used in mapOptimization
    // 用当前激光帧起始时刻的imu里程计，初始化lidar位姿，后面用于mapOptmization
    cloudInfo.odomX = startOdomMsg.pose.pose.position.x;
    cloudInfo.odomY = startOdomMsg.pose.pose.position.y;
    cloudInfo.odomZ = startOdomMsg.pose.pose.position.z;
    cloudInfo.odomRoll  = roll;
    cloudInfo.odomPitch = pitch;
    cloudInfo.odomYaw   = yaw;
    cloudInfo.odomResetId = (int)round(startOdomMsg.pose.covariance[0]);

    cloudInfo.odomAvailable = true;

    // get end odometry at the end of the scan
    odomDeskewFlag = false;
    // 如果队列中的时间戳都比 scan 结束时间早，则退出函数
    if (odomQueue.back().header.stamp.toSec() < timeScanNext)
        return;

    // 提取当前激光帧结束时刻的imu里程计
    nav_msgs::Odometry endOdomMsg;

    for (int i = 0; i < (int)odomQueue.size(); ++i)
    {
        endOdomMsg = odomQueue[i];
        // 找到队列中第一个时间戳不早于 scan 结束时间的 odom 元素
        if (ROS_TIME(&endOdomMsg) < timeScanNext)
            continue;
        else
            break;
    }
    // 四舍五入后 odom 开始和结束时刻的协方差要相等，否则退出函数
    if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
        return;

    // 计算开始到结束的 translation 和 欧拉角
    Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

    tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

    // 起止时刻imu里程计的相对变换
    Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

    // 相对变换，提取增量平移、旋转（欧拉角）
    float rollIncre, pitchIncre, yawIncre;
    pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

    odomDeskewFlag = true;
}
```

___

如果 odom 和 imu 的预处理都成功，那么进行下一步，检查激光点是否合规，然后进行运动畸变矫正，保存激光点。

```
void projectPointCloud()
{
    int cloudSize = (int)laserCloudIn->points.size();
    // range image projection
    // 遍历当前帧激光点云
    for (int i = 0; i < cloudSize; ++i)
    {
        // pcl格式
        PointType thisPoint;
        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;
        thisPoint.intensity = laserCloudIn->points[i].intensity;

        int rowIdn = laserCloudIn->points[i].ring;
        // 扫描线检查
        if (rowIdn < 0 || rowIdn >= N_SCAN)
            continue;
        // 扫描线如果有降采样，跳过采样的扫描线这里要跳过
        if (rowIdn % downsampleRate != 0)
            continue;

        float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
        // 水平扫描角度步长，例如一周扫描1800次，则两次扫描间隔角度0.2°
        static float ang_res_x = 360.0/float(Horizon_SCAN);
        int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;
        // 扫描线检查
        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;

        float range = pointDistance(thisPoint);
        // 不考虑一米内的扫描
        if (range < 1.0)
            continue;
        // 如果不为 FLT_MAX，说明已经存过该点，不再处理
        if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
            continue;

        // for the amsterdam dataset
        // if (range < 6.0 && rowIdn <= 7 && (columnIdn >= 1600 || columnIdn <= 200))
        //     continue;
        // if (thisPoint.z < -2.0)
        //     continue;

        rangeMat.at<float>(rowIdn, columnIdn) = range;
        
         // 激光运动畸变校正
        // 利用当前帧起止时刻之间的imu数据计算旋转增量，imu里程计数据计算平移增量，进而将每一时刻激光点位置变换到第一个激光点坐标系下，进行运动补偿
        thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time); // Velodyne
        // thisPoint = deskewPoint(&thisPoint, (float)laserCloudIn->points[i].t / 1000000000.0); // Ouster
        // 转换成一维索引，存校正之后的激光点
        int index = columnIdn  + rowIdn * Horizon_SCAN;
        fullCloud->points[index] = thisPoint;
    }
}
```

其中最主要的函数是对点云中的点进行去畸变。

```
PointType deskewPoint(PointType *point, double relTime)
{
    // 如果不需要去畸变，或者没有 imu 信息
    if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
        return *point;
    // relTime 是当前激光点相对于激光帧起始时刻的时间，pointTime 则是当前激光点的时间戳
    double pointTime = timeScanCur + relTime;

    // 在当前激光帧起止时间范围内，计算某一时刻的旋转（相对于起始时刻的旋转增量）
    float rotXCur, rotYCur, rotZCur;
    // 该函数用来计算当前三轴的旋转
    // 首先寻找 imu 队列中首个时间晚于当前点时间的 imu 信息
    // 如果 imu 队列中时间都比当前点的早，或者 imu 队列中只有一个元素
    // 则将 imu 队列中最后一个元素的三轴旋转赋值给当前点
    // 如果找到了，则用该 imu 数据和上一时刻 imu 数据
    // 用线性插值法，算得该点时间下的 imu 三轴旋转，赋值给当前点
    findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

    // 在当前激光帧起止时间范围内，计算某一时刻的平移（相对于起始时刻的平移增量）
    float posXCur, posYCur, posZCur;
    // 当前点的三轴位置均赋值为 0
    findPosition(relTime, &posXCur, &posYCur, &posZCur);

    // 如果是起始点，则将 transStartInverse 赋值为当前点 tf 的逆
    // 乘以下面的 transFinal 即当前 tf，为单位矩阵
    if (firstPointFlag == true)
    {
        transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
        firstPointFlag = false;
    }

    // transform points to start
    // 当前时刻激光点与第一个激光点的位姿变换
    Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
    Eigen::Affine3f transBt = transStartInverse * transFinal;

    // tf 矩阵点乘 [x,y,z,1]^T，当前激光点在第一个激光点坐标系下的坐标
    PointType newPoint;
    newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
    newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
    newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
    newPoint.intensity = point->intensity;

    return newPoint;
}
```

___

第四步就是提取分割后的点云，给雷达里程计使用，第五步是发布该点云，都是常规方法不做赘述。

```
void cloudExtraction()
{
    // 有效激光点数量
    int count = 0;
    // extract segmented cloud for lidar odometry
    // 遍历所有激光点
    for (int i = 0; i < N_SCAN; ++i)
    {
        // 记录每根扫描线起始第 5 个激光点在一维数组中的索引
        cloudInfo.startRingIndex[i] = count - 1 + 5;

        for (int j = 0; j < Horizon_SCAN; ++j)
        {
            // 有效激光点
            if (rangeMat.at<float>(i,j) != FLT_MAX)
            {
                // mark the points' column index for marking occlusion later
                // 记录激光点对应的 Horizon_SCAN 方向上的索引
                cloudInfo.pointColInd[count] = j;
                // save range info
                // 激光点距离
                cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                // save extracted cloud
                // 加入有效激光点
                extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                // size of extracted cloud
                ++count;
            }
        }
        // 记录每根扫描线倒数第5个激光点在一维数组中的索引
        cloudInfo.endRingIndex[i] = count - 1 - 5;
    }
}

void publishClouds()
{
    // 发布当前帧校正后点云，有效点
    cloudInfo.header = cloudHeader;
    cloudInfo.cloud_deskewed  = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, "base_link");
    pubLaserCloudInfo.publish(cloudInfo);
}
```

___

第六步重置参数和成员函数分配内存在一起讲，大致就是根据传入的点云信息分配内存，而重置参数是重置输入和发布点云数组，数据清零，并且将符号位重置，具体如下。

```
void allocateMemory()
{
    laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
    fullCloud.reset(new pcl::PointCloud<PointType>());
    extractedCloud.reset(new pcl::PointCloud<PointType>());

    fullCloud->points.resize(N_SCAN*Horizon_SCAN);

    cloudInfo.startRingIndex.assign(N_SCAN, 0);
    cloudInfo.endRingIndex.assign(N_SCAN, 0);

    cloudInfo.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
    cloudInfo.pointRange.assign(N_SCAN*Horizon_SCAN, 0);

    resetParameters();
}

// 重置参数，接收每帧lidar数据都要重置这些参数
void resetParameters()
{
    laserCloudIn->clear();
    extractedCloud->clear();
    // reset range matrix for range image projection
    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

    imuPointerCur = 0;
    firstPointFlag = true;
    odomDeskewFlag = false;

    for (int i = 0; i < queueLength; ++i)
    {
        imuTime[i] = 0;
        imuRotX[i] = 0;
        imuRotY[i] = 0;
        imuRotZ[i] = 0;
    }
}
```

___

## 目录