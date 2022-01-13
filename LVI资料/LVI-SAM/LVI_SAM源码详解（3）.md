## 雷达里程计

## **_featureExtraction_**

定义了一个 ![[公式]](https://www.zhihu.com/equation?tex=FeatureExtraction) 类，由 ![[公式]](https://www.zhihu.com/equation?tex=ParamServer) 公有继承而来。

作者可能比较懒，把所有的成员都定义为公有的。

这里介绍一下 ![[公式]](https://www.zhihu.com/equation?tex=lvi%5C_sam%3A%3Acloud%5C_info) ，其定义为![[公式]](https://www.zhihu.com/equation?tex=%3A%3Alvi%5C_sam%3A%3Acloud%5C_info%5C_%3Cstd%3A%3Aallocator%3Cvoid%3E%3E) ， ![[公式]](https://www.zhihu.com/equation?tex=cloud%5C_info%5C_) 其中又包含点云信息，imu 和 odom 等信息的成员变量。

```
ros::Subscriber subLaserCloudInfo;

// 发布当前激光帧提取特征之后的点云信息
ros::Publisher pubLaserCloudInfo;
// 发布当前激光帧提取的角点点云
ros::Publisher pubCornerPoints;
// 发布当前激光帧提取的平面点点云
ros::Publisher pubSurfacePoints;

// 当前激光帧运动畸变校正后的有效点云
pcl::PointCloud<PointType>::Ptr extractedCloud;
// 当前激光帧角点点云集合
pcl::PointCloud<PointType>::Ptr cornerCloud;
// 当前激光帧平面点点云集合
pcl::PointCloud<PointType>::Ptr surfaceCloud;

pcl::VoxelGrid<PointType> downSizeFilter;

// 当前激光帧点云信息，包括的历史数据有：运动畸变校正，点云数据，初始位姿，姿态角，有效点云数据，角点点云，平面点点云等
lvi_sam::cloud_info cloudInfo;
std_msgs::Header cloudHeader;

// 当前激光帧点云的曲率
std::vector<smoothness_t> cloudSmoothness;
float *cloudCurvature;
// 特征提取标记，1表示遮挡、平行，或者已经进行特征提取的点，0表示还未进行特征提取处理
int *cloudNeighborPicked;
// 1 表示角点，-1 表示平面点
int *cloudLabel;
```

___

首先两个函数用于初始化，构造函数调用初始化值函数，主要对一些 subscriber，publisher和成员变量初始化。

```
FeatureExtraction()
{
    // 订阅当前激光帧运动畸变校正后的点云信息
    subLaserCloudInfo = nh.subscribe<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

    // 发布当前激光帧提取特征之后的点云信息
    pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/feature/cloud_info", 1);
    // 发布当前激光帧的角点点云
    pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_corner", 1);
    // 发布当前激光帧的面点点云
    pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_surface", 1);
    
    // 初始化
    initializationValue();
}

// 初始化
void initializationValue()
{
    cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

    downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

    extractedCloud.reset(new pcl::PointCloud<PointType>());
    cornerCloud.reset(new pcl::PointCloud<PointType>());
    surfaceCloud.reset(new pcl::PointCloud<PointType>());

    cloudCurvature = new float[N_SCAN*Horizon_SCAN];
    cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
    cloudLabel = new int[N_SCAN*Horizon_SCAN];
}
```

___

下面是核心函数群。

订阅当前激光帧运动畸变校正后的点云信息：  
1、计算当前激光帧点云中每个点的曲率。  
2、标记属于遮挡、平行两种情况的点，不做特征提取。  
3、点云角点、平面点特征提取：  
1) 遍历扫描线，每根扫描线扫描一周的点云划分为6段，针对每段提取20个角点、不限数量的平面点，加入角点集合、平面点集合；  
2) 认为非角点的点都是平面点，加入平面点云集合，最后降采样。  
4、发布角点、面点点云，发布带特征点云数据的当前激光帧点云信息。

```
void laserCloudInfoHandler(const lvi_sam::cloud_infoConstPtr& msgIn)
{
    cloudInfo = *msgIn; // new cloud info
    cloudHeader = msgIn->header; // new cloud header
    // 当前激光帧运动畸变校正后的有效点云
    pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud); // new cloud for extraction

    // 计算当前激光帧点云中每个点的曲率
    calculateSmoothness();

    // 标记属于遮挡、平行两种情况的点，不做特征提取
    markOccludedPoints();
    
    // 点云角点、平面点特征提取
    // 1、遍历扫描线，每根扫描线扫描一周的点云划分为6段，针对每段提取20个角点、不限数量的平面点，加入角点集合、平面点集合
    // 2、认为非角点的点都是平面点，加入平面点云集合，最后降采样
    extractFeatures();

    // 发布角点、面点点云，发布带特征点云数据的当前激光帧点云信息
    publishFeatureCloud();
}
```

___

代码很清晰易懂，首先保存当前点云，接着首先是 ![[公式]](https://www.zhihu.com/equation?tex=calculateSmoothness%28%29) 。

和往常的点云处理一样，前后十个点算差距范围，存入平滑度数组用于排序。

```
void calculateSmoothness()
{
    // 遍历当前激光帧运动畸变校正后的有效点云
    int cloudSize = extractedCloud->points.size();
    for (int i = 5; i < cloudSize - 5; i++)
    {
        // 用当前激光点前后5个点计算当前点的曲率，平坦位置处曲率较小，角点处曲率较大
        float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                        + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                        + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                        + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                        + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                        + cloudInfo.pointRange[i+5];            

        // 距离差值平方作为曲率
        cloudCurvature[i] = diffRange*diffRange;//diffX * diffX + diffY * diffY + diffZ * diffZ;

        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
        // cloudSmoothness for sorting
        // 存储该点曲率值、激光点一维索引
        cloudSmoothness[i].value = cloudCurvature[i];
        cloudSmoothness[i].ind = i;
    }
}
```

___

接着是 ![[公式]](https://www.zhihu.com/equation?tex=markOccludedPoints%28%29) 。

如果相邻点的下标差小于10，剔除距离更远的六个连续点，因为在雷达运动过程中这些点可能会被距离近的点遮蔽，因此不能作为特征点。如下图所示，Laser从左向右移动过程中，会出现 A 点被 B 点遮挡的情况。

![](https://pic4.zhimg.com/v2-431ffcc2f5b0845b20effc3326f36923_b.jpg)

如果当前点与前后点的距离差都较大，则视其为瑕点，因为入射角太小可能导致误差较大，去除当前点。

```
void markOccludedPoints()
{
    int cloudSize = extractedCloud->points.size();
    // mark occluded points and parallel beam points
   
    for (int i = 5; i < cloudSize - 6; ++i)
    {
        // occluded points
        // depth 为点到雷达的距离，也可以说是点的深度
        float depth1 = cloudInfo.pointRange[i];
        float depth2 = cloudInfo.pointRange[i+1];
        // 两个激光点之间的一维索引差值，如果在一条扫描线上，那么值为1；如果两个点之间有一些无效点被剔除了，可能会比1大，但不会特别大
        // 如果恰好前一个点在扫描一周的结束时刻，下一个点是另一条扫描线的起始时刻，那么值会很大
        int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));

        // 两个点在同一扫描线上，且距离相差大于0.3，认为存在遮挡关系（也就是这两个点不在同一平面上，如果在同一平面上，距离相差不会太大）
        // 远处的点会被遮挡，标记一下该点以及相邻的5个点，后面不再进行特征提取
        if (columnDiff < 10){
            // 10 pixel diff in range image
            if (depth1 - depth2 > 0.3){
                cloudNeighborPicked[i - 5] = 1;
                cloudNeighborPicked[i - 4] = 1;
                cloudNeighborPicked[i - 3] = 1;
                cloudNeighborPicked[i - 2] = 1;
                cloudNeighborPicked[i - 1] = 1;
                cloudNeighborPicked[i] = 1;
            }else if (depth2 - depth1 > 0.3){
                cloudNeighborPicked[i + 1] = 1;
                cloudNeighborPicked[i + 2] = 1;
                cloudNeighborPicked[i + 3] = 1;
                cloudNeighborPicked[i + 4] = 1;
                cloudNeighborPicked[i + 5] = 1;
                cloudNeighborPicked[i + 6] = 1;
            }
        }
        // parallel beam
        // 用前后相邻点判断当前点所在平面是否与激光束方向平行
        float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
        float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));

        // 平行则标记一下
        if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
            cloudNeighborPicked[i] = 1;
    }
}
```

___

最后是 ![[公式]](https://www.zhihu.com/equation?tex=extractFeatures%28%29) 。

遍历所有扫描线，每条线用线性插值分成六等分。用变量 ![[公式]](https://www.zhihu.com/equation?tex=cloudNeighborPicked%5B%5D) 记录访问过的点，防止重复访问。

首先用 ![[公式]](https://www.zhihu.com/equation?tex=edgeThreshold) 来标记出边特征点，然后用 ![[公式]](https://www.zhihu.com/equation?tex=surfThreshold) 来标记出面特征点，然后遍历保存面特征点，对其进行滤波。

```
void extractFeatures()
{
    cornerCloud->clear();
    surfaceCloud->clear();

    pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

    // 遍历扫描线
    for (int i = 0; i < N_SCAN; i++)
    {
        surfaceCloudScan->clear();
        // 每个 SCAN 分成六段进行处理，每段分开提取有限数量的特征，保证特征均匀分布
        for (int j = 0; j < 6; j++)
        {
            // 用线性插值对 SCAN 进行等分，取得 sp 和 ep，即 start point 和 end point
            // startRingIndex为扫描线起始第5个激光点在一维数组中的索引
            int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
            int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;
           
            if (sp >= ep)
                continue;

            // 按照曲率从小到大排序点云
            std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

            int largestPickedNum = 0;
            // 按照曲率从大到小遍历
            for (int k = ep; k >= sp; k--)
            {
                // 激光点的索引
                int ind = cloudSmoothness[k].ind;
                // 用 cloudNeighborPicked[ind] 防止重复访问
                // 当前激光点还未被处理，且曲率大于阈值，则认为是角点
                // 此处 edgeThreshold 默认值为 0.1，即距离大于 0.1
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                {
                    largestPickedNum++;
                    // 每段只取20个角点，如果单条扫描线扫描一周是1800个点，则划分6段，每段300个点，从中提取20个角点
                    if (largestPickedNum <= 20){
                        // 标记为角点
                        cloudLabel[ind] = 1;
                        // 加入角点点云
                        cornerCloud->push_back(extractedCloud->points[ind]);
                    } else {
                        break;
                    }

                    // 标记已被处理
                    cloudNeighborPicked[ind] = 1;
                    // 同一条扫描线上前5个点标记一下，不再处理，避免特征聚集
                    for (int l = 1; l <= 5; l++)
                    {
                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    // 同一条扫描线上前5个点标记一下，不再处理，避免特征聚集
                    for (int l = -1; l >= -5; l--)
                    {
                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            // 按照曲率从小到大遍历
            for (int k = sp; k <= ep; k++)
            {
                // 激光点的索引
                int ind = cloudSmoothness[k].ind;
                // 当前激光点还未被处理，且曲率小于阈值，则认为是平面点
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                {
                    // 标记为平面点
                    cloudLabel[ind] = -1;
                    // 标记已被处理
                    cloudNeighborPicked[ind] = 1;

                    for (int l = 1; l <= 5; l++) {
                        // 同一条扫描线上后 5 个点标记一下，不再处理，避免特征聚集
                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        // 同一条扫描线上前5个点标记一下，不再处理，避免特征聚集
                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                // 平面点和未被处理的点，都认为是平面点，加入平面点云集合
                if (cloudLabel[k] <= 0){
                    surfaceCloudScan->push_back(extractedCloud->points[k]);
                }
            }
        }
        
        // 平面点云降采样
        surfaceCloudScanDS->clear();
        downSizeFilter.setInputCloud(surfaceCloudScan);
        downSizeFilter.filter(*surfaceCloudScanDS);
        // 加入平面点云集合
        *surfaceCloud += *surfaceCloudScanDS;
    }
}
```

___

最后两函数，作用如名字所述，清理和发布点云。

```
void freeCloudInfoMemory()
{
    cloudInfo.startRingIndex.clear();
    cloudInfo.endRingIndex.clear();
    cloudInfo.pointColInd.clear();
    cloudInfo.pointRange.clear();
}

void publishFeatureCloud()
{
    // 清理
    freeCloudInfoMemory();
    // 发布角点、面点点云，用于rviz展示
    cloudInfo.cloud_corner  = publishCloud(&pubCornerPoints,  cornerCloud,  cloudHeader.stamp, lidarFrame);
    cloudInfo.cloud_surface = publishCloud(&pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
    // 发布当前激光帧点云信息，加入了角点、面点点云数据，发布给mapOptimization
    pubLaserCloudInfo.publish(cloudInfo);
}
```

___

## 目录