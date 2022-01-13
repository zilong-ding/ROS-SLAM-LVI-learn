构造函数中下一步是 gps 的回调函数，由于我身边设备暂时未用到，下次再作讨论，待更新。

___

接着是闭环检测回调函数。

1、闭环 scan-to-map，icp 优化位姿。  
1) 在历史关键帧中查找与当前关键帧距离最近的关键帧集合，选择时间相隔较远的一帧作为候选闭环帧；  
2) 提取当前关键帧特征点集合，降采样；提取闭环匹配关键帧前后相邻若干帧的关键帧特征点集合，降采样；  
3) 执行 scan-to-map 优化，调用 icp 方法，得到优化后位姿，构造闭环因子需要的数据，在因子图优化中一并加入更新位姿。  
2、rviz 展示闭环边。

```
void loopHandler(const std_msgs::Float64MultiArray::ConstPtr& loopMsg)
{
    // control loop closure frequency
    // 控制闭环检测频率，减轻计算负载
    static double last_loop_closure_time = -1;
    {
        // std::lock_guard<std::mutex> lock(mtx);
        if (timeLaserInfoCur - last_loop_closure_time < 5.0)
            return;
        else
            last_loop_closure_time = timeLaserInfoCur;
    }
    // 闭环scan-to-map，icp优化位姿
    performLoopClosure(*loopMsg);
}

void performLoopClosure(const std_msgs::Float64MultiArray& loopMsg)
{
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>());
    {
        std::lock_guard<std::mutex> lock(mtx);
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    }

    // get lidar keyframe id
    int key_cur = -1; // latest lidar keyframe id
    int key_pre = -1; // previous lidar keyframe id
    {
        loopFindKey(loopMsg, copy_cloudKeyPoses6D, key_cur, key_pre);
        // 前后最近的关键帧下标不合法
        if (key_cur == -1 || key_pre == -1 || key_cur == key_pre)// || abs(key_cur - key_pre) < 25)
            return;
    }

    // check if loop added before
    // 检查该帧是否已经被回环过，因为图像回环，很多图像可能指向同一个关键帧
    {
        // if image loop closure comes at high frequency, many image loop may point to the same key_cur
        auto it = loopIndexContainer.find(key_cur);
        if (it != loopIndexContainer.end())
            return;
    }
    
    // get lidar keyframe cloud
    // 提取
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
    {
        // 提取当前关键帧特征点集合，降采样
        loopFindNearKeyframes(copy_cloudKeyPoses6D, cureKeyframeCloud, key_cur, 0);
        // 提取闭环匹配关键帧前后相邻若干帧的关键帧特征点集合，降采样
        loopFindNearKeyframes(copy_cloudKeyPoses6D, prevKeyframeCloud, key_pre, historyKeyframeSearchNum);
        // 如果特征点较少，返回
        if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
            return;
        // 发布闭环匹配关键帧局部map
        if (pubHistoryKeyFrames.getNumSubscribers() != 0)
            publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, "odom");
    }

    // get keyframe pose
    // 求前后两帧位置之间的 tf
    Eigen::Affine3f pose_cur;
    Eigen::Affine3f pose_pre;
    Eigen::Affine3f pose_diff_t; // serves as initial guess
    {
        pose_cur = pclPointToAffine3f(copy_cloudKeyPoses6D->points[key_cur]);
        pose_pre = pclPointToAffine3f(copy_cloudKeyPoses6D->points[key_pre]);

        Eigen::Vector3f t_diff;
        t_diff.x() = - (pose_cur.translation().x() - pose_pre.translation().x());
        t_diff.y() = - (pose_cur.translation().y() - pose_pre.translation().y());
        t_diff.z() = - (pose_cur.translation().z() - pose_pre.translation().z());
        if (t_diff.norm() < historyKeyframeSearchRadius)
            t_diff.setZero();
        pose_diff_t = pcl::getTransformation(t_diff.x(), t_diff.y(), t_diff.z(), 0, 0, 0);
    }

    // transform and rotate cloud for matching
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    // pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;
    // ICP参数设置
    icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2);
    icp.setMaximumIterations(100);
    icp.setRANSACIterations(0);
    icp.setTransformationEpsilon(1e-3);
    icp.setEuclideanFitnessEpsilon(1e-3);

    // initial guess cloud
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud_new(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*cureKeyframeCloud, *cureKeyframeCloud_new, pose_diff_t);

    // match using icp
    // scan-to-map，调用icp匹配
    icp.setInputSource(cureKeyframeCloud_new);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    if (pubIcpKeyFrames.getNumSubscribers() != 0)
    {
        pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*cureKeyframeCloud_new, *closed_cloud, icp.getFinalTransformation());
        publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, "odom");
    }

    // add graph factor
    // 已收敛，或者匹配分数足够好
    if (icp.getFitnessScore() < historyKeyframeFitnessScore && icp.hasConverged() == true)
    {
        // get gtsam pose
        gtsam::Pose3 poseFrom = affine3fTogtsamPose3(Eigen::Affine3f(icp.getFinalTransformation()) * pose_diff_t * pose_cur);
        gtsam::Pose3 poseTo   = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[key_pre]);
        // get noise
        float noise = icp.getFitnessScore();
        gtsam::Vector Vector6(6);
        Vector6 << noise, noise, noise, noise, noise, noise;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);
        
        // save pose constraint
        // 保存当前回环组合的下标、位姿变换、噪声
        mtx.lock();
        loopIndexQueue.push_back(make_pair(key_cur, key_pre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        mtx.unlock();
        // add loop pair to container
        // 在容器中加入两个下标关联，以后可以防止重复处理
        loopIndexContainer[key_cur] = key_pre;
    }

    // visualize loop constraints
    // rviz展示闭环边
    if (!loopIndexContainer.empty())
    {
        visualization_msgs::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = "odom";
        markerNode.header.stamp = timeLaserInfoStamp;
        markerNode.action = visualization_msgs::Marker::ADD;
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
        markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
        markerNode.color.a = 1;
        // loop edges
        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = "odom";
        markerEdge.header.stamp = timeLaserInfoStamp;
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
        {
            int key_cur = it->first;
            int key_pre = it->second;
            geometry_msgs::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge.publish(markerArray);
    }
}

void loopFindKey(const std_msgs::Float64MultiArray& loopMsg, 
                     const pcl::PointCloud<PointTypePose>::Ptr& copy_cloudKeyPoses6D,
                     int& key_cur, int& key_pre)
{
    // 对 loopMsg 可能存在的赋值错误的情况
    if (loopMsg.data.size() != 2)
        return;

    // 提取当前和上一帧的回环时间戳
    double loop_time_cur = loopMsg.data[0];
    double loop_time_pre = loopMsg.data[1];

    // historyKeyframeSearchTimeDiff 默认为 30 秒
    if (abs(loop_time_cur - loop_time_pre) < historyKeyframeSearchTimeDiff)
        return;

    int cloudSize = copy_cloudKeyPoses6D->size();
    if (cloudSize < 2)
        return;

    // latest key
    key_cur = cloudSize - 1;
    // 找到第一个大于 loop_time_cur 的时间戳下标
    for (int i = cloudSize - 1; i >= 0; --i)
    {
        if (copy_cloudKeyPoses6D->points[i].time > loop_time_cur)
            key_cur = round(copy_cloudKeyPoses6D->points[i].intensity);
        else
            break;
    }

    // previous key
    key_pre = 0;
    // 找到第一个小于 loop_time_cur 的时间戳下标
    for (int i = 0; i < cloudSize; ++i)
    {
        if (copy_cloudKeyPoses6D->points[i].time < loop_time_pre)
            key_pre = round(copy_cloudKeyPoses6D->points[i].intensity);
        else
            break;
    }
}
```

___

接着是在 main 函数中建立的闭环检测线程：

```
std::thread loopDetectionthread(&mapOptimization::loopClosureThread, &MO);
```

不断利用 kd 树在全局关键帧内搜索，当前关键帧前后两个最近的关键帧，调用回环函数。

```
void loopClosureThread()
{
    if (loopClosureEnableFlag == false)
        return;

    ros::Rate rate(0.5);
    while (ros::ok())
    {
        rate.sleep();
        performLoopClosureDetection();
    }
}

void performLoopClosureDetection()
{
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;

    int key_cur = -1;
    int key_pre = -1;

    // 当前关键帧索引，候选闭环匹配帧索引
    double loop_time_cur = -1;
    double loop_time_pre = -1;

    // find latest key and time
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (cloudKeyPoses3D->empty())
            return;
        
        kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
        kdtreeHistoryKeyPoses->radiusSearch(cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

        key_cur = cloudKeyPoses3D->size() - 1;
        loop_time_cur = cloudKeyPoses6D->points[key_cur].time;
    }

    // find previous key and time
    {
        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            if (abs(cloudKeyPoses6D->points[id].time - loop_time_cur) > historyKeyframeSearchTimeDiff)
            {
                key_pre = id;
                loop_time_pre = cloudKeyPoses6D->points[key_pre].time;
                break;
            }
        }
    }

    if (key_cur == -1 || key_pre == -1 || key_pre == key_cur ||
        loop_time_cur < 0 || loop_time_pre < 0)
        return;

    std_msgs::Float64MultiArray match_msg;
    match_msg.data.push_back(loop_time_cur);
    match_msg.data.push_back(loop_time_pre);
    performLoopClosure(match_msg);
}
```

___

main 函数中还创建了一个功能性的进程。

```
std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);
```

保存 pcd 和发布全局点云。

```
void visualizeGlobalMapThread()
{
    ros::Rate rate(0.2);
    while (ros::ok()){
        rate.sleep();
        publishGlobalMap();
    }

    // 该变量在 params_lidar.yaml 中设置
    if (savePCD == false)
        return;

    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files ..." << endl;
    // create directory and remove old files;
    savePCDDirectory = std::getenv("HOME") + savePCDDirectory;
    int unused = system((std::string("exec rm -r ") + savePCDDirectory).c_str());
    unused = system((std::string("mkdir ") + savePCDDirectory).c_str()); ++unused;
    // save key frame transformations
    pcl::io::savePCDFileASCII(savePCDDirectory + "trajectory.pcd", *cloudKeyPoses3D);
    pcl::io::savePCDFileASCII(savePCDDirectory + "transformations.pcd", *cloudKeyPoses6D);
    // extract global point cloud map        
    pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) 
    {
        // clip cloud
        // pcl::PointCloud<PointType>::Ptr cornerTemp(new pcl::PointCloud<PointType>());
        // pcl::PointCloud<PointType>::Ptr cornerTemp2(new pcl::PointCloud<PointType>());
        // *cornerTemp = *transformPointCloud(cornerCloudKeyFrames[i],  &cloudKeyPoses6D->points[i]);
        // for (int j = 0; j < (int)cornerTemp->size(); ++j)
        // {
        //     if (cornerTemp->points[j].z > cloudKeyPoses6D->points[i].z && cornerTemp->points[j].z < cloudKeyPoses6D->points[i].z + 5)
        //         cornerTemp2->push_back(cornerTemp->points[j]);
        // }
        // pcl::PointCloud<PointType>::Ptr surfTemp(new pcl::PointCloud<PointType>());
        // pcl::PointCloud<PointType>::Ptr surfTemp2(new pcl::PointCloud<PointType>());
        // *surfTemp = *transformPointCloud(surfCloudKeyFrames[i],  &cloudKeyPoses6D->points[i]);
        // for (int j = 0; j < (int)surfTemp->size(); ++j)
        // {
        //     if (surfTemp->points[j].z > cloudKeyPoses6D->points[i].z && surfTemp->points[j].z < cloudKeyPoses6D->points[i].z + 5)
        //         surfTemp2->push_back(surfTemp->points[j]);
        // }
        // *globalCornerCloud += *cornerTemp2;
        // *globalSurfCloud   += *surfTemp2;

        // origin cloud
        // 将线和面特征点转到全局坐标系下
        *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i],  &cloudKeyPoses6D->points[i]);
        *globalSurfCloud   += *transformPointCloud(surfCloudKeyFrames[i],    &cloudKeyPoses6D->points[i]);
        cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
    }
    // down-sample and save corner cloud
    // 下采样，保存边特征点
    downSizeFilterCorner.setInputCloud(globalCornerCloud);
    pcl::io::savePCDFileASCII(savePCDDirectory + "cloudCorner.pcd", *globalCornerCloud);
    // down-sample and save surf cloud
    // 下采样，保存面特征点
    downSizeFilterSurf.setInputCloud(globalSurfCloud);
    pcl::io::savePCDFileASCII(savePCDDirectory + "cloudSurf.pcd", *globalSurfCloud);
    // down-sample and save global point cloud map
    // 下采样，保存全局特征点云
    *globalMapCloud += *globalCornerCloud;
    *globalMapCloud += *globalSurfCloud;
    downSizeFilterSurf.setInputCloud(globalMapCloud);
    pcl::io::savePCDFileASCII(savePCDDirectory + "cloudGlobal.pcd", *globalMapCloud);
    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files completed" << endl;
}

void publishGlobalMap()
{
    // 没有节点监听
    if (pubLaserCloudSurround.getNumSubscribers() == 0)
        return;

    // 没有特征点
    if (cloudKeyPoses3D->points.empty() == true)
        return;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
    pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

    // kd-tree to find near key frames to visualize
    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;
    // search near key frames to visualize
    mtx.lock();
    // 搜索最近的关键帧附近的点
    kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
    kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
    mtx.unlock();
    // 保存附近的点
    for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
        globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
    // downsample near selected key frames
    // 下采样附近的点
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
    downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);

    // extract visualized and downsampled key frames
    for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
        if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
            continue;
        int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
        // 将特征点位姿转到全局坐标系下
        *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
        *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
    }
    // downsample visualized points
    // 下采样要 rviz 可视化的点
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
    publishCloud(&pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, "odom");    
}
```

___

## 目录