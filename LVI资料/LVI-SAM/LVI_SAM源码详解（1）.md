## 雷达里程计

_**imuPreintegration**_

定义了一个 ![[公式]](https://www.zhihu.com/equation?tex=IMUPreintegration) 类，由 ![[公式]](https://www.zhihu.com/equation?tex=ParamServer) 公有继承而来。

成员变量大致如下所示。

```
// ROS 订阅与发布
ros::Subscriber subImu;
ros::Subscriber subOdometry;
ros::Publisher pubImuOdometry;
ros::Publisher pubImuPath;

// map -> odom
tf::Transform map_to_odom;
tf::TransformBroadcaster tfMap2Odom;
// odom -> base_link
tf::TransformBroadcaster tfOdom2BaseLink;

bool systemInitialized = false;
// 噪声协方差
gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
gtsam::Vector noiseModelBetweenBias;

// imu 预积分
gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;
// imu 数据队列
std::deque<sensor_msgs::Imu> imuQueOpt;
std::deque<sensor_msgs::Imu> imuQueImu;

// imu 因子图优化过程中的状态变量
gtsam::Pose3 prevPose_;
gtsam::Vector3 prevVel_;
// imu 状态
gtsam::NavState prevState_;
gtsam::imuBias::ConstantBias prevBias_;

gtsam::NavState prevStateOdom;
gtsam::imuBias::ConstantBias prevBiasOdom;

bool doneFirstOpt = false;
double lastImuT_imu = -1;
double lastImuT_opt = -1;

gtsam::ISAM2 optimizer;
gtsam::NonlinearFactorGraph graphFactors;
gtsam::Values graphValues;

const double delta_t = 0;

int key = 1;
int imuPreintegrationResetId = 0;
// imu 到 lidar 的 tf
gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));;
```

___

首先是构造函数，对一些 subscriber 和 publisher 进行了定义，其中包括处理 imu 和里程计的回调函数，用 gtsam 定义了一些方差和噪声参数。

```
IMUPreintegration()
{
    // 订阅imu原始数据，用下面因子图优化的结果，施加两帧之间的imu预计分量，预测每一时刻（imu频率）的imu里程计
    subImu      = nh.subscribe<sensor_msgs::Imu>  (imuTopic, 2000, &IMUPreintegration::imuHandler, this, ros::TransportHints().tcpNoDelay());
    // 订阅激光里程计，来自mapOptimization，用两帧之间的imu预计分量构建因子图，优化当前帧位姿（这个位姿仅用于更新每时刻的imu里程计，以及下一次因子图优化）
    subOdometry = nh.subscribe<nav_msgs::Odometry>(PROJECT_NAME + "/lidar/mapping/odometry", 5, &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());
    // 发布 imu 里程计
    pubImuOdometry = nh.advertise<nav_msgs::Odometry> ("odometry/imu", 2000);
    pubImuPath     = nh.advertise<nav_msgs::Path>     (PROJECT_NAME + "/lidar/imu/path", 1);

    map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
    // imu预积分的噪声协方差
    boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
    p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
    p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
    p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
    gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias
    // 噪声先验
    priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
    priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e2); // m/s
    priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
    // 激光里程计scan-to-map优化过程中发生退化，则选择一个较大的协方差
    correctionNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2); // meter
    noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
    // imu预积分器，用于预测每一时刻（imu频率）的imu里程计（转到lidar系了，与激光里程计同一个系）
    imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
    // imu预积分器，用于因子图优化
    imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization        
}
```

___

首先是 imu 的回调函数。

根据上一次的 ![[公式]](https://www.zhihu.com/equation?tex=odom) 和偏差用 gtsam 来估计当下的 ![[公式]](https://www.zhihu.com/equation?tex=odom) 作为 imu 的位姿，通过静态 TF 变换求得雷达的位姿，将其逐个发布。

![](https://pic3.zhimg.com/v2-0fad4124eb0919a01e250fd36a34a5c2_b.jpg)

如图所示，由于 IMU 的频率比雷达高得多，因此在下一帧点云即时间戳到来之前，会有大量的IMU数据读入，如上图中紫色线条所示，则可以利用这段时间间隔内的IMU测量量进行预积分操作。

用上一帧激光里程计，加上从上一帧对应时刻到当前时刻 imu 预计分量，得到当前状态，也就是 imu 里程计，再将其转到 lidar 坐标系发布。

```
void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
{
    // imu 原始测量数据转换到 lidar 系，加速度、角速度、RPY
    // imu 的速度并没有在该处处理，而是在下面处理了积分后的 imu 位置
    sensor_msgs::Imu thisImu = imuConverter(*imuMsg);
    // publish static tf
    // map 和 odom 坐标系完全重合，发布 map 坐标系，实际中并没有帮助
    tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, thisImu.header.stamp, "map", "odom"));
    
    // 两个双端队列分别是优化前后的 imu 数据，将当前帧 imu 数据添加到队列
    imuQueOpt.push_back(thisImu);
    imuQueImu.push_back(thisImu);

    // 如果还没有执行过 odom 优化，或者上一次优化失败导致系统重置
    // 则等待一次 odom 的优化再继续函数流程
    if (doneFirstOpt == false)
        return;
    
    // 如果首次优化，则定义初始时间间隔为 1/500 秒，否则是与上一帧作差
    double imuTime = ROS_TIME(&thisImu);
    double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
    lastImuT_imu = imuTime;

    // integrate this single imu message
    // imu预积分器添加一帧imu数据，起始时刻是上一帧激光里程计时刻
    imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                            gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);

    // predict odometry
    // 用上一帧激光里程计时刻对应的状态、偏置，施加从该时刻开始到当前时刻的imu预计分量，得到当前时刻的状态
    gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

    // publish odometry
    // 发布 imu 里程计
    nav_msgs::Odometry odometry;
    odometry.header.stamp = thisImu.header.stamp;
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "odom_imu";

    // transform imu pose to ldiar
    // 通过静态 tf 变换到 lidar 坐标系
    // x，y，z都是反向的
    gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
    gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

    odometry.pose.pose.position.x = lidarPose.translation().x();
    odometry.pose.pose.position.y = lidarPose.translation().y();
    odometry.pose.pose.position.z = lidarPose.translation().z();
    odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
    odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
    odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
    odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
    
    odometry.twist.twist.linear.x = currentState.velocity().x();
    odometry.twist.twist.linear.y = currentState.velocity().y();
    odometry.twist.twist.linear.z = currentState.velocity().z();
    odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
    odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
    odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
    // information for VINS initialization
    // 这里的协方差并不是指可信度，而是方便给 VINS 传参
    odometry.pose.covariance[0] = double(imuPreintegrationResetId);
    odometry.pose.covariance[1] = prevBiasOdom.accelerometer().x();
    odometry.pose.covariance[2] = prevBiasOdom.accelerometer().y();
    odometry.pose.covariance[3] = prevBiasOdom.accelerometer().z();
    odometry.pose.covariance[4] = prevBiasOdom.gyroscope().x();
    odometry.pose.covariance[5] = prevBiasOdom.gyroscope().y();
    odometry.pose.covariance[6] = prevBiasOdom.gyroscope().z();
    odometry.pose.covariance[7] = imuGravity;
    pubImuOdometry.publish(odometry);

    // publish imu path
    // 发布 imu 轨迹，但只保存 3s
    static nav_msgs::Path imuPath;
    static double last_path_time = -1;
    if (imuTime - last_path_time > 0.1)
    {
        last_path_time = imuTime;
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = thisImu.header.stamp;
        pose_stamped.header.frame_id = "odom";
        pose_stamped.pose = odometry.pose.pose;
        imuPath.poses.push_back(pose_stamped);
        while(!imuPath.poses.empty() && abs(imuPath.poses.front().header.stamp.toSec() - imuPath.poses.back().header.stamp.toSec()) > 3.0)
            imuPath.poses.erase(imuPath.poses.begin());
        if (pubImuPath.getNumSubscribers() != 0)
        {
            imuPath.header.stamp = thisImu.header.stamp;
            imuPath.header.frame_id = "odom";
            pubImuPath.publish(imuPath);
        }
    }

    // publish transformation
    // 发布 odom 到 base_link 的变换，其实也就是到 imu 的变换
    // 因为作者采用的模型的 base_link 就是 imu
    // 可以考虑改为发布 odom 到 imu 的 TF
    tf::Transform tCur;
    tf::poseMsgToTF(odometry.pose.pose, tCur);
    tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, thisImu.header.stamp, "odom", "base_link");
    tfOdom2BaseLink.sendTransform(odom_2_baselink);
}
```

___

然后是里程计回调函数。

每隔100帧激光里程计，重置ISAM2优化器，添加里程计、速度、偏置先验因子，执行优化。

计算前一帧激光里程计与当前帧激光里程计之间的imu预积分量，用前一帧状态施加预积分量得到当前帧初始状态估计，添加来自mapOptimization的当前帧位姿，进行因子图优化，更新当前帧状态。

优化之后，执行重传播。获得 imu 真实的 bias，用来计算当前时刻之后的 imu 预积分。

```
void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    // 当前帧激光里程计时间戳
    double currentCorrectionTime = ROS_TIME(odomMsg);

    // make sure we have imu data to integrate
    if (imuQueOpt.empty())
        return;
    // 当前帧激光位姿，来自scan-to-map匹配、因子图优化后的位姿
    float p_x = odomMsg->pose.pose.position.x;
    float p_y = odomMsg->pose.pose.position.y;
    float p_z = odomMsg->pose.pose.position.z;
    float r_x = odomMsg->pose.pose.orientation.x;
    float r_y = odomMsg->pose.pose.orientation.y;
    float r_z = odomMsg->pose.pose.orientation.z;
    float r_w = odomMsg->pose.pose.orientation.w;
    int currentResetId = round(odomMsg->pose.covariance[0]);
    gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

    // correction pose jumped, reset imu pre-integration
    // 当前里程计位姿 id 保存在协方差的第一项，如果和当前 imu 位姿 id 不同，说明有跳变，重置优化
    if (currentResetId != imuPreintegrationResetId)
    {
        resetParams();
        imuPreintegrationResetId = currentResetId;
        return;
    }


    // 0. initialize system
    // 系统初始化，第一帧
    if (systemInitialized == false)
    {
        // 重置优化参数，也就是重置 ISAM2 优化器
        // 将 relinearizeThreshold 赋值为 0.1，将relinearizeSkip 赋值为 1
        // new 分配出新的 graphFactors 和 graphValues
        resetOptimization();

        // pop old IMU message
        // 从imu优化队列中删除当前帧激光里程计时刻之前的imu数据
        while (!imuQueOpt.empty())
        {
            if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
            {
                lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                imuQueOpt.pop_front();
            }
            else
                break;
        }
        // initial pose
        // 添加里程计位姿先验因子
        prevPose_ = lidarPose.compose(lidar2Imu);    // compose 为矩阵点乘
        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
        graphFactors.add(priorPose);
        // initial velocity
        // 添加速度先验因子
        prevVel_ = gtsam::Vector3(0, 0, 0);
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
        graphFactors.add(priorVel);
        // initial bias
        // 添加imu偏置先验因子
        prevBias_ = gtsam::imuBias::ConstantBias();
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
        graphFactors.add(priorBias);
        // add values
        // 变量节点赋初值
        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);
        // optimize once
        // 假定起始为 0 速度，进行一次优化，用于刚开始的 scan-matching
        // 对于起始速度小于 10 m/s，角速度小于 180°/s，效果都非常好
        // 但是这总归是基于 0 起始速度估计的，起始估计并不是实际情况，因此清除图优化中内容
        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();

        imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        // 有一个帧了，系统初始化成功
        key = 1;
        systemInitialized = true;
        return;
    }


    // reset graph for speed
    // 每隔100帧激光里程计，重置ISAM2优化器，保证优化效率，防止内存溢出
    if (key == 100)
    {
        // get updated noise before reset
        // 根据最近的下标即 99，获得前一帧的位姿、速度、偏置噪声模型
        gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
        gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
        gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
        // reset graph
        // 重置ISAM2优化器
        resetOptimization();
        // add pose
        // 因为噪声值不同，实际算出的 bias 和标定出来的不同，需要更新图
        // 添加位姿先验因子，用前一帧的值初始化
        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
        graphFactors.add(priorPose);
        // add velocity
        // 添加速度先验因子，用前一帧的值初始化
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
        graphFactors.add(priorVel);
        // add bias
        // 添加偏置先验因子，用前一帧的值初始化
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
        graphFactors.add(priorBias);
        // add values
        // 变量节点赋初值，用前一帧的值初始化
        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);
        // optimize once
        // 更新一次优化器，然后将图清掉
        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();
        // 只剩一个帧了
        key = 1;
    }


    // 1. integrate imu data and optimize
    // 如论文中所述，初始化完成后就可以估计 IMU 偏差，机器人位姿，速度
    // 计算前一帧与当前帧之间的imu预积分量，用前一帧状态施加预积分量得到当前帧初始状态估计，
    // 添加来自mapOptimization的当前帧位姿，进行因子图优化，更新当前帧状态
    while (!imuQueOpt.empty())
    {
        // pop and integrate imu data that is between two optimizations
        // 提取前一帧与当前帧之间的imu数据，计算预积分
        sensor_msgs::Imu *thisImu = &imuQueOpt.front();
        double imuTime = ROS_TIME(thisImu);
        // delta_t = 0
        if (imuTime < currentCorrectionTime - delta_t)
        {
            // 时间差初始值为 1/500 s，否则为与上一帧的时间差
            double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
            // 此处积分只用到了线加速度和角速度
            imuIntegratorOpt_->integrateMeasurement(
                    gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                    gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
            
            lastImuT_opt = imuTime;
            // 从队列中删除已经处理的imu数据
            imuQueOpt.pop_front();
        }
        else
            break;
    }
    // add imu factor to graph
    // 加imu预积分因子
    const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
    // 参数：前一帧位姿，前一帧速度，当前帧位姿，当前帧速度，前一帧偏置，预计分量
    gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
    graphFactors.add(imu_factor);
    // add imu bias between factor
    // 添加imu偏置因子，前一帧偏置，当前帧偏置，观测值，噪声协方差；deltaTij()是积分段的时间
    graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                        gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
    // add pose factor
    // 添加位姿因子
    gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
    gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, correctionNoise);
    graphFactors.add(pose_factor);
    // insert predicted values
    // 用前一帧的状态、偏置，施加 imu 预计分量，得到当前帧的状态
    gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
    graphValues.insert(X(key), propState_.pose());
    graphValues.insert(V(key), propState_.v());
    graphValues.insert(B(key), prevBias_);
    // optimize
    optimizer.update(graphFactors, graphValues);
    optimizer.update();
    graphFactors.resize(0);
    graphValues.clear();
    // Overwrite the beginning of the preintegration for the next step.
    // 优化结果
    gtsam::Values result = optimizer.calculateEstimate();
    // 更新当前帧位姿、速度
    prevPose_  = result.at<gtsam::Pose3>(X(key));
    prevVel_   = result.at<gtsam::Vector3>(V(key));
    // 更新当前帧状态
    prevState_ = gtsam::NavState(prevPose_, prevVel_);
    // 更新当前帧imu偏置
    prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
    // Reset the optimization preintegration object.
    // 重置预积分器，设置新的偏置，这样下一帧激光里程计进来的时候，预积分量就是两帧之间的增量
    imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
    // check optimization
    // 检测是否有失败：
    // 1. imu 的速度大于 30，则速度过大
    // 2. imu 的偏差大于 0.1，则偏差
    // 如果有上述失败，则重置参数
    if (failureDetection(prevVel_, prevBias_))
    {
        resetParams();
        return;
    }


    // 2. after optiization, re-propagate imu odometry preintegration
    // 为了维持实时性 imuIntegrateImu 就得在每次 odom 触发优化后立刻获取最新的 bias
    // 同时对imu测量值 imuQueImu 执行 bias 改变的状态重传播处理, 这样可以最大限度的保证实时性和准确性
    prevStateOdom = prevState_;
    prevBiasOdom  = prevBias_;
    // first pop imu message older than current correction data
    double lastImuQT = -1;
    // 从 imu 队列中删除当前激光里程计时刻之前的 imu 数据
    while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
    {
        lastImuQT = ROS_TIME(&imuQueImu.front());
        imuQueImu.pop_front();
    }
    // repropogate
    // 对剩余的imu数据计算预积分
    if (!imuQueImu.empty())
    {
        // reset bias use the newly optimized bias
        // 重置预积分器和最新的偏置，使用最新的偏差更新 bias 值
        imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
        // integrate imu message from the beginning of this optimization
        // 由于更新了偏差值，需要重新预积分
        for (int i = 0; i < (int)imuQueImu.size(); ++i)
        {
            sensor_msgs::Imu *thisImu = &imuQueImu[i];
            double imuTime = ROS_TIME(thisImu);
            double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);

            imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                    gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
            lastImuQT = imuTime;
        }
    }

    ++key;
    doneFirstOpt = true;
}
```

___

## 目录