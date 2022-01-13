camera-IMU标定

原博客地址：https://blog.csdn.net/zhubaohua_bupt/article/details/80222321

## **1标定目的**：

我们进行camera-IMU的目的是为了得到IMU和相机坐标系的相对位姿矩阵T。

## **2标定工具**：

我们利用Kalibr标定工程对camera-IMU进行标定，链接[https://github.com/ethz-asl/kalibr/wiki/calibrating-the-vi-sensor](https://github.com/ethz-asl/kalibr/wiki/calibrating-the-vi-sensor).以下内容是在对在camera-IMU成功标定后的总结。

## 3**输入文件：**

以下内容默认为标定双目相机。和静态标定不同，本标定的输入文件包括

1 用带imu的相机拍摄的视频

2 视频中imu实时数据

3 imu参数和cam的内外参

4标定板信息

后面会详细的介绍一下输入文件以及制作方法。

           Kalibr工程有两种，一种是已经编译好的包，叫**CDE package**,另一种是未经编译的源文件。

           前者：安装简单，不需要依赖ROS，但是功能不全。

           后者：安装稍麻烦，但功能全，建议安装这种。

后者安装方法：

## **1安装 ROS**

## **2安装依赖项**

sudo apt-get install python-setuptools python-rosinstall ipython libeigen3-devlibboost-all-dev doxygen libopencv-dev ros-indigo-vision-opencvros-indigo-image-transport-plugins ros-indigo-cmake-modulespython-software-properties software-properties-common libpoco-devpython-matplotlib python-scipy python-git python-pip ipython libtbb-devlibblas-dev liblapack-dev python-catkin-tools libv4l-dev

sudo pip installpython-igraph --upgrade

**注意：pip未安装会报错**

## **3创建一个工作空间，这就是以后的工程根路径**

mkdir -p~/kalibr\_workspace/src  
**cd ~/kalibr\_workspace  
**source/opt/ros/indigo/setup.bash  
catkin init  
catkin config --extend /opt/ros/indigo  
catkin config --cmake-args -DCMAKE\_BUILD\_TYPE=Release

## **4下载工程代码并编译**

cd~/kalibr\_workspace/src  
git clone [https://github.com/ethz-asl/Kalibr.git](https://github.com/ethz-asl/Kalibr.git)

cd~/kalibr\_workspace  
catkin build -DCMAKE\_BUILD\_TYPE=Release -j4

**注意：如果你电脑内存不太够用，试着减少编译时开的线程数，把参数 –j4换成-j2或直接省略。**

## **5设置一下你刚才编译工程的环境变量**

source~/kalibr\_workspace/devel/setup.bash

**至此，安装部分结束，标定需要的所有工具都在本工程的devel/bin/目录下。**

**一个标定的例子：**

**它们分别是：包含图形和imu数据的bag文件、相机参数文件、IMU参数文件和标定板参数文件**。

## **1 bag 文件**

**<1>. bag 文件内容：**

**这是在连续时间获得的拍摄标定版的图像和IMU数据包，需要自己采集后再利用**[kalibr](https://github.com/ethz-asl/kalibr)**提供的一个工具去转化成.bag包。**

**.bag文件的具体内容是：**标定需要的图像以及相对应的imu数据。格式是：

+-- dataset-dir

 +--cam0

 │   +-- 1385030208726607500.png

 │   +--      ...

 │   \\-- 1385030212176607500.png

 +--cam1

 │   +-- 1385030208726607500.png

 │   +--      ...

 │   \\-- 1385030212176607500.png

 \\--imu0.csv

**IMU文件格式是**：19位时间戳（精确到ns），角速度，含重力的加速度

timestamp,omega\_x,omega\_y,omega\_z,alpha\_x,alpha\_y,alpha\_z

1385030208736607488,0.5,-0.2,-0.1,8.1,-1.9,-3.3

 ...

1386030208736607488,0.5,-0.1,-0.1,8.1,-1.9,-3.3

**<2>bag的制作工具：**

kalibr\_bagcreater--folder dataset-dir --output-bag awsome.bag

dataset-dir是数据输入路径：

其内文件结构应是这样:

/cam0/image\_raw

/cam1/image\_raw

/imu0

awsome.bag 是制作好的bag文件。输出默认在kalibr\_bagcreater同目录下。

## **2相机参数文件：**camchain.[yaml](https://so.csdn.net/so/search?q=yaml&spm=1001.2101.3001.7020)：

cam0:

 camera\_model:pinhole

 intrinsics:\[461.629, 460.152, 362.680, 246.049\]

 distortion\_model:radtan

 distortion\_coeffs: \[-0.27695497, 0.06712482, 0.00087538, 0.00011556\]

 timeshift\_cam\_imu: -8.121e-05

 rostopic:/cam0/image\_raw

 resolution: \[752,480\]

cam1:

 camera\_model:omni

 intrinsics:\[0.80065662, 833.006, 830.345, 373.850, 253.749\]

 distortion\_model:radtan

 distortion\_coeffs: \[-0.33518750, 0.13211436, 0.00055967, 0.00057686\]

 T\_cn\_cnm1:

 - \[ 0.99998854,0.00216014, 0.00427195,-0.11003785\]

 - \[-0.00221074,0.99992702, 0.01187697, 0.00045792\]

 -\[-0.00424598,-0.01188627, 0.99992034,-0.00064487\]

 - \[0.0, 0.0, 0.0,1.0\]

 timeshift\_cam\_imu: -8.681e-05

 rostopic:/cam1/image\_raw

 resolution: \[752,480\]

-   **camera\_model//相机模型**  
    camera projection type (pinhole / omni)
-   **intrinsics//相机内参**  
    vector containing the intrinsic parameters for the given projection type. elements are as follows:  
    pinhole: \[fu fv pu pv\]  
    omni: \[xi fu fv pu pv\]  
    **distortion\_model//畸变模型**  
    lens distortion type (radtan / equidistant)
-   **distortion\_coeffs//畸变参数**  
    parameter vector for the distortion model
-   **T\_cn\_cnm1//左右摄像头的相对位姿**  
    camera extrinsic transformation, always with respect to the last camera in the chain  
    (e.g. cam1: T\_cn\_cnm1 = T\_c1\_c0, takes cam0 to cam1 coordinates)
-   **timeshift\_cam\_imu//在捕捉数据时，imu数据和图像时间偏移**  
    timeshift between camera and IMU timestamps in seconds (t\_imu = t\_cam + shift)
-   **rostopic**  
    topic of the camera's image stream
-   **resolution**  
    camera resolution \[width,height\]

## 3 **IMU参数文件：imu.yaml**

#Accelerometers

accelerometer\_noise\_density: 1.86e-03   #Noise density (continuous-time)

accelerometer\_random\_walk:   4.33e-04  #Bias random walk

#Gyroscopes

gyroscope\_noise\_density:     1.87e-04  #Noise density (continuous-time)

gyroscope\_random\_walk:       2.66e-05   #Bias random walk

update\_rate:            200.0    #Hz //imu输出数据频率

编写这个文件需要imu的手册。

## 4 **标定板参数文件：**target.yam

Kalibr支持三种标定板，分别是**Aprilgrid**、**Checkerboard和Circlegrid。**

**参数比较简单：见https://github.com/ethz-asl/kalibr/wiki/calibration-targets**

在制作完成标定需要文件后，就可以对cam-imu进行标定了。

比如：你的标定板文件是april\_6x6.yaml，相机参数文件时camchain.yaml，imu参数文件是imu\_adis16448.yaml，图集文件是dynamic.bag，\--bag-from-to 5 45这里的5和45是你想在标定时利用图集数据的时间段，单位是S。执行

kalibr\_calibrate\_imu\_camera --targetapril\_6x6.yaml --cam camchain.yaml --imu imu\_adis16448.yaml --bag  dynamic.bag --bag-from-to 5 45

你会得到几个输出文件。

-   **report-imucam-%BAGNAME%.pdf**: Report in PDF format. Contains all plots for documentation.
-   **results-imucam-%BAGNAME%.txt**: Result summary as a text file.
-   **camchain-imucam-%BAGNAME%.yaml**: 这个文件是在输入文件camchain.yaml基础上增加了标定后的cam-imu信息的结果文件。我们想要的T\_cam\_imu矩阵就在这里。

**1 virtual memory exhaust ：原因：电脑内存不够**

**2 如果电脑没有安装 numpy，会报错，安装numpy。**

**3 执行指令时如果找不到输入文件，附绝对路径试试。**

**4 CDE包只支持64位linux系统**

**5注意：在采集标定图集时**

**i,确保图像不要模糊，运动不要太快。**

**ii，最好要遍历imu的所有轴，即充分旋转和加速。**

**6calib可以标定单目+imu，bag内只放单目图像，修改带参运行中的参数即可。**

**7虽然标定板有几种可以选择，但作者建议使用aprilgrid类型的标定板来标定，aprilgrid地址：https://drive.google.com/file/d/0B0T1sizOvRsUdjFJem9mQXdiMTQ/edit  
**