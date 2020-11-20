## 代码的文件目录

1、ar_demo：一个ar应用demo

2、benchmark_publisher：接收并发布数据集的基准值

3、camera_model

- calib：相机参数标定
- camera_models：各种相机模型类
- chessboard：检测棋盘格
- gpl
- sparse_graph
- intrinsic_calib.cc：相机标定模块main函数

4、config：系统配置文件存放处

5、feature_trackers：

- feature_tracker_node.cpp ROS 节点函数，回调函数
- feature_tracker.cpp 图像特征光流跟踪

6、pose_graph：

- keyframe.cpp 关键帧选取、描述子计算与匹配
- pose_graph.cpp 位姿图的建立与图优化
- pose_graph_node.cpp ROS 节点函数，回调函数，主线程

7、support_files：帮助文档、Bow字典、Brief模板文件

8、vins_estimator

- factor：实现IMU、camera等残差模型
- initial：系统初始化，外参标定，SFM
- utility：相机可视化，四元数等数据转换
- estimator.cpp：紧耦合的VIO状态估计器实现
- estimator_node.cpp：ROS 节点函数，回调函数，主线程
- feature_manager.cpp：特征点管理，三角化，关键帧等
- parameters.cpp：读取参数

参考博客：[VINS-Mono论文学习与代码解读----目录与参考](https://www.baidu.com/s?ie=UTF-8&wd=Manii)

------



## 代码说明

### 1、图像预处理

在feature_trackers文件夹下有六个文件：

- feature_tracker.cpp 图像特征光流跟踪
- feature_tracker.h 对应的头文件
- feature_tracker_node.cpp 特征跟踪线程的系统入口，有ROS节点函数，回调函数等
- parameters.cpp 参数读取
- parameters.h 对应的头文件
- tic_toc.h 计时函数头文件

------

#### feature_tracker_node.cpp 特征跟踪线程的系统入口

两个函数：

第一个：主函数

```c++
int main(int argc, char **argv){}
```

作用：

特征跟踪线程的系统入口。

订阅图像话题，发布三个话题：特征点信息、特征点图像、特征跟踪模块是否出错

输入：

图像话题(如/cam0/image_raw)

输出：

三个话题：特征点信息、特征点图像、特征跟踪模块是否出错

用到的函数：

- `readParameters(n);`

  函数形式：`void readParameters(ros::NodeHandle &n)`{}

  位置：feature_trackers/parameters.cpp

  作用：读取参数，读取如config->euroc->euroc_config.yaml中的一些配置参数

- `trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);`

  函数形式：`void FeatureTracker::readIntrinsicParameter(const string &calib_file){}`

  位置：feature_trackers/feature_tracker.cpp

  作用：读取相机内参

  用法说明：单目时i=0，只有一个相机和对应的内参，trackerData[NUM_OF_CAM]是定义的FeatureTracker类，这里NUM_OF_CAM=1

- `cv::imread(FISHEYE_MASK, 0);`

  函数形式：`CV_EXPORTS_W Mat imread( const String& filename, int flags = IMREAD_COLOR );`

  位置：opencv安装的位置

  作用：读取图像

  用法说明：opencv库函数，读取FISHEYE_MASK代表的路径对应的图像，第二个参数代表图像的类型，flags=1为彩色图像；fags=0为灰度图像

```c++
void img_callback(const sensor_msgs::ImageConstPtr &img_msg){}
```
