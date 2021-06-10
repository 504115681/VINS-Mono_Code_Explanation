

# 6月8日 

## 第1章-第4讲-ROS简介

1、find 命令，找文件

```
-name<范本样式>：指定字符串作为寻找文件或目录的范本样式；
```

```latex
x@x-Legion-Y7000P-2019:~/catkin_ws5/src/PL-VIO-OK$ find ./ -name "package.xml"
./camera_model/package.xml
./vins_estimator/package.xml
./benchmark_publisher/package.xml
./feature_tracker/package.xml
./sim_data_pub/package.xml
./pose_graph/package.xml
```

2、VINS有三个可执行文件，三个节点。

3、ROS缺点：

数据COPY次数过多。（4次COPY）

## 第1章-第5讲-坐标系定义

1、VINS里面都是右手系。

2、

相机坐标系：

IMU坐标系：

3、第一个参考帧设为世界坐标系。因为有IMU所以重力方向已知，将世界坐标系z轴与重力方向对齐，roll和pitch可以求出来。

于是初始位姿为：（0,0,0,roll,pitch,0）。

4、twc是相机坐标系到世界坐标系的平移向量（以世界坐标系为参考坐标系）。也是相机坐标系相对于世界坐标系。写完整为：w^twc(w在左上角)。

# 6月10日 

## 第2章-第1讲-前端光流代码讲解（1）

1、声明一个句柄，～代表这个节点的命名空间。实际上发出去的是：**/feature_tracker**/feature

```c++
ros::init(argc, argv, "feature_tracker");//ros节点初始化,指定节点的名称"feature_tracker"。节点的名称必须唯一(名称内不能包含 / 等符号)

ros::NodeHandle n("~");//声明一个句柄，～代表这个节点的命名空间。
pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);   //发布跟踪的特征点信息。实际上发出去的是：/feature_tracker/feature
```

2、频率控制。

如果实际发布频率大于设定值10HZ，肯定就不发了，因为图片为30HZ，担心后端处理不过来。所以，要想发布图像帧，那么实际频率要比设定值10HZ小。

但是，如果实际频率与设定频率的累积误差大于0.01倍的设定频率了，重新计数。因为当误差很小时，说明分母很大了，即使1s发的超过了设定的频率值，可能也不会判断出来啦，需要将分母清0。

**注意！即使不发布该帧话题，也会进行光流追踪。**

```c++
// frequency control  f = 1 / T ,频率=1秒/周期，周期=1秒/频率
if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)//round()四舍五入
{
	PUB_THIS_FRAME = true;
	// reset the frequency control     即：实际频率 >0.99 * FREQ，说明实际频率快超过设定频率了，重新计数。
	if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
	{
		first_image_time = img_msg->header.stamp.toSec();
		pub_count = 0;
	}
}
else
	PUB_THIS_FRAME = false;
```

## 第2章-第2讲-前端光流代码讲解（2）

1、**LK金字塔光流追踪：**

```c++
cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
```

通过上一帧的特征点进行跟踪，得到现在这一帧的特征点。**而不是提取上一帧和当前帧的特征点，然后再匹配和跟踪。**

参数设为3,总共4层。最上面的是L3，最小。最下面是L0。

上一层金字塔追踪结果作为下一层金字塔追踪结果的初值。（做4遍光流追踪，时间会多一点。）

- 既避免了不用金字塔导致光流法中小运动假设不满足，陷入局部最优；
- 又避免用金字塔导致精度下降（如果只用最上面的追踪结果，然后乘上比例，扩大到最下层，会导致像素误差从1个像素变的更大）。

2、瘦身，双指针去除特定值。时间复杂度：O（n），空间复杂度：0。

```c++
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
	int j = 0;
	for (int i = 0; i < int(v.size()); i++)
        if (status[i])
			v[j++] = v[i];
	v.resize(j);
}
```

3、

void FeatureTracker::rejectWithF()

首先根据不同的相机模型将二维坐标转换到去畸变前的三维坐标，即像素坐标左成内参矩阵的逆到去畸变前的三维坐标，然后去畸变，然后再到归一化平面，然后再通过虚拟相机的内参（fx=FOCAL_LENGTH;fy=FOCAL_LENGTH;cx=COL / 2.0;cy=ROW / 2.0）到像素坐标。

```
//根据不同的相机模型将二维坐标转换到三维坐标，并去畸变。
m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
//对于PINHOLE（针孔相机）可将三维坐标转换到归一化平面，再通过虚拟相机的内参到像素坐标。
tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;//焦点距离（focal length  FOCAL_LENGTH）

tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;

un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
```

## 第2章-第4讲-光流前端vs描述子前端

光流法特征跟踪比描述子稍微鲁棒一些（ORB SLAM作者这么说的）。其实对于相邻帧特征跟踪，光流法更快。而描述子主要是用来做回环检测、重定位，也就是时间差了比较多的帧之间，中间有遮挡或者看不到也没事，还可以检测出来是同一个特征点。

## 第2章-第5讲-高效去畸变方式

离中心点越远，畸变越大。

畸变前的点-->畸变后的点，是正向过程，比较好算。但是去畸变过程，比较难。

VINS中是：逐次逼近式去畸变。