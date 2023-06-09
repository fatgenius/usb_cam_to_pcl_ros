# 图像到点云转换器

## 简介
这是一个使用OpenCV和PCL库将相机图像转换为点云的ROS（机器人操作系统）节点。点云数据将作为ROS消息进行发布。

## 依赖项
请确保已安装以下依赖项：

- ROS
- OpenCV
- PCL（点云库）

## 构建包
1. 创建一个Catkin工作空间（如果尚未创建）：
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
2. 将包克隆到`src`目录：

cd ~/catkin_ws/src
git clone <repository_url>
3. 构建包：
cd ~/catkin_ws
catkin_make

## 运行节点
1. 启动ROS主节点：
roscore

2. 运行节点：


   确保用实际的包名替换`<package_name>`。

3. 节点订阅`/usb_cam/image_raw`话题以接收图像输入，请确保将相机图像发布到该话题。

4. 节点将生成的点云作为ROS消息发布到`/spatio_temporal_voxel_layer/point_cloud`话题。

## 调整相机参数
需要根据实际相机设置在代码中设置相机内参。请修改代码中的以下行：

cameraIntrinsic.at<double>(0, 0) = <focal_length_x>;  // x轴焦距
cameraIntrinsic.at<double>(1, 1) = <focal_length_y>;  // y轴焦距
cameraIntrinsic.at<double>(0, 2) = <principal_point_x>;  // 主点x坐标
cameraIntrinsic.at<double>(1, 2) = <principal_point_y>;  // 主点y坐标

## 代码流程

- 初始化ROS节点和节点句柄
- 订阅相机图像话题 `/usb_cam/image_raw`，指定回调函数 `imageCallback`
- 创建点云发布器，发布到话题 `/spatio_temporal_voxel_layer/point_cloud`
- 设置相机内参矩阵 `cameraIntrinsic`
- 进入ROS主循环

## imageCallback
- 输入：相机图像消息
- 将ROS图像消息转换为OpenCV格式
- 将图像转换为灰度图像
- 创建空的点云对象 `pointCloud`
- 遍历灰度图像像素
  - 获取像素的灰度值
  - 根据灰度值计算深度 `depth`，调用函数 `mapGrayToDepth`
  - 根据像素位置和深度计算点的三维坐标，存储到点云对象 `pointCloud`
- 将点云对象转换为ROS消息 `rosPointCloud`
- 创建TF监听器 `listener`
- 获取地图到相机的变换，存储到变量 `transform`
- 对点云进行坐标系变换，将点云转换到地图坐标系下，存储到点云对象 `transformedPointCloud`
- 将转换后的点云数据填入ROS消息 `rosPointCloud`
- 设置ROS消息的元数据：坐标系为地图坐标系，时间戳为当前时间
- 发布点云消息 `rosPointCloud` 到话题 `/spatio_temporal_voxel_layer/point_cloud`

## mapGrayToDepth
- 输入：灰度值 `grayValue`
- 定义灰度值和深度的映射范围
- 将灰度值映射到深度范围内
- 返回深度值

