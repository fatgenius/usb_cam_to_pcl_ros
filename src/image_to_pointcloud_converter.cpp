#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

ros::Publisher pointCloudPub;
cv::Mat cameraIntrinsic;  // 相机内参

float mapGrayToDepth(uchar grayValue)
{
  // Define the mapping range for gray values and depth
  const float minGrayValue = 0;
  const float maxGrayValue = 255;
  const float minDepth = 0.1;  // Minimum depth value
  const float maxDepth = 10.0;  // Maximum depth value

  // Map the gray value to depth within the defined range
  float depth = ((grayValue - minGrayValue) / (maxGrayValue - minGrayValue)) * (maxDepth - minDepth) + minDepth;

  return depth;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // 将ROS图像消息转换为OpenCV格式
  cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

  // 将图像转换为灰度图像
  cv::Mat grayImage;
  cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

  // 创建点云对象
  pcl::PointCloud<pcl::PointXYZ> pointCloud;

  // 根据灰度图像生成点云数据
  for (int i = 0; i < grayImage.rows; i++) {
    for (int j = 0; j < grayImage.cols; j++) {
      uchar grayValue = grayImage.at<uchar>(i, j);
      float depth = mapGrayToDepth(grayValue);  // 根据灰度值计算深度

      pcl::PointXYZ point;
      point.x = (j - cameraIntrinsic.at<double>(0, 2)) * depth / cameraIntrinsic.at<double>(0, 0);
      point.y = (i - cameraIntrinsic.at<double>(1, 2)) * depth / cameraIntrinsic.at<double>(1, 1);
      point.z = depth;
      pointCloud.push_back(point);
    }
  }

  // 将点云数据转换为ROS消息
  sensor_msgs::PointCloud2 rosPointCloud;
  pcl::toROSMsg(pointCloud, rosPointCloud);

  // 创建TF监听器
  tf::TransformListener listener;

  // // 获取地图到相机的变换
  // tf::StampedTransform transform;
  // try {
  //   listener.lookupTransform("camera_link", msg->header.frame_id, ros::Time(0), transform);
  // } catch (tf::TransformException& ex) {
  //   ROS_ERROR("%s", ex.what());
  //   return;
  // }

  // // 对点云进行坐标系变换，将点云转换到地图坐标系下
  // pcl::PointCloud<pcl::PointXYZ> transformedPointCloud;
  // pcl_ros::transformPointCloud(pointCloud, transformedPointCloud, transform);

  // // 将转换后的点云数据填入ROS消息
  // pcl::toROSMsg(transformedPointCloud, rosPointCloud);

  // // 设置ROS消息的元数据
  // rosPointCloud.header.frame_id = "camera_link";  // 设置点云坐标系为地图坐标系
  // rosPointCloud.header.stamp = ros::Time::now();  // 设置时间戳

  // // 发布点云消息
  // pointCloudPub.publish(rosPointCloud);

 // ...

  // ...

  // 获取地图到相机的变换
  tf::StampedTransform transform;
  try {
    listener.lookupTransform("/map", "/camera_link", ros::Time(0), transform);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // 对点云进行坐标系变换，将点云转换到地图坐标系下
  pcl::PointCloud<pcl::PointXYZ> transformedPointCloud;
  pcl_ros::transformPointCloud(pointCloud, transformedPointCloud, transform);

  // 将转换后的点云数据填入ROS消息
  pcl::toROSMsg(transformedPointCloud, rosPointCloud);

  // 设置ROS消息的元数据
  rosPointCloud.header.frame_id = "map";  // 设置点云坐标系为地图坐标系
  rosPointCloud.header.stamp = ros::Time::now();  // 设置时间戳

  // 发布点云消息
  pointCloudPub.publish(rosPointCloud);
}

// ...




int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_to_pointcloud_converter");
  ros::NodeHandle nh;

  // 订阅图像话题
  ros::Subscriber imageSub = nh.subscribe("/usb_cam/image_raw", 1, imageCallback);

  // 创建点云发布器
  pointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/spatio_temporal_voxel_layer/point_cloud", 1);

  // 设置相机内参（根据实际相机内参进行设置）
  cameraIntrinsic = cv::Mat::eye(3, 3, CV_64F);
  cameraIntrinsic.at<double>(0, 0) = 274.6069129213586;  // focal length in x-axis
  cameraIntrinsic.at<double>(1, 1) = 276.31161814332177; // focal length in y-axis
  cameraIntrinsic.at<double>(0, 2) = 311.5795188709375;  // principal point x-coordinate
  cameraIntrinsic.at<double>(1, 2) = 224.33402263099595; // principal point y-coordinate

  ros::spin();

  return 0;
}
