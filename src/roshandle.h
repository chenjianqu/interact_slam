/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of interact_slam.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/


#ifndef ROSHANDLE_H
#define ROSHANDLE_H


#include "mainwindow.h"
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "ros/master.h"
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include<QString>
#include<QSet>
#include<QList>


class MainWindow;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

class RosHandle
{
public:
  ros::NodeHandle *nh;
  MainWindow *win;

  RosHandle(ros::NodeHandle *nh_);
  image_transport::ImageTransport *it;
  image_transport::Subscriber rawImageSub;
  image_transport::Subscriber instanceImageSub;
  ros::Subscriber navigationSub;
  ros::Subscriber denseMapSub;
  ros::Subscriber sparseMapSub;

  ros::ServiceClient orderService;
  tf::TransformListener* listener;

  void RawImageSubscribe(QString topic);
  void RawImageCallBack(const sensor_msgs::ImageConstPtr& msg);
  void InstanceImageSubscribe(QString topic);
  void InstanceImageCallBack(const sensor_msgs::ImageConstPtr& msg);
  void DenseMapSubscribe(QString topic);
  void DenseMapCallBack(const PointCloud::ConstPtr& msg);
  void SparseMapSubscribe(QString topic);
  void SparseMapCallBack(const PointCloudXYZ::ConstPtr& msg);
  void NavigationSubscribe(QString topic);
  void NavigationCallBack(const nav_msgs::OccupancyGrid& msg);

  QString SendOrder(QString msg);

  void PoseLookup();

  QSet<QString> GetTopics(const QSet<QString>& message_types, const QList<QString>& transports);
  QList<QString> UpdateTopicList(QSet<QString> message_types);
  QList<QString> GetSpecificTopics(const QSet<QString>& message_types);
  QString GetAllTopics();


private:
  int rawImageCounter;


};

#endif // INTERACT_H
