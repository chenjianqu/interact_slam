/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of interact_slam.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/


#include "ros/ros.h"
#include "ros/master.h"
#include "std_msgs/String.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"

#include "roshandle.h"
#include "mainwindow.h"


#include <thread>
#include <mutex>


#include <QApplication>
#include<QString>
#include<QSet>
#include<QList>
#include<QTimer>


using namespace std;


class CallBackClass
{
public:
  MainWindow *win;
  RosHandle *rh;

  CallBackClass(ros::NodeHandle *nhp);

  void ImageCallback(const sensor_msgs::ImageConstPtr& msg);

protected:
  unsigned long frameCounter;
};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "Console");
  ros::NodeHandle nh;

  QApplication a(argc,argv);

  CallBackClass cb(&nh);

  cb.win->timer->setInterval(200);
  cb.win->connect(cb.win->timer,&QTimer::timeout,[=]()
  {
    ros::spinOnce();
    cb.rh->PoseLookup();

  });

  cb.win->timerSlow->setInterval(10000);
  cb.win->connect(cb.win->timerSlow,&QTimer::timeout,[=]()
  {
    cb.win->UpdateInstance();
  });

   cb.win->show();

   cb.win->timer->start();
   cb.win->timerSlow->start();

   return a.exec();
}


CallBackClass::CallBackClass(ros::NodeHandle *nhp)
{
  frameCounter=0;

  rh=new RosHandle(nhp);
  win=new MainWindow();

  win->rh=rh;
  rh->win=win;

  win->Init();
}



void CallBackClass::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try{
     //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
     win->SetRawImage(cv_bridge::toCvShare(msg, "bgr8")->image);
     //cvWaitKey(1);

  }catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}












