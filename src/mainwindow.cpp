/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of interact_slam.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "roshandle.h"
#include <math.h>
#include <string>
#include <fstream>




MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  interactString="交互程序正在启动\n";
  timer=new QTimer();
  timerSlow=new QTimer();
  trackPointCloud.reset (new PointCloudXYZ);

  showInstanceNum=0;
  showInstanceLabelNum=0;

  std::string coco_label_[]={"person", "bicycle", "car", "motorcycle", "airplane", "bus",
                  "train", "truck", "boat", "traffic light", "fire hydrant",
                  "stop sign", "parking meter", "bench", "bird", "cat", "dog",
                  "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
                  "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
                  "skis", "snowboard", "sports ball", "kite", "baseball bat",
                  "baseball glove", "skateboard", "surfboard", "tennis racket",
                  "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
                  "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
                  "hot dog", "pizza", "donut", "cake", "chair", "couch",
                  "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
                  "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
                  "toaster", "sink", "refrigerator", "book", "clock", "vase",
                  "scissors", "teddy bear", "hair drier", "toothbrush"};
      //复制到COCO_label数组
  std::copy(std::begin(coco_label_),std::end(coco_label_),std::begin(COCO_CLASSES));

  rawImageTopicString="/camera/rgb";
  instanceImageTopicString="/ps/rgb";
  densePointCloudTopicString="/point_cloud/raw";
  sparsePointCloudTopicString="/point_cloud/sparse";
  navigationTopicString="/map";
}

MainWindow::~MainWindow()
{
  delete ui;
}


void MainWindow::Init()
{
  pointSize=1;
  InitPCLViewer();
  InitWindow();

  //init comboBox
  on_rawImageTopicRefreshButton_clicked();
  on_instanceSegmentationImageTopicRefreshButton_clicked();
  on_densePointCloudMapTopicRefreshButton_clicked();
  on_sparsePointCloudMapTopicRefreshButton_clicked();
  on_navigationMapTopicRefreshButton_clicked();
}



void MainWindow::InitWindow()
{
  cv::Mat imgRand = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
  cv::randu(imgRand, cv::Scalar::all(0), cv::Scalar::all(255));
  ui->rawImage->setPixmap(QPixmap::fromImage(Mat2QImage(imgRand)));
  ui->instanceSegmentationImage->setPixmap(QPixmap::fromImage(Mat2QImage(imgRand)));
  ui->navigationMap->setPixmap(QPixmap::fromImage(Mat2QImage(imgRand)));

  //on_rawImageTopicRefreshButton_clicked();

  ui->interactOutputLabel->setText(interactString);
  ui->interactOutputLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
  ui->interactOutputLabel->setTextInteractionFlags(Qt::TextSelectableByKeyboard);

  ui->rawImageEnableCheckbox->setChecked(true);
//  ui->instanceSegmentationImageEnableCheckbox->setChecked(false);
  //ui->densePointCloudMapEnableCheckbox->setChecked(false);
  //ui->sparsePointCloudMapEnableCheckbox->setChecked(false);
  //ui->navigationMapEnableCheckbox->setChecked(false);
  ui->instanceSegmentationImageSubComboBox->setEnabled(false);
  ui->instanceSegmentationImageTopicRefreshButton->setEnabled(false);
  ui->densePointCloudMapSubComboBox->setEnabled(false);
  ui->densePointCloudMapTopicRefreshButton->setEnabled(false);
  ui->sparsePointCloudMapSubComboBox->setEnabled(false);
  ui->sparsePointCloudMapTopicRefreshButton->setEnabled(false);
  ui->navigationMapSubComboBox->setEnabled(false);
  ui->navigationMapTopicRefreshButton->setEnabled(false);
}


QImage MainWindow::Mat2QImage(cv::Mat img)
{
  if(img.type()==16){
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    return QImage((const unsigned char*)(img.data), int(img.cols), int(img.rows), img.step, QImage::Format_RGB888);
  }
  else if(img.type()==0){
    return QImage((const unsigned char*)(img.data), int(img.cols), int(img.rows), img.step, QImage::Format_Mono);
  }
  else if(img.type()==5){
    return QImage((const unsigned char*)(img.data), int(img.cols), int(img.rows), img.step, QImage::Format_Mono);
  }
}




QImage MainWindow::GetRandomImage()
{
  cv::Mat imgRand = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
  cv::randu(imgRand, cv::Scalar::all(0), cv::Scalar::all(255));
  return Mat2QImage(imgRand);
}


void MainWindow::InitPCLViewer()
{
      densePointCloudViewer.reset (new pcl::visualization::PCLVisualizer ("DensePointMap", false));
      densePointCloudViewer->setBackgroundColor (0, 0, 0);
      densePointCloudViewer->addCoordinateSystem (0.5);//设置坐标轴大小
      densePointCloudViewer->initCameraParameters ();
      ui->densePointCloudMap->SetRenderWindow (densePointCloudViewer->getRenderWindow());
      densePointCloudViewer->setupInteractor (ui->densePointCloudMap->GetInteractor (), ui->densePointCloudMap->GetRenderWindow ());
      ui->densePointCloudMap->update ();

      sparsePointCloudViewer.reset (new pcl::visualization::PCLVisualizer ("FeaturePointMap", false));
      sparsePointCloudViewer->setBackgroundColor (0, 0, 0);
      sparsePointCloudViewer->addCoordinateSystem (0.5);//设置坐标轴大小
      sparsePointCloudViewer->initCameraParameters ();
      ui->sparsePointCloudMap->SetRenderWindow (sparsePointCloudViewer->getRenderWindow());
      sparsePointCloudViewer->setupInteractor (ui->sparsePointCloudMap->GetInteractor (), ui->sparsePointCloudMap->GetRenderWindow ());
      ui->sparsePointCloudMap->update ();

      densePointCloud.reset (new PointCloud);
      densePointCloud->points.resize (1000);
      for (size_t i = 0; i < densePointCloud->points.size (); ++i)
      {
        densePointCloud->points[i].x = 2 * rand () / (RAND_MAX + 1.0f);
        densePointCloud->points[i].y = 2 * rand () / (RAND_MAX + 1.0f);
        densePointCloud->points[i].z = 2 * rand () / (RAND_MAX + 1.0f);

        densePointCloud->points[i].r = 255;
        densePointCloud->points[i].g = 255;
        densePointCloud->points[i].b = 255;
      }

      sparsePointCloud.reset (new PointCloudXYZ);
      sparsePointCloud->points.resize (1000);
      for (size_t i = 0; i < sparsePointCloud->points.size (); ++i)
      {
        sparsePointCloud->points[i].x = 2 * rand () / (RAND_MAX + 1.0f);
        sparsePointCloud->points[i].y = 2 * rand () / (RAND_MAX + 1.0f);
        sparsePointCloud->points[i].z = 2 * rand () / (RAND_MAX + 1.0f);
      }

      densePointCloudViewer->addPointCloud (densePointCloud, "cloud");
      ui->densePointCloudMap->update ();

      sparsePointCloudViewer->addPointCloud (sparsePointCloud, "cloud");
     ui->sparsePointCloudMap->update ();
}


void MainWindow::UpdateInstance()
{
  std::string fileName="/media/chen/chen/Robot/ws/data/InstanceMat.exr";
  cv::Mat img=cv::imread(fileName,-1);
  if(img.empty())
    return;

  instVector.clear();
  for(int i=0;i<img.rows;i++)
  {
    std::vector<float> v;
    for(int j=0;j<img.cols;j++)
      v.push_back(float(img.at<float>(i,j)));
    instVector.push_back(v);
  }

  if(ui->densePointCloudMapInstanceCheckbox->checkState()==Qt::CheckState::Checked){
    HideInstance(densePointCloudViewer);
    ShowInstance(densePointCloudViewer);
  }
  if(ui->sparsePointCloudMapInstanceCheckbox->checkState()==Qt::CheckState::Checked){
    HideInstance(sparsePointCloudViewer);
    ShowInstance(sparsePointCloudViewer);
  }
  if(ui->densePointCloudMapShowLabelCheckbox->checkState()==Qt::CheckState::Checked){
    HideInstanceLabel(densePointCloudViewer);
    ShowInstanceLabel(densePointCloudViewer);
  }
  if(ui->sparsePointCloudMapShowLabelCheckbox->checkState()==Qt::CheckState::Checked){
    HideInstanceLabel(sparsePointCloudViewer);
    ShowInstanceLabel(sparsePointCloudViewer);
  }
}


void MainWindow::ShowInstance(ViewerPtr viewer)
{
  if(instVector.size()>0)
  {
    for(int i=0;i<instVector.size();i++)
    {
        std::vector<float> v=instVector[i];
        std::string name="instance"+std::to_string(i);
        double color_r=rand() / double(RAND_MAX);
        double color_g=rand() / double(RAND_MAX);
        double color_b=rand() / double(RAND_MAX);
        //绘制立方体
        viewer->addCube(v[2],v[3],v[4],v[5],v[6],v[7],color_r,color_g,color_b,name);
    }
    showInstanceNum=instVector.size();
  }
}


void MainWindow::HideInstance(ViewerPtr viewer)
{
  if(showInstanceNum>0)
  {
    for(int i=0;i<showInstanceNum;i++)
        viewer->removeShape("instance"+std::to_string(i));
  }
}


void MainWindow::ShowInstanceLabel(ViewerPtr viewer)
{
  if(instVector.size()>0)
  {
    for(int i=0;i<instVector.size();i++)
    {
        std::vector<float> v=instVector[i];
        std::string insName=COCO_CLASSES[int(v[0])]+std::to_string(int(v[1]));
        double color_r=rand() / double(RAND_MAX);
        double color_g=rand() / double(RAND_MAX);
        double color_b=rand() / double(RAND_MAX);
        PointXYZ p;
        p.x=v[2];
        p.y=v[4];
        p.z=v[6];
        //绘制标签，尺度为1.0,颜色为红色
        //viewer->addText3D("*"+insName,p,0.03,color_r,color_g,color_b,"label"+std::to_string(i));
       // viewer->addText3D("*"+insName,p,0.03,color_r,color_g,color_b);
    }
    showInstanceLabelNum=instVector.size();
  }
}

void MainWindow::HideInstanceLabel(ViewerPtr viewer)
{
  if(showInstanceLabelNum>0)
  {
    //for(int i=0;i<showInstanceLabelNum;i++)
     //   viewer->removeText3D("");
  }
}


void MainWindow::SetRawImage(cv::Mat img)
{
  QImage imgQ=Mat2QImage(img);
  ui->rawImage->setPixmap(QPixmap::fromImage(imgQ));
}

void MainWindow::SetInstanceImage(cv::Mat img)
{
  QImage imgQ=Mat2QImage(img);
  ui->instanceSegmentationImage->setPixmap(QPixmap::fromImage(imgQ));
}


void MainWindow::SetDenseMap(PointCloud::ConstPtr pc)
{
  densePointCloudViewer->updatePointCloud(pc,"cloud");
  ui->densePointCloudMap->update ();
}

void MainWindow::SetSparseMap(PointCloudXYZ::ConstPtr pc)
{
  sparsePointCloudViewer->updatePointCloud(pc,"cloud");
  ui->sparsePointCloudMap->update ();
}

void MainWindow::SetNavigationMap(const nav_msgs::OccupancyGrid& msg)
{
  int width=int(msg.info.width);
  int height=int(msg.info.height);

  QPoint leftButtonPosition=QPoint(msg.info.origin.position.x,msg.info.origin.position.y);
  float step=msg.info.resolution;

  QPoint originPosition;
  originPosition.setX(int(-leftButtonPosition.x()*10));
  originPosition.setY(int(-leftButtonPosition.y()*10));


  std::vector<signed char> vec=msg.data;
  int cursor=width*height;

  cv::Mat img = cv::Mat::zeros(cv::Size(width, height), CV_8UC1);
  for (int i = 0; i<img.rows ; i++) {
      uchar *imageRow = img.ptr(i);
      for (int j = img.cols-1;j>=0; j--)
          imageRow[j]=(unsigned char)(255-vec[--cursor]*255/100);
  }
  cv::Mat imgColor = cv::Mat::zeros(cv::Size(width, height), CV_8UC3);
  cv::cvtColor(img, imgColor, cv::COLOR_GRAY2RGB);

  cv::Mat imgResize;
  //图片自适应窗口
  double wPh=586.0/440.0;
  if(width/height>wPh){
    double scale=586.0/width;
    cv::resize(imgColor, imgResize, cv::Size(),scale,scale);
  }
  else{
    double scale=440.0/height;
    cv::resize(imgColor, imgResize, cv::Size(),scale,scale);
  }




  QImage qimg=Mat2QImage(imgResize);

  QPainter painter(&qimg);
  QPen pen(QColor(0,255,0));
  painter.setPen(pen);

  if(0<=originPosition.x() && originPosition.x()<width
     && 0<=originPosition.y() && originPosition.y()<height)
  painter.drawEllipse(originPosition,10,10);


  ui->navigationMap->setPixmap(QPixmap::fromImage(qimg));
}



inline void MainWindow::SetOutputString(QString s)
{
  interactString+=s+"\n";
  QList<QString> sl=interactString.split(QRegExp("[\r\n]"),QString::SkipEmptyParts);

  if(sl.length()>30){
    interactString="";
    int i=sl.length()-30;
    for (QList<QString>::const_iterator jt = sl.begin(); jt != sl.end(); jt++)
    {
      i--;
      if(i>0)
        continue;
      interactString+=*jt+"\n";
    }
  }
  ui->interactOutputLabel->setText(interactString);
}




void MainWindow::InsertPose(tf::Quaternion q,tf::Vector3 t)
{
  PointXYZ p;
  p.x=float(t.getX());
  p.y=float(t.getY());
  p.z=float(t.getZ());

  if(trackPointCloud->size()==0){
    trackPointCloud->points.push_back(p);
    return;
  }

  PointXYZ lastPose=trackPointCloud->points[trackPointCloud->size()-1];
  double distance=sqrt(pow(lastPose.x-p.x,2)+pow(lastPose.y-p.y,2)+pow(lastPose.z-p.z,2));
  if(distance<0.01)
    return;
  trackPointCloud->points.push_back(p);

  if(ui->densePointCloudMapTrackCheckbox->checkState() == Qt::CheckState::Checked)
  {
    densePointCloudViewer->addLine(lastPose,p,"track"+std::to_string(trackPointCloud->size()));
    ui->densePointCloudMap->update();
  }

  if(ui->sparsePointCloudMapTrackCheckbox->checkState() == Qt::CheckState::Checked)
  {
    sparsePointCloudViewer->addLine(lastPose,p,"track"+std::to_string(trackPointCloud->size()));
    ui->sparsePointCloudMap->update();
  }
}







void MainWindow::on_rawImageEnableCheckbox_stateChanged(int arg1)
{
  if(arg1==2) //Enable
  {
    ui->rawImageSubComboBox->setEnabled(true);
    ui->rawImageTopicRefreshButton->setEnabled(true);
    rh->RawImageSubscribe(rawImageTopicString);
    on_rawImageTopicRefreshButton_clicked();

    SetOutputString("打开数据源监视器");
  }
  else if(arg1==0) //Disable
  {
    ui->rawImageSubComboBox->setEnabled(false);
    ui->rawImageTopicRefreshButton->setEnabled(false);
    rh->rawImageSub.shutdown();
    cv::Mat imgRand = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
      cv::randu(imgRand, cv::Scalar::all(0), cv::Scalar::all(255));
    ui->rawImage->setPixmap(QPixmap::fromImage(Mat2QImage(imgRand)));
    SetOutputString("关闭数据源监视器");
  }
}

void MainWindow::on_instanceSegmentationImageEnableCheckbox_stateChanged(int arg1)
{
  if(arg1==2) //Enable
  {
    ui->instanceSegmentationImageSubComboBox->setEnabled(true);
    ui->instanceSegmentationImageTopicRefreshButton->setEnabled(true);
    rh->InstanceImageSubscribe(instanceImageTopicString);
    on_instanceSegmentationImageTopicRefreshButton_clicked();

    SetOutputString("打开实例分割监视器");
  }
  else if(arg1==0) //Disable
  {
    ui->instanceSegmentationImageSubComboBox->setEnabled(false);
    ui->instanceSegmentationImageTopicRefreshButton->setEnabled(false);
    rh->instanceImageSub.shutdown();
    cv::Mat imgRand = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
      cv::randu(imgRand, cv::Scalar::all(0), cv::Scalar::all(255));
    ui->instanceSegmentationImage->setPixmap(QPixmap::fromImage(Mat2QImage(imgRand)));
    SetOutputString("关闭实例分割监视器");
  }
}

void MainWindow::on_densePointCloudMapEnableCheckbox_stateChanged(int arg1)
{
  if(arg1==2) //Enable
  {
    ui->densePointCloudMapSubComboBox->setEnabled(true);
    ui->densePointCloudMapTopicRefreshButton->setEnabled(true);
    rh->DenseMapSubscribe(densePointCloudTopicString);
    on_densePointCloudMapTopicRefreshButton_clicked();

    SetOutputString("打开稠密点云监视器");
  }
  else if(arg1==0) //Disable
  {
    ui->densePointCloudMapSubComboBox->setEnabled(false);
    ui->densePointCloudMapTopicRefreshButton->setEnabled(false);
    rh->denseMapSub.shutdown();
    SetOutputString("关闭稠密点云监视器");
  }
}

void MainWindow::on_sparsePointCloudMapEnableCheckbox_stateChanged(int arg1)
{
  if(arg1==2) //Enable
  {
    ui->sparsePointCloudMapSubComboBox->setEnabled(true);
    ui->sparsePointCloudMapTopicRefreshButton->setEnabled(true);
    rh->SparseMapSubscribe(sparsePointCloudTopicString);
    on_sparsePointCloudMapTopicRefreshButton_clicked();

    SetOutputString("打开稀疏点云监视器");
  }
  else if(arg1==0) //Disable
  {
    ui->sparsePointCloudMapSubComboBox->setEnabled(false);
    ui->sparsePointCloudMapTopicRefreshButton->setEnabled(false);
    rh->sparseMapSub.shutdown();
    SetOutputString("关闭稀疏点云监视器");
  }
}

void MainWindow::on_navigationMapEnableCheckbox_stateChanged(int arg1)
{
  if(arg1==2) //Enable
  {
    ui->navigationMapSubComboBox->setEnabled(true);
    ui->navigationMapTopicRefreshButton->setEnabled(true);
    rh->NavigationSubscribe(navigationTopicString);
    on_navigationMapTopicRefreshButton_clicked();
    SetOutputString("打开导航监视器");
  }
  else if(arg1==0) //Disable
  {
    ui->navigationMapSubComboBox->setEnabled(false);
    ui->navigationMapTopicRefreshButton->setEnabled(false);
    rh->navigationSub.shutdown();
    cv::Mat imgRand = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
      cv::randu(imgRand, cv::Scalar::all(0), cv::Scalar::all(255));
    ui->navigationMap->setPixmap(QPixmap::fromImage(Mat2QImage(imgRand)));
    SetOutputString("关闭导航监视器");
  }
}

void MainWindow::on_pointSizeSlider_valueChanged(int value)
{
    pointSize=value;
    ui->pointSizeLabel->setNum(value);
}

void MainWindow::on_pointSizeSlider_sliderReleased()
{
  densePointCloudViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloud");
  ui->densePointCloudMap->update();
  sparsePointCloudViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloud");
  ui->sparsePointCloudMap->update();
}

void MainWindow::on_densePointCloudMapInstanceCheckbox_stateChanged(int arg1)
{
  if(arg1==2) //Enable
  {
    ShowInstance(densePointCloudViewer);
    SetOutputString("稠密地图显示实例");
  }
  else if(arg1==0) //Disable
  {
    HideInstance(densePointCloudViewer);
    SetOutputString("稠密地图隐藏实例");
  }
}

void MainWindow::on_sparsePointCloudMapInstanceCheckbox_stateChanged(int arg1)
{
  if(arg1==2) //Enable
  {
    ShowInstance(sparsePointCloudViewer);
    SetOutputString("稀疏地图显示实例");
  }
  else if(arg1==0) //Disable
  {
    HideInstance(sparsePointCloudViewer);
    SetOutputString("稀疏地图隐藏实例");
  }
}



void MainWindow::on_rawImageTopicRefreshButton_clicked()
{
  ui->rawImageSubComboBox->clear();

  QSet<QString> message_types;
  message_types.insert("sensor_msgs/Image");
  message_types.insert("sensor_msgs/CompressedImage");
  QList<QString> topicList=rh->GetSpecificTopics(message_types);

  if(topicList.length()<1){
    SetOutputString("未监听到数据源");
    return;
  }

  SetOutputString("监听到的数据源:");
  for (QList<QString>::const_iterator it = topicList.begin(); it != topicList.end(); it++)
  {
    QString label(*it);
    label.replace(" ", "/");
    ui->rawImageSubComboBox->addItem(label, QVariant(*it));
    SetOutputString(label);
  }
  ui->rawImageSubComboBox->setCurrentIndex(0);//default
  rawImageTopicString=ui->rawImageSubComboBox->currentText();
}

void MainWindow::on_rawImageSubComboBox_currentIndexChanged(const QString &arg1)
{
  if(arg1.length()<2)
    return;
  rawImageTopicString = arg1;
  if(ui->rawImageEnableCheckbox->checkState() == Qt::CheckState::Checked){
    SetOutputString("设置数据源的主题为："+rawImageTopicString);
    rh->RawImageSubscribe(rawImageTopicString);
  }
  else{
    SetOutputString("数据源监视器未打开");
  }
}


void MainWindow::on_instanceSegmentationImageTopicRefreshButton_clicked()
{
  ui->instanceSegmentationImageSubComboBox->clear();

  QSet<QString> message_types;
  message_types.insert("sensor_msgs/Image");
  message_types.insert("sensor_msgs/CompressedImage");
  QList<QString> topicList=rh->GetSpecificTopics(message_types);

  if(topicList.length()<1){
    SetOutputString("未监听到实例分割");
    return;
  }

  SetOutputString("监听到的实例分割");

  for (QList<QString>::const_iterator it = topicList.begin(); it != topicList.end(); it++)
  {
    QString label(*it);
    label.replace(" ", "/");
    ui->instanceSegmentationImageSubComboBox->addItem(label, QVariant(*it));
    SetOutputString(label);
  }
  ui->instanceSegmentationImageSubComboBox->setCurrentIndex(0);//default
  instanceImageTopicString=ui->instanceSegmentationImageSubComboBox->currentText();
}


void MainWindow::on_instanceSegmentationImageSubComboBox_currentIndexChanged(const QString &arg1)
{
  if(arg1.length()<2)
    return;
  instanceImageTopicString = arg1;
  if(ui->instanceSegmentationImageEnableCheckbox->checkState() == Qt::CheckState::Checked){
    SetOutputString("设置实例分割的主题为："+instanceImageTopicString);
    rh->InstanceImageSubscribe(instanceImageTopicString);
  }
  else{
    SetOutputString("实例分割监视器未打开");
  }
}

void MainWindow::on_densePointCloudMapTopicRefreshButton_clicked()
{
    ui->densePointCloudMapSubComboBox->clear();

    QSet<QString> message_types;
    message_types.insert("sensor_msgs/PointCloud2");
    QList<QString> topicList=rh->GetSpecificTopics(message_types);

    if(topicList.length()<1){
      SetOutputString("未监听到稠密点云");
      return;
    }

    SetOutputString("监听到的稠密点云");

    for (QList<QString>::const_iterator it = topicList.begin(); it != topicList.end(); it++)
    {
      QString label(*it);
      label.replace(" ", "/");
      ui->densePointCloudMapSubComboBox->addItem(label, QVariant(*it));
      SetOutputString(label);
    }
    ui->densePointCloudMapSubComboBox->setCurrentIndex(0);//default
    densePointCloudTopicString=ui->densePointCloudMapSubComboBox->currentText();
}

void MainWindow::on_densePointCloudMapSubComboBox_currentIndexChanged(const QString &arg1)
{
  if(arg1.length()<2)
    return;
  densePointCloudTopicString = arg1;
  if(ui->densePointCloudMapEnableCheckbox->checkState() == Qt::CheckState::Checked){
    SetOutputString("设置稠密点云的主题为："+densePointCloudTopicString);
    rh->DenseMapSubscribe(densePointCloudTopicString);
  }
  else{
    SetOutputString("稠密点云监视器未打开");
  }
}

void MainWindow::on_sparsePointCloudMapTopicRefreshButton_clicked()
{
  ui->sparsePointCloudMapSubComboBox->clear();

  QSet<QString> message_types;
  message_types.insert("sensor_msgs/PointCloud2");
  QList<QString> topicList=rh->GetSpecificTopics(message_types);

  if(topicList.length()<1){
    SetOutputString("未监听到稀疏点云");
    return;
  }

  SetOutputString("监听到的稀疏点云");
  for (QList<QString>::const_iterator it = topicList.begin(); it != topicList.end(); it++)
  {
    QString label(*it);
    label.replace(" ", "/");
    ui->sparsePointCloudMapSubComboBox->addItem(label, QVariant(*it));
    SetOutputString(label);
  }
  ui->sparsePointCloudMapSubComboBox->setCurrentIndex(0);//default
  sparsePointCloudTopicString=ui->sparsePointCloudMapSubComboBox->currentText();
}

void MainWindow::on_sparsePointCloudMapSubComboBox_currentIndexChanged(const QString &arg1)
{
  if(arg1.length()<2)
    return;
  sparsePointCloudTopicString = arg1;
  if(ui->sparsePointCloudMapEnableCheckbox->checkState() == Qt::CheckState::Checked){
    SetOutputString("设置稀疏点云的主题为："+sparsePointCloudTopicString);
    rh->SparseMapSubscribe(sparsePointCloudTopicString);
  }
  else{
    SetOutputString("稀疏点云监视器未打开");
  }
}

void MainWindow::on_navigationMapTopicRefreshButton_clicked()
{
  ui->navigationMapSubComboBox->clear();

  QSet<QString> message_types;
  message_types.insert("nav_msgs/OccupancyGrid");
  QList<QString> topicList=rh->GetSpecificTopics(message_types);

  if(topicList.length()<1){
    SetOutputString("未监听到导航地图");
    return;
  }

  SetOutputString("监听到的导航地图");
  for (QList<QString>::const_iterator it = topicList.begin(); it != topicList.end(); it++)
  {
    QString label(*it);
    label.replace(" ", "/");
    ui->navigationMapSubComboBox->addItem(label, QVariant(*it));
    SetOutputString(label);
  }
  ui->navigationMapSubComboBox->setCurrentIndex(0);//default
  navigationTopicString=ui->navigationMapSubComboBox->currentText();
}

void MainWindow::on_navigationMapSubComboBox_currentIndexChanged(const QString &arg1)
{
  if(arg1.length()<2)
    return;
  navigationTopicString = arg1;
  if(ui->navigationMapEnableCheckbox->checkState() == Qt::CheckState::Checked){
    SetOutputString("设置导航地图的主题为："+navigationTopicString);
    rh->NavigationSubscribe(navigationTopicString);
  }
  else{
    SetOutputString("导航地图监视器未打开");
  }
}

void MainWindow::on_interactSendButton_clicked()
{
  QString s=ui->interactInputLineEdit->text();

  if(s=="")
    return;

  ui->interactInputLineEdit->clear();
  ui->interactInputLineEdit->setFocus();
  SetOutputString("我:"+s);

  QString result=rh->SendOrder(s);
  SetOutputString("系统:"+result);
}



void MainWindow::on_interactOutpuClearButton_clicked()
{
    interactString="";
    ui->interactOutputLabel->clear();
}


void MainWindow::on_pushButton_clicked()
{
    QString s=rh->GetAllTopics();
    SetOutputString(s);
}

void MainWindow::on_densePointCloudMapTrackCheckbox_stateChanged(int arg1)
{
  if(arg1==2) //Enable
  {
    if(trackPointCloud->size()<2)
      return;
    PointXYZ lastPoint=trackPointCloud->points[0];
    for(int i=1;i<trackPointCloud->size();i++){
      PointXYZ currentPoint=trackPointCloud->points[i];
      densePointCloudViewer->addLine(lastPoint,currentPoint,"track"+std::to_string(i));
      lastPoint=currentPoint;
    }
    ui->densePointCloudMap->update();
    SetOutputString("稠密显示轨迹");
  }
  else if(arg1==0) //Disable
  {
    densePointCloudViewer->removeAllShapes();
    ui->densePointCloudMap->update();
    SetOutputString("稠密隐藏轨迹");
  }
}

void MainWindow::on_sparsePointCloudMapTrackCheckbox_stateChanged(int arg1)
{
  if(arg1==2) //Enable
  {
    if(trackPointCloud->size()<2)
      return;
    PointXYZ lastPoint=trackPointCloud->points[0];
    for(int i=1;i<trackPointCloud->size();i++){
      PointXYZ currentPoint=trackPointCloud->points[i];
      sparsePointCloudViewer->addLine(lastPoint,currentPoint,"track"+std::to_string(i));
      lastPoint=currentPoint;
    }
    ui->sparsePointCloudMap->update();
    SetOutputString("稀疏显示轨迹");
  }
  else if(arg1==0) //Disable
  {
    sparsePointCloudViewer->removeAllShapes();
    ui->sparsePointCloudMap->update();
    SetOutputString("稀疏隐藏轨迹");
  }
}

void MainWindow::on_densePointCloudMapShowLabelCheckbox_stateChanged(int arg1)
{
  if(arg1==2) //Enable
  {
    ShowInstanceLabel(densePointCloudViewer);
    SetOutputString("稠密地图显示标签");
  }
  else if(arg1==0) //Disable
  {
    HideInstanceLabel(densePointCloudViewer);
    SetOutputString("稠密地图隐藏标签");
  }
}


void MainWindow::on_sparsePointCloudMapShowLabelCheckbox_stateChanged(int arg1)
{
  if(arg1==2) //Enable
  {
    ShowInstanceLabel(sparsePointCloudViewer);
    SetOutputString("稀疏地图显示标签");
  }
  else if(arg1==0) //Disable
  {
    HideInstanceLabel(sparsePointCloudViewer);
    SetOutputString("稀疏地图隐藏标签");
  }
}

void MainWindow::on_rawImageSubComboBox_currentTextChanged(const QString &arg1)
{

}

void MainWindow::on_interactVoiceButton_clicked()
{
  if(ui->voiceInputCheckbox->checkState()==Qt::CheckState::Checked)
  {
    SetOutputString("请开始说话");
  }
  else
  {

    SetOutputString("语音输入未打开");
  }
}

void MainWindow::on_voiceInputCheckbox_stateChanged(int arg1)
{
  if(arg1==2) //Enable
  {

    SetOutputString("打开语音输入");
  }
  else if(arg1==0) //Disable
  {

    SetOutputString("关闭语音输入");
  }
}

void MainWindow::on_voiceOutputCheckbox_stateChanged(int arg1)
{
  if(arg1==2) //Enable
  {
    SetOutputString("打开语音输出");
  }
  else if(arg1==0) //Disable
  {
    SetOutputString("关闭语音输出");
  }
}
