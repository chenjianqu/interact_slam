#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QImage>
#include<QPixmap>
#include <QString>
#include<QList>
#include<QTimer>
#include<QPainter>


#include <vtkRenderWindow.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>


#include "roshandle.h"


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerPtr;

namespace Ui {
class MainWindow;
}


class RosHandle;//头文件相互包含要进行类别的前置声明


class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
    RosHandle *rh;

    ViewerPtr densePointCloudViewer;
    ViewerPtr sparsePointCloudViewer;
    PointCloud::Ptr densePointCloud;
    PointCloudXYZ::Ptr sparsePointCloud;
    PointCloudXYZ::Ptr trackPointCloud;

    QString interactString;

    QString rawImageTopicString;
    QString instanceImageTopicString;
    QString densePointCloudTopicString;
    QString sparsePointCloudTopicString;
    QString navigationTopicString;

    QTimer *timer;
    QTimer *timerSlow;


    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void InitWindow();
    void Init();
    void InitPCLViewer();

    void SetRawImage(cv::Mat img);
    void SetInstanceImage(cv::Mat img);
    void SetDenseMap(PointCloud::ConstPtr pc);
    void SetSparseMap(PointCloudXYZ::ConstPtr pc);
    void SetNavigationMap(const nav_msgs::OccupancyGrid& msg);
    void InsertPose(tf::Quaternion q,tf::Vector3 t);
    void UpdateInstance();
    QImage GetRandomImage();

  inline void SetOutputString(QString s);
  inline QImage Mat2QImage(cv::Mat img);


private slots:
  void on_rawImageEnableCheckbox_stateChanged(int arg1);

  void on_instanceSegmentationImageEnableCheckbox_stateChanged(int arg1);

  void on_densePointCloudMapEnableCheckbox_stateChanged(int arg1);

  void on_sparsePointCloudMapEnableCheckbox_stateChanged(int arg1);

  void on_navigationMapEnableCheckbox_stateChanged(int arg1);

  void on_pointSizeSlider_valueChanged(int value);

  void on_pointSizeSlider_sliderReleased();

  void on_densePointCloudMapInstanceCheckbox_stateChanged(int arg1);

  void on_sparsePointCloudMapInstanceCheckbox_stateChanged(int arg1);


  void on_rawImageTopicRefreshButton_clicked();

  void on_rawImageSubComboBox_currentIndexChanged(const QString &arg1);

  void on_instanceSegmentationImageTopicRefreshButton_clicked();

  void on_instanceSegmentationImageSubComboBox_currentIndexChanged(const QString &arg1);

  void on_densePointCloudMapTopicRefreshButton_clicked();

  void on_densePointCloudMapSubComboBox_currentIndexChanged(const QString &arg1);

  void on_sparsePointCloudMapTopicRefreshButton_clicked();

  void on_sparsePointCloudMapSubComboBox_currentIndexChanged(const QString &arg1);

  void on_navigationMapTopicRefreshButton_clicked();

  void on_navigationMapSubComboBox_currentIndexChanged(const QString &arg1);

  void on_interactSendButton_clicked();

  void on_interactOutpuClearButton_clicked();

  void on_rawImageSubComboBox_currentTextChanged(const QString &arg1);

  void on_pushButton_clicked();

  void on_densePointCloudMapTrackCheckbox_stateChanged(int arg1);

  void on_sparsePointCloudMapTrackCheckbox_stateChanged(int arg1);

  void on_densePointCloudMapShowLabelCheckbox_stateChanged(int arg1);

  void on_sparsePointCloudMapShowLabelCheckbox_stateChanged(int arg1);

  void on_interactVoiceButton_clicked();

  void on_voiceInputCheckbox_stateChanged(int arg1);

  void on_voiceOutputCheckbox_stateChanged(int arg1);

private:
  void ShowInstance(ViewerPtr viewer);
  void ShowInstanceLabel(ViewerPtr viewer);
  void HideInstance(ViewerPtr viewer);
  void HideInstanceLabel(ViewerPtr viewer);

  Ui::MainWindow *ui;

  int pointSize;
  int showInstanceNum;
  int showInstanceLabelNum;

  std::string COCO_CLASSES[80];

  std::vector<std::vector<float>> instVector;
};

#endif // MAINWINDOW_H
