#include "roshandle.h"
#include "ros/ros.h"
#include "interact_slam/Order.h"


RosHandle::RosHandle(ros::NodeHandle *nh_)
{
  rawImageCounter=0;
  nh=nh_;
  it=new image_transport::ImageTransport(*nh);

  listener=new tf::TransformListener();//tf监听器

  orderService=nh->serviceClient<interact_slam::Order>("OrderService");
}


QString RosHandle::SendOrder(QString msg)
{
  interact_slam::Order srv;
  srv.request.request=msg.toStdString();
  if(orderService.call(srv))
  {
    return QString(srv.response.response.c_str());
  }
  else{
    return QString("命令服务调用失败");
  }
}

void RosHandle::PoseLookup()
{
  tf::StampedTransform m;
  try{
    listener->lookupTransform("world", "orb_slam2", ros::Time(0), m);//查找坐标转换
    tf::Quaternion q=m.getRotation();
    tf::Vector3 t=m.getOrigin();
    //cout<<t.getX()<<" "<<t.getX()<<" "<<t.getX()<<endl;
    win->InsertPose(q,t);
  }
  catch (tf::TransformException &ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(0.1).sleep();
  }
}




void RosHandle::RawImageSubscribe(QString topic)
{
  rawImageSub.shutdown();
  if (!topic.isEmpty())
  {
     rawImageSub=it->subscribe(topic.toStdString(),1,&RosHandle::RawImageCallBack,this);
  }
}

void RosHandle::RawImageCallBack(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try{
       cv_ptrRGB = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
  }
  win->SetRawImage(cv_ptrRGB->image);
}

void RosHandle::InstanceImageSubscribe(QString topic)
{
  instanceImageSub.shutdown();
  if (!topic.isEmpty())
  {
     instanceImageSub=it->subscribe(topic.toStdString(),1,&RosHandle::InstanceImageCallBack,this);
  }
}

void RosHandle::InstanceImageCallBack(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try{
       cv_ptrRGB = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
  }
  win->SetInstanceImage(cv_ptrRGB->image);
}

void RosHandle::DenseMapSubscribe(QString topic)
{
  denseMapSub.shutdown();
  if (!topic.isEmpty())
  {
     denseMapSub=nh->subscribe(topic.toStdString(),1,&RosHandle::DenseMapCallBack,this);
  }
}


void RosHandle::DenseMapCallBack(const PointCloud::ConstPtr& msg)
{
  win->SetDenseMap(msg);
}


void RosHandle::SparseMapSubscribe(QString topic)
{
  sparseMapSub.shutdown();
  if (!topic.isEmpty())
  {
     sparseMapSub=nh->subscribe(topic.toStdString(),1,&RosHandle::SparseMapCallBack,this);
  }
}


void RosHandle::SparseMapCallBack(const PointCloudXYZ::ConstPtr& msg)
{
  win->SetSparseMap(msg);
  //cout<<msg->size()<<endl;
}


void RosHandle::NavigationSubscribe(QString topic)
{
  navigationSub.shutdown();
  if (!topic.isEmpty())
  {
     navigationSub=nh->subscribe(topic.toStdString(),1,&RosHandle::NavigationCallBack,this);
  }
}


void RosHandle::NavigationCallBack(const nav_msgs::OccupancyGrid& msg)
{
  win->SetNavigationMap(msg);
}




QList<QString> RosHandle::UpdateTopicList(QSet<QString> message_types)
{
  // get declared transports
  QList<QString> transports;
  image_transport::ImageTransport it(*nh);
  std::vector<std::string> declared = it.getDeclaredTransports();
  for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++)
  {
    QString transport = it->c_str();
    QString prefix = "image_transport/";
    if (transport.startsWith(prefix))
      transport = transport.mid(prefix.length());// strip prefix from transport name
    transports.append(transport);
    std::cout<<transport.toStdString()<<std::endl;
  }

  // fill combo box
  QList<QString> topics = GetTopics(message_types, transports).values();
  topics.append("");
  qSort(topics);
  return topics;
}



QSet<QString> RosHandle::GetTopics(const QSet<QString>& message_types, const QList<QString>& transports)
{
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  QSet<QString> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
    all_topics.insert(it->name.c_str());

  QSet<QString> topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    if (message_types.contains(it->datatype.c_str()))
    {
      QString topic = it->name.c_str();
      topics.insert(topic);// add raw topic

      // add transport specific sub-topics
      for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
      {
        if (all_topics.contains(topic + "/" + *jt))
        {
          QString sub = topic + " " + *jt;
          topics.insert(sub);
        }
      }
    }

  }
  return topics;
}




QList<QString> RosHandle::GetSpecificTopics(const QSet<QString>& message_types)
{
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  QSet<QString> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
    all_topics.insert(it->name.c_str());

  QSet<QString> tp;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    if (message_types.contains(it->datatype.c_str()))
    {
      QString topic = it->name.c_str();
      tp.insert(topic);// add raw topic
    }
  }

  QList<QString> topics = tp.values();
  topics.append("");
  qSort(topics);

  return topics;
}




QString RosHandle::GetAllTopics()
{
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  QString s="";

  QSet<QString> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    s+="主题:"+QString(it->name.c_str())+"\n";
    s+="数据类型:"+QString(it->datatype.c_str())+"\n";
    s+="\n";
  }

  return s;
}
