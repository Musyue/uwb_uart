#include "renov_localization/KFRos.h"


using kf_ros::KFRos;

KFRos::KFRos(std::string file_name_1, 
             std::string file_name_2, 
             ros::NodeHandle n) {
  
  start = false;
  filter_1 = k_filter::KalmanFilter(file_name_1, file_name_2);
  position_pub = n.advertise<geometry_msgs::Point>("global_position", 100);
  pose_pub = n.advertise<nav_msgs::Odometry>("global_pose", 100);

}

void KFRos::GetRangeCallback(const uwb_uart::Uwbdis& uwb_dis) {
  
  Eigen::Vector3d range;
  range << uwb_dis.T_A0, 
           uwb_dis.T_A1,
           uwb_dis.T_A2;
  if (!start) {
    t = ros::Time::now().toSec();
    pre_t = ros::Time::now().toSec();
    start = true;
  } else {
    t = ros::Time::now().toSec() - pre_t;
    pre_t = ros::Time::now().toSec();
  }
  Eigen::Vector3d position = filter_1.DataFusion(range, t);
  geometry_msgs::Point p;
  p.x = position(0);
  p.y = position(1);
  p.z = position(2);

  position_pub.publish(p); 

}
