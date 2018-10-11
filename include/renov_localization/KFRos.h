#ifndef KFROS_H
#define KFROS_H

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "uwb_uart/Uwbdis.h"


#include "KalmanFilter.h"


namespace kf_ros {

class KFRos {
 public:
  KFRos(std::string file_name_1, 
        std::string file_name_2, 
        ros::NodeHandle n);
  KFRos() {};
  ~KFRos() {};
  void GetRangeCallback(const uwb_uart::Uwbdis& uwb_dis);

 private:
  double t;
  double pre_t;
  bool start;
  k_filter::KalmanFilter filter_1;
  ros::Publisher position_pub;
  ros::Publisher pose_pub;
  
  
  
};  // class KFRos

}  // namespace kf_ros




#endif
