#include "ros/ros.h"

#include "renov_localization/KalmanFilter.h"
#include "renov_localization/KFRos.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "kalman_filter");
  ros::NodeHandle n;
  ros::Rate r(30);

  std::string addr_1 = "/data/ros/ur_ws_yue/src/uwb_uart/data/uwb_position.txt";
  std::string addr_2 = "/data/ros/ur_ws_yue/src/uwb_uart/data/acceleration.txt";
  kf_ros::KFRos localize(addr_1, addr_2, n);

  ros::Subscriber range_sub = n.subscribe("uwb_dis_info", 100, 
                                          &kf_ros::KFRos::GetRangeCallback,
                                          &localize);
  ros::spin();
  r.sleep();  
  return 0;
}
