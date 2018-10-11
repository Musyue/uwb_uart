#include <sstream>
#include <string>

#include "ros/ros.h"
#include "uwb_uart/velo.h"

using std::string;
using std::stringstream;

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_send_velo");
  ros::NodeHandle n;
  ros::Publisher velo_pub = n.advertise<uwb_uart::velo>("get_velo", 100);
  
  ros::Rate r(5);
  double v_1, v_2;
  double v_l, v_r;
  std::cin >> v_l >> v_r;
  v_1 = 0.5 * (v_l - v_r);
  v_2 = 0.5 * (v_l + v_r);

  string v1, v2;
  stringstream ss1, ss2;
  ss1 << v_1;
  ss2 << v_2;
  ss1 >> v1;
  ss2 >> v2; 

  while (ros::ok()) {
    uwb_uart::velo v;
    v.v1 = v1;
    v.v2 = v2;
    std::cout << v1 << " and " << v2 << std::endl;
    velo_pub.publish(v);
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
