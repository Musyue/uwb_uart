#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

#include "ros/ros.h"
#include "uwb_uart/velo.h"
#include "std_msgs/Bool.h"

#define pi 3.1415926

using std::vector;
using std::string;
using std::stringstream;

class ManuPath {
 public:
  ManuPath(string file_name, ros::NodeHandle n);
  ManuPath() {}
  ~ManuPath() {}
  void ReadDocu(string file_name);
  vector<double> Translate_1();
  vector<double> Rotate_1();
  vector<double> Turning();
  vector<double> Translate_2();

  void PubVelo();

 private:
  ros::Publisher velo_pub;
  ros::Publisher stop_pub;
  bool start;
  double t;
  double pre_t;
  
  double v_line_1;
  double t1;
  double v_line_2;
  double t2;
  double w_1;
  double ta1;
  double v_turn;
  double r_turn;
  double ta2;
  double ratio;

};

ManuPath::ManuPath(string file_name, ros::NodeHandle n) {
  start = false;
  ReadDocu(file_name);
  velo_pub = n.advertise<uwb_uart::velo>("get_velo", 1.0);
  stop_pub = n.advertise<std_msgs::Bool>("task_state", 10);
  ta2 = 0.2;//r_turn * (pi/ratio) / v_turn;
}

void ManuPath::ReadDocu(string file_name) {

  std::fstream fins(file_name, std::fstream::in);
  if (fins.is_open()) {
    std::string line;
    while (getline(fins, line)) {
      std::stringstream ss(line);
      ss >> v_line_1 >> t1 
         >> w_1 >> ta1
         >> v_line_2 >> t2
         >> v_turn >> r_turn
         >> ratio;
    }        
    std::cout << "distance got" << std::endl;
  } else {
    std::cerr << "file failure" << std::endl;
  }
  fins.close();

}

vector<double> ManuPath::Translate_1() {
  double velo = v_line_1 / 0.001;
  vector<double> v;
  v.push_back(velo);
  v.push_back(velo);
  return v;
}

vector<double> ManuPath::Translate_2() {
  double velo = v_line_2 / 0.001;
  vector<double> v;
  v.push_back(velo);
  v.push_back(velo);
  return v;
}

vector<double> ManuPath::Rotate_1() {
  double w = w_1;
  double l = 0.58;
  double v_l = 0.5 * w * l / 0.001;
  double v_r = -0.5 * w * l / 0.001;
  vector<double> v;
  v.push_back(v_l);
  v.push_back(v_r);
  return v;
}

vector<double> ManuPath::Turning() {
  double l = 0.58;
  double v_l, v_r;
  v_l = v_turn + (v_turn * l) / (2 * r_turn);
  v_r = v_turn - (v_turn * l) / (2 * r_turn);
  v_l = v_l / 0.001;
  v_r = v_r / 0.001;
  vector<double> v;
  v.push_back(v_l);
  v.push_back(v_r);
  return v;
}

void ManuPath::PubVelo() {
  if (!start) {
    t = 0;
    pre_t = ros::Time::now().toSec();
    start = true;
  } else {
    double delta_t = ros::Time::now().toSec() - pre_t;
    t = t + delta_t;
    pre_t = ros::Time::now().toSec();
  }
  std::cout << "=========" << std::endl;
  std::cout << t << std::endl;
  
  std_msgs::Bool stop_flag;
  stop_flag.data = false;
  vector<double> v;
  double stop_dur = 0.5;

  if (t <= t1) {
    v = Translate_1();
  } else if (t > t1 and t <= t1 + stop_dur) {
    v.push_back(0.0);
    v.push_back(0.0);

  } else if (t > t1 + stop_dur and 
             t <= t1 + stop_dur + ta1) {
    v = Rotate_1();
  } else if (t > t1 + stop_dur + ta1 and 
             t <= t1 + stop_dur + ta1 + stop_dur) {
    v.push_back(0.0);
    v.push_back(0.0);

  } else if (t > t1 + stop_dur + ta1 + stop_dur and 
             t <= t1 + stop_dur + ta1 + stop_dur + t2) {
    v = Translate_2();
  } else if (t > t1 + stop_dur + ta1 + stop_dur + t2 and 
             t <= t1 + stop_dur + ta1 + stop_dur + t2 + stop_dur) {
    v.push_back(0.0);
    v.push_back(0.0);

  } else if (t > t1 + stop_dur + ta1 + stop_dur + t2 + stop_dur and 
             t <= t1 + stop_dur + ta1 + stop_dur + t2 + stop_dur + ta2) {
    v.push_back(0);
    v.push_back(0);
   // v = Turning();
  } else {
    v.push_back(0);
    v.push_back(0);
    stop_flag.data = true;
    std::cout << "I stop" << std::endl;
  }

  double v_1, v_2;
  v_1 = 0.5 * (v[0] - v[1]);
  v_2 = 0.5 * (v[0] + v[1]);
  stringstream ss1, ss2;
  uwb_uart::velo speed;
  ss1 << v_1;
  ss2 << v_2;
  ss1 >> speed.v1;
  ss2 >> speed.v2;

  velo_pub.publish(speed);
  stop_pub.publish(stop_flag);

}


int main(int argc, char** argv) {

  ros::init(argc, argv, "manual_path");
  ros::NodeHandle n;
  string file = "/data/ros/ur_ws_yue/src/uwb_uart/data/dis_time_enhance.txt";
  ManuPath move(file, n);
  ros::Rate r(10);
  while (ros::ok()) {
    move.PubVelo();
    ros::spinOnce();
    r.sleep();
  }
  return 0;

}
