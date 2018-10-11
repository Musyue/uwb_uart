#include <fstream>
#include <cmath>

#include "renov_localization/KalmanFilter.h"

using Eigen::ComputeThinU;
using Eigen::ComputeThinV;

using k_filter::KalmanFilter;


k_filter::KalmanFilter::KalmanFilter(std::string file_name_1,
                                     std::string file_name_2) {
    

  state = Eigen::VectorXd(6);
  ReadPosi(file_name_1);
  ReadAccel(file_name_2);
  if_init = false;
  p = Eigen::MatrixXd::Identity(p.rows(), p.cols());

}

void k_filter::KalmanFilter::ReadPosi(std::string file_name) {
    
  std::fstream fins(file_name, std::fstream::in);
  if (fins.is_open()) {
    std::string line;
    while (getline(fins, line)) {
      std::stringstream ss(line);
      double x, y, z;
      ss >> x >> y >> z;
      PosData3d uwb_in(Eigen::Vector3d(x, y, z), 0);
      uwb_input.push_back(uwb_in);
    }        
    std::cout << "UWB positions got" << std::endl;
  } else {
    std::cerr << "I don't have UWB position" << std::endl;
  }
  fins.close();

}

void k_filter::KalmanFilter::ReadAccel(std::string file_name) {
  
  std::fstream fins(file_name, std::fstream::in);
  if (fins.is_open()) {
    std::string line;
    while (getline(fins, line)) {
      std::stringstream ss(line);
      double vx, vy, vz, acc_x, acc_y, acc_z, c;
      ss >> vx >> vy >> vz >> acc_x >> acc_y >> acc_z >> c;
      acc << acc_x, acc_y, acc_z;
      state(3) = vx;
      state(4) = vy;
      state(5) = vz;
      coe = c;
    }        
    std::cout << "acceleration and velocity got" << std::endl;
  } else {
    std::cerr << "I don't have acceleration and velocity" << std::endl;
  }
  fins.close();

}

void KalmanFilter::Trilateration(const Eigen::Vector3d& distance) {
    
  Eigen::Vector3d location;  
  int n, rows;
  int count = 0;

  if (uwb_input.size() < 3) {
    std::cout << "The quantity of stations is not enough" << std::endl;
  }

  uwb_input[0]._dis = distance(0);
  uwb_input[1]._dis = distance(1);
  uwb_input[2]._dis = distance(2);
  // uwb_input[3]._dis = distance(3);

  // "n" refers to quantity of UWB stations
  // "rows" refers to the quantity of equations
  n = uwb_input.size();
  rows = n*(n-1) / 2;
  Eigen::MatrixXd m(rows, 3);
  Eigen::VectorXd result(rows);
  for (size_t i = 0; i < n; i++) {

    double x1, x2, y1, y2, z1, z2;
    double d1, d2;
    x1 = uwb_input[i]._pos(0);
    y1 = uwb_input[i]._pos(1);
    z1 = uwb_input[i]._pos(2);
    d1 = uwb_input[i]._dis;

    for (size_t j = i+1; j < n; j++) {

      x2 = uwb_input[j]._pos(0);
      y2 = uwb_input[j]._pos(1);
      z2 = uwb_input[j]._pos(2);
      d2 = uwb_input[j]._dis;

      m(count, 0) = x1 - x2;
      m(count, 1) = y1 - y2;
      m(count, 2) = z1 - z2;
      result(count) = (pow(x1, 2) - pow(x2, 2) +
                       pow(y1, 2) - pow(y2, 2) +
                       pow(z1, 2) - pow(z2, 2) +
                       pow(d2, 2) - pow(d1, 2)) / 2;
      count++;
    }    
  }
  location = m.jacobiSvd(ComputeThinU | ComputeThinV).solve(result);
  state(0) = location(0);
  state(1) = location(1);
  state(2) = location(2);

  std::cout << "The state is [ "
            << "x : " << state(0) << "  "
            << "y : " << state(1) << "  " 
            << "z : " << state(2) << " ]" << std::endl;

}

Eigen::Matrix<double, 3, 6> KalmanFilter::GetHmatrix(Eigen::Vector3d& p) {

  Eigen::Matrix<double, 3, 6> h = Eigen::MatrixXd::Zero(3, 6);
  std::vector<double> denomi;

  for (size_t i = 0; i < uwb_input.size(); i++) {
    double d;
    d = sqrt(pow((uwb_input[i]._pos(0) - p(0)), 2) + 
             pow((uwb_input[i]._pos(1) - p(1)), 2) +
             pow((uwb_input[i]._pos(2) - p(2)), 2));
    denomi.push_back(d);
  }
  
  for (size_t i = 0; i < uwb_input.size(); i++) {
    h(i, 0) = (2 * (p(0) - uwb_input[i]._pos(0))) / (2 * denomi[i]);
    h(i, 1) = (2 * (p(1) - uwb_input[i]._pos(1))) / (2 * denomi[i]);
    h(i, 2) = (2 * (p(2) - uwb_input[i]._pos(2))) / (2 * denomi[i]);
  }
  return h;

}

Eigen::Matrix<double, 6, 6> KalmanFilter::GetQmatrix(const double delta_t) {

  Eigen::Matrix<double, 6, 6> q;
  Eigen::VectorXd wt(6);
  wt << 0.5 * pow(delta_t, 2) * acc(0),
        0.5 * pow(delta_t, 2) * acc(1),
        0.5 * pow(delta_t, 2) * acc(2),
        delta_t * acc(0),
        delta_t * acc(1),
        delta_t * acc(2);
  q = wt * wt.transpose();
  return q;

}


Eigen::Vector3d KalmanFilter::DataFusion(const Eigen::Vector3d& distance,
                                         const double delta_t) {

  if (!if_init) {
    if_init = true;
    Trilateration(distance);
    Eigen::Vector3d tmp_state = state.head(3);
    return tmp_state;    
  } else {

    Eigen::Matrix<double, 6, 6> q_t;    
    Eigen::Matrix<double, 6, 6> f_t;    
    Eigen::Matrix<double, 3, 6> h;
    Eigen::Matrix<double, 3, 3> r_t;
    Eigen::Matrix<double, 6, 3> k_t;

    f_t = Eigen::MatrixXd::Identity(6,6); 
    f_t(0,3) = delta_t;
    f_t(1,4) = delta_t;
    f_t(2,5) = delta_t;

    state = f_t * state;// + wt;
    q_t = GetQmatrix(delta_t);
    p = f_t * p * f_t.transpose() + q_t;
    Eigen::Vector3d tmp_vec = state.head(3);
    h = GetHmatrix(tmp_vec);
    std::cout << "the h matrix is :" << std::endl << h << std::endl;
    std::cout << "the distance is :" << std::endl << distance << std::endl;
    std::cout << "the (h * x) is :" << std::endl << h * state << std::endl;
    
    // is r_t calculated or randomly determined
    Eigen::Vector3d v_t(100, 600, 100);
    r_t = v_t * v_t.transpose();
    k_t = p * h.transpose() * (h * p * h.transpose() + r_t).inverse();
    state = state + k_t * (distance - h * state);
    p = p - k_t * h * p;
    Eigen::Vector3d tmp_state = state.head(3);

    std::cout << "The state is [ "
              << "x : " << state(0) << "  "
              << "y : " << state(1) << "  " 
              << "z : " << state(2) << " ]" << std::endl << std::endl;
    
    return tmp_state; 
  }
}
