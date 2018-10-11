#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>



namespace k_filter {

template <class T>
struct StationData {
  StationData (const T& pos, double dis) : _pos(pos), _dis(dis) {}
  T _pos;
  double _dis; 
};  // struct StationData

typedef StationData<Eigen::Vector3d> PosData3d;
typedef std::vector<PosData3d> PosDataVec3d;



class KalmanFilter {
 public:
  KalmanFilter(std::string file_name_1,
               std::string file_name_2);
  KalmanFilter() {}
  ~KalmanFilter() {}
  void Trilateration(const Eigen::Vector3d& distance);
  void ReadPosi(std::string file_name);
  void ReadAccel(std::string file_name);
  Eigen::Matrix<double, 3, 6> GetHmatrix(Eigen::Vector3d& p);
  Eigen::Matrix<double, 6, 6> GetQmatrix(const double delta_t);
  Eigen::Vector3d DataFusion(const Eigen::Vector3d& distance,
                             const double delta_t);

 private:
  PosDataVec3d uwb_input;
  Eigen::VectorXd state;
  Eigen::Vector3d acc;
  Eigen::Matrix<double, 6, 6> ft;
  Eigen::Matrix<double, 6, 6> p;
  double coe;  // the coefficient used to control Rt matrix
  bool if_init;
};  // class KalmanFilter

}  // namespace k_filter

#endif
