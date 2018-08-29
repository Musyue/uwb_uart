#ifndef TRILATERATION_H
#define TRILATERATION_H

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

#include "uwb_uart/Uwbdis.h"

namespace Trila {

template <class T>
struct StationData {
    StationData (const T &pos, double dis)
        : _pos(pos), _dis(dis) { }
    T _pos;
    double _dis; 
};

typedef StationData<Eigen::Vector3d> PosData3d;
typedef std::vector<PosData3d> PosDataVec3d;



class Trilateration {
 public:
    Trilateration(std::string file_name, ros::NodeHandle n);
    ~Trilateration() { }
    void PosiCalcu(const uwb_uart::Uwbdis &range);
    void ReadPosi(std::string file_name);
 private:
    PosDataVec3d _uwb_input;
    ros::Publisher _posi_pub;
};

}  // namespace Trila

#endif

