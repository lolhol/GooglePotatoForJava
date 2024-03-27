//
// Created by ubuntu on 3/23/24.
//

#ifndef MAP_CREATOR_UTIL_UTIL3D_ODOMDATA3D_H_
#define MAP_CREATOR_UTIL_UTIL3D_ODOMDATA3D_H_

#include "cartographer/sensor/odometry_data.h"
#include "Identity.h"

class OdomData3D {
 public:
  OdomData3D(Identity identity, double x, double y, double z, float q[]) : identity(identity) {
    this->x = x;
    this->y = y;
    this->z = z;

    this->q_x = q[0];
    this->q_y = q[1];
    this->q_z = q[2];
    this->q_w = q[3];
  };

  OdomData3D(Identity identity, double x, double y, float q[]) : identity(identity) {
    this->x = x;
    this->y = y;
    this->z = 0;

    this->q_x = q[0];
    this->q_y = q[1];
    this->q_z = q[2];
    this->q_w = q[3];
  };

  double x;
  double y;
  double z;

  float q_x;
  float q_y;
  float q_z;
  float q_w;

  Identity identity;

  operator cartographer::sensor::OdometryData() const;
};

#endif //MAP_CREATOR_UTIL_UTIL3D_ODOMDATA3D_H_
