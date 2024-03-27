//
// Created by ubuntu on 3/23/24.
//

#include "OdomData3D.h"
#include "cartographer/transform/rigid_transform.h"
#include "../data_transform.h"

OdomData3D::operator cartographer::sensor::OdometryData() const {
  cartographer::transform::Rigid3d
      pose = cartographer::transform::Rigid3d(Eigen::Vector3d(x, y, z), Eigen::Quaterniond(q_x, q_y, q_z, q_w));
  return {to_carto_time(identity.timeUS), pose};
}
