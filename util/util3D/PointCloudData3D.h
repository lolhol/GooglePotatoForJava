//
// Created by ubuntu on 3/21/24.
//

#ifndef MAP_CREATOR_PointCloudData3D_H
#define MAP_CREATOR_PointCloudData3D_H

#include <Eigen/Core>
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "Identity.h"

class PointCloudData3D {
 public:
  PointCloudData3D() : PointCloudData3D({100, "range"}) {};

  PointCloudData3D(Identity identity) : identity(identity) {};

  PointCloudData3D(int size, Identity identity) : identity(identity) {
    points.reserve(size);
    intensities.reserve(size);
  };

  void add_point(double x, double y, double z) {
    this->add_point(x, y, z, 0.25);
  };

  void add_point(double x, double y, double z, double intensity) {
    points.push_back(Eigen::Vector3d(x, y, z));
    intensities.push_back((float) intensity);
  };

  Identity identity;

  std::vector<Eigen::Vector3d> points;
  std::vector<float> intensities;

  void toCharArray(std::vector<char> *output);

  void fromCharArray(char *buf, int size);

  cartographer::sensor::TimedPointCloudData
  toTimedPointCloud(double time_offset, int64_t startTime);
};

#endif //MAP_CREATOR_PointCloudData3D3D_H
