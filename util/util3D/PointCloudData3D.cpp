//
// Created by ubuntu on 3/21/24.
//

#include "PointCloudData3D.h"

cartographer::sensor::TimedPointCloudData
PointCloudData3D::toTimedPointCloud(double time_offset, int64_t startTime) {
  double timeSec = static_cast<double>((identity.timeUS - startTime)) / 1000.;

  cartographer::common::Time
      time = cartographer::common::FromUniversal(123) + cartographer::common::FromSeconds(timeSec);

  cartographer::sensor::TimedPointCloud carto_data;
  int size = points.size();
  carto_data.reserve(size);
  double timestep = time_offset / size;
  float total_time = 0;
  //std::cout << "Add range data at timestamp " << timestep << std::endl;
  for (int i = 0; i < size; i++) {
    cartographer::sensor::TimedRangefinderPoint tmp_point{
        Eigen::Vector3f(points.at(i)(0), points.at(i)(1),
                        points.at(i)(2)), 0.0F};

    carto_data.push_back(tmp_point);
    total_time += timestep;
  }

  return cartographer::sensor::TimedPointCloudData{time, Eigen::Vector3f(0.0F, 0.0F, 0.0F), carto_data, intensities};
}
