//
// Created by ubuntu on 3/26/24.
//

#include <Eigen/Core>
#include "CartoDataConvert.h"

std::vector<std::vector<float>> CreatePointPointCloudFromHybridGrid(
    const cartographer::mapping::proto::HybridGrid &hybrid_grid,
    double min_probability) {
  std::vector<std::vector<float>> point_cloud = std::vector<std::vector<float>>();

  double resolution = hybrid_grid.resolution();

  for (int i = 0; i < hybrid_grid.values_size(); i++) {
    int value = hybrid_grid.values(i);
    if (value > 32767 * min_probability) {
      int x, y, z;
      x = hybrid_grid.x_indices(i);
      y = hybrid_grid.y_indices(i);
      z = hybrid_grid.z_indices(i);

      // Transform the cell indices to an actual voxel center point
      Eigen::Vector3f point = Eigen::Vector3f(x * resolution + resolution / 2,
                                              y * resolution + resolution / 2,
                                              z * resolution + resolution / 2);

      int prob_int = hybrid_grid.values(i);
      point_cloud.push_back({point.x(), point.y(), point.z(), static_cast<float>(prob_int / 32767.0)});
    }
  }

  return point_cloud;
}