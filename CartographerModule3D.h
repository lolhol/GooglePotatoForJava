//
// Created by ubuntu on 3/22/24.
//

#ifndef MAP_CREATOR_CARTOGRAPHERMODULE3D_H
#define MAP_CREATOR_CARTOGRAPHERMODULE3D_H

#include <string>
#include <functional>
#include "util/util3D/Position3D.h"
#include "util/util3D/OdomData3D.h"
#include <cartographer/common/fixed_ratio_sampler.h>
#include <cartographer/common/time.h>
#include <cartographer/io/submap_painter.h>
#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/pose_extrapolator.h>

#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

using PoseUpdateCallback3D = std::function<void(const Position3D &)>;

class CartographerModule3D {
 public:
  CartographerModule3D(
      std::string dir, std::string file, PoseUpdateCallback3D poseUpdateCallback,
      float lidarScanTimeHz,
      std::vector<std::string> imuNames,
      std::vector<std::string> odomNames,
      std::vector<std::string> rangeNames
  );

  int handleLidarData(PointCloudData3D &data);
  int handleImuData(IMUData3D &data);
  int handleOdomData(OdomData3D &data);
  void stopAndOptimize();
  std::vector<std::vector<float>> paintMap();
 private:
  bool usingIMU;
  bool usingOdom;

  PoseUpdateCallback3D poseUpdateCallback;
  double radar_scan_time = 1000000;
  long latest_sensor_timestamp = -1;

  int64_t startTime = 0;

  std::string config_files_dir;
  std::string config_file_name;

  pthread_mutex_t sensor_mutex;
  pthread_mutex_t mutex;

  std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder;
  cartographer::mapping::TrajectoryBuilderInterface *trajectory_builder;
  cartographer::mapping::proto::MapBuilderOptions map_builder_options;
  cartographer::mapping::proto::TrajectoryBuilderOptions trajectory_builder_options;
  int trajectory_id;
};

#endif //MAP_CREATOR_CARTOGRAPHERMODULE3D_H
