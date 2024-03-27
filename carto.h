#ifndef CARTO_MODULE_H
#define CARTO_MODULE_H

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

#include "simple_grid_map.h"
#include "util/pose.h"
#include "util/sensor_data.h"

using namespace cartographer;

using PoseUpdateCallback = std::function<void(const Position &)>;

class CartoModule {
 public:
  CartoModule(const char *dir, const char *file, PoseUpdateCallback cb_function, bool use_imu, bool use_odom,
              double lidar_scan_time_hz);

  ~CartoModule();

  int handle_lidar_data(PointCloudData &data);

  int handle_imu_data(ImuData2D &data);

  // void paint_map_cur_visible(std::vector<char> *output);
  void paint_map(std::vector<char> *output);

  // void paint_map_raw(std::vector<char> *output);

  void stop_and_optimize();

 private:
  bool using_imu = false;
  bool using_odom = false;
  bool is_3d_map = false;
  std::string range_name = "range0";
  std::string imu_name = "imu0";
  std::string odom_name = "odom0";
  int64_t startTime = 0;

  std::unique_ptr<SimpleGridMap> current_map;

  std::string config_files_dir;
  std::string config_file_name;
  double radar_scan_time = 1000000;
  bool isAdded = false;

  pthread_mutex_t sensor_mutex;
  long latest_sensor_timestamp = -1;

  pthread_mutex_t mutex;

  std::unique_ptr<mapping::MapBuilderInterface> map_builder;            // 建图接口MapBuilder
  mapping::TrajectoryBuilderInterface *trajectory_builder;              // 路径接口指针TrajectoryBuilder
  mapping::proto::MapBuilderOptions map_builder_options;                // MapBuilder参数
  mapping::proto::TrajectoryBuilderOptions trajectory_builder_options;  // TrajectoryBuilder参数
  int trajectory_id;

  PoseUpdateCallback callback;

  void OnLocalSlamResult(
      const int trajectory_id, const common::Time time,
      const transform::Rigid3d local_pose,
      sensor::RangeData range_data_in_local,
      const std::unique_ptr<const mapping::TrajectoryBuilderInterface::InsertionResult> insertion_result);

  void OnLocalSlamResult2(
      const int trajectory_id, const common::Time time,
      const transform::Rigid3d local_pose,
      sensor::RangeData range_data_in_local);

  io::SubmapSlice paintSubmapSlice(
      const std::unique_ptr<const mapping::TrajectoryBuilderInterface::InsertionResult> insertion_result);
};

#endif
