#ifndef CARTO_DATA_TRANSFORM_H
#define CARTO_DATA_TRANSFORM_H

#include "../simple_grid_map.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/transform/transform.h"
#include "pose.h"
#include "sensor_data.h"

Position from_carto_rigid3d(long time_us, cartographer::transform::Rigid3d &pose, MapInfo map_info);

Position from_carto_rigid3d_raw(long time_us, cartographer::transform::Rigid3d &pose);

cartographer::transform::Rigid3d to_carto_ridig3d(Position &pose);

ImuData2D from_carto_imu(cartographer::sensor::ImuData &imu);

cartographer::sensor::ImuData to_carto_imu(ImuData2D &imu);

cartographer::sensor::TimedPointCloudData
to_carto_point_cloud(PointCloudData &data, double time_offset, int64_t startTime);

long from_carto_time(cartographer::common::Time time);

cartographer::common::Time to_carto_time(double timestamp_us);

double from_hz_to_sec(double hz);

#endif
