//
// Created by ubuntu on 3/22/24.
//

#ifndef MAP_CREATOR_DATATRANSFORM3D_H
#define MAP_CREATOR_DATATRANSFORM3D_H


#include "../../simple_grid_map.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/transform/transform.h"
#include "Position3D.h"
#include "PointCloudData3D.h"
#include "IMUData3D.h"

double fromXYZWToYaw(double x, double y, double z, double w);

double fromXYZWToPitch(double x, double y, double z, double w);

double fromXYZWToRoll(double x, double y, double z, double w);

std::vector<double> fromRPYToXYZW(double r, double p, double y);

#endif
