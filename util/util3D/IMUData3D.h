//
// Created by ubuntu on 3/21/24.
//

#ifndef MAP_CREATOR_IMUDATA3D_H
#define MAP_CREATOR_IMUDATA3D_H

#include "cartographer/sensor/imu_data.h"
#include "Identity.h"
#include <Eigen/Core>
#include <vector>

class IMUData3D {
 public:
  IMUData3D(Identity identity, double linear_acceleration_x, double linear_acceleration_y, double linear_acceleration_z,
            double quaternion_x, double quaternion_y, double quaternion_z, double quaternion_w,
            double angular_velocity_x, double angular_velocity_y, double angular_velocity_z) : identity(identity) {
    this->linear_acceleration_x = linear_acceleration_x;
    this->linear_acceleration_y = linear_acceleration_y;
    this->linear_acceleration_z = linear_acceleration_z;

    this->rotation_x = quaternion_x;
    this->rotation_y = quaternion_y;
    this->rotation_z = quaternion_z;
    this->rotation_w = quaternion_w;

    this->angular_velocity_x = angular_velocity_x;
    this->angular_velocity_y = angular_velocity_y;
    this->angular_velocity_z = angular_velocity_z;
  };

  IMUData3D(cartographer::sensor::ImuData &imu, std::string name);

  IMUData3D(Identity identity, double linear_acceleration_x, double linear_acceleration_y, double linear_acceleration_z,
            double angular_velocity_x, double angular_velocity_y, double angular_velocity_z) : identity(identity) {
    this->identity = identity;
    this->linear_acceleration_x = linear_acceleration_x;
    this->linear_acceleration_y = linear_acceleration_y;
    this->linear_acceleration_z = linear_acceleration_z;

    this->angular_velocity_x = angular_velocity_x;
    this->angular_velocity_y = angular_velocity_y;
    this->angular_velocity_z = angular_velocity_z;
  };

  IMUData3D(Identity identity,
            float linear_accelerations[],
            float quaternion[],
            float angular_velocities[]) : identity(identity) {
    this->identity = identity;

    this->linear_acceleration_x = linear_accelerations[0];
    this->linear_acceleration_y = linear_accelerations[1];
    this->linear_acceleration_z = linear_accelerations[2];

    this->rotation_x = quaternion[0];
    this->rotation_y = quaternion[1];
    this->rotation_z = quaternion[2];
    this->rotation_w = quaternion[3];

    this->angular_velocity_x = angular_velocities[0];
    this->angular_velocity_y = angular_velocities[1];
    this->angular_velocity_z = angular_velocities[2];
  };

  Identity identity;

  float linear_acceleration_x;
  float linear_acceleration_y;
  float linear_acceleration_z;

  float rotation_x = 0.0F;
  float rotation_y = 0.0F;
  float rotation_z = 0.0F;
  float rotation_w = 1.0F;

  float angular_velocity_x;
  float angular_velocity_y;
  float angular_velocity_z;

  void toCharArray(std::vector<char> *output);
  void fromCharArray(char *buf, int size);
  cartographer::sensor::ImuData toCartoImu(int64_t startTime);
};

#endif //MAP_CREATOR_IMUDATA3D_H
