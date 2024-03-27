#include "IMUData3D.h"
#include "../data_transform.h"

void IMUData3D::toCharArray(std::vector<char> *output) {
  output->reserve(output->size() + sizeof(IMUData3D));
  char *dataPtr = reinterpret_cast<char *>(this);
  for (int i = 0; i < sizeof(IMUData3D); i++) {
    output->push_back(dataPtr[i]);
  }
}

void IMUData3D::fromCharArray(char *buf, int size) {
  if (size >= sizeof(IMUData3D)) {
    auto *p = (IMUData3D *) buf;
    identity = p->identity;

    linear_acceleration_x = p->linear_acceleration_x;
    linear_acceleration_y = p->linear_acceleration_y;
    linear_acceleration_z = p->linear_acceleration_z;

    rotation_x = p->rotation_x;
    rotation_y = p->rotation_y;
    rotation_z = p->rotation_z;
    rotation_w = p->rotation_w;

    angular_velocity_x = p->angular_velocity_x;
    angular_velocity_y = p->angular_velocity_y;
    angular_velocity_z = p->angular_velocity_z;
  }
}

cartographer::sensor::ImuData IMUData3D::toCartoImu(int64_t startTime) {
  Eigen::Vector3d linear_acceleration(linear_acceleration_x, linear_acceleration_y,
                                      linear_acceleration_z);
  Eigen::Vector3d angular_velocity(angular_velocity_x, angular_velocity_y, angular_velocity_z);

  double timeSec = static_cast<double>((identity.timeUS - startTime)) / 1000.;

  return cartographer::sensor::ImuData{
      cartographer::common::FromUniversal(123) + cartographer::common::FromSeconds(timeSec),
      linear_acceleration,
      angular_velocity};
}

IMUData3D::IMUData3D(cartographer::sensor::ImuData &imu, std::string name)
    : identity(Identity(from_carto_time(imu.time), name)) {
  linear_acceleration_x = imu.linear_acceleration(0);
  linear_acceleration_y = imu.linear_acceleration(1);
  linear_acceleration_z = imu.linear_acceleration(2);

  angular_velocity_x = -imu.angular_velocity(0);
  angular_velocity_y = -imu.angular_velocity(1);
  angular_velocity_z = -imu.angular_velocity(2);
}
