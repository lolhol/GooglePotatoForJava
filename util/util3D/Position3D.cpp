//
// Created by ubuntu on 3/21/24.
//

#include "Position3D.h"

#include <cmath>

using cartographer::transform::Rigid3d;

Position3D::Position3D(long time_us, const Rigid3d &pose) {
    Eigen::Matrix<double, 3, 1> trans = pose.translation();
    Eigen::Matrix3d rotation_matrix = pose.rotation().matrix();

    double x_rot = rotation_matrix(0);
    double y_rot = rotation_matrix(1);
    double z_rot = rotation_matrix(2);
    double w_rot = rotation_matrix(3);

    this->x = trans.x();
    this->y = trans.y();
    this->z = trans.z();

    this->timestamp = time_us;

    this->pitch = fromXYZWToPitch(x_rot, y_rot, z_rot, w_rot);
    this->yaw = fromXYZWToYaw(x_rot, y_rot, z_rot, w_rot);
    this->roll = fromXYZWToRoll(x_rot, y_rot, z_rot, w_rot);
}

Position3D::operator cartographer::transform::Rigid3d() const {
    Eigen::Matrix<double, 3, 1> trans(x, y, z);
    std::vector<double> xyzw = fromRPYToXYZW(yaw, pitch, roll);
    Eigen::Quaterniond quaternion(xyzw.at(3), xyzw.at(0), xyzw.at(1), xyzw.at(2));
    return cartographer::transform::Rigid3d{trans, quaternion};
}

Position3D operator+(const Position3D &a, const Position3D &b) {
    long timestamp = a.timestamp;
    if (b.timestamp > a.timestamp) {
        timestamp = b.timestamp;
    }
    return {timestamp, a.x + b.x, a.y + b.y, a.z + b.z, a.pitch + b.pitch, a.yaw + b.yaw, a.roll + b.roll};
}

Position3D operator-(const Position3D &a, const Position3D &b) {
    long timestamp = a.timestamp;
    if (b.timestamp > a.timestamp) {
        timestamp = b.timestamp;
    }
    return {timestamp, a.x - b.x, a.y - b.y, a.z - b.z, a.pitch - b.pitch, a.yaw - b.yaw, a.roll - b.roll};
}

// TODO: check if this is correct
Position3D operator*(const Position3D &a, const Position3D &b) {
    long timestamp = a.timestamp > b.timestamp ? a.timestamp : b.timestamp;

    double cos_yaw_a = cos(a.yaw);
    double sin_yaw_a = sin(a.yaw);
    double cos_pitch_a = cos(a.pitch);
    double sin_pitch_a = sin(a.pitch);
    double cos_roll_a = cos(a.roll);
    double sin_roll_a = sin(a.roll);

    double x = b.x * (cos_yaw_a * cos_pitch_a) - b.y * (sin_yaw_a * cos_pitch_a) + b.z * sin_pitch_a + a.x;
    double y = b.x * (cos_yaw_a * sin_pitch_a * sin_roll_a - sin_yaw_a * cos_roll_a) +
               b.y * (cos_yaw_a * cos_roll_a + sin_yaw_a * sin_pitch_a * sin_roll_a) +
               b.z * (-sin_yaw_a * sin_roll_a + cos_yaw_a * sin_pitch_a) + a.y;
    double z = b.x * (sin_yaw_a * sin_pitch_a * cos_roll_a + cos_yaw_a * sin_roll_a) +
               b.y * (sin_yaw_a * sin_pitch_a * sin_roll_a - cos_yaw_a * cos_roll_a) +
               b.z * (cos_yaw_a * cos_pitch_a) + a.z;

    double yaw = a.yaw + b.yaw;
    double pitch = a.pitch + b.pitch;
    double roll = a.roll + b.roll;

    return {timestamp, x, y, z, pitch, yaw, roll};
}

Position3D operator/(const Position3D &p, const Position3D &a) {
    double x_diff = p.x - a.x;
    double y_diff = p.y - a.y;
    double z_diff = p.z - a.z;
    double pitch_diff = p.pitch - a.pitch;
    double yaw_diff = p.yaw - a.yaw;
    double roll_diff = p.roll - a.roll;

    double x_result = x_diff / a.x;
    double y_result = y_diff / a.y;
    double z_result = z_diff / a.z;
    double pitch_result = pitch_diff / a.pitch;
    double yaw_result = yaw_diff / a.yaw;
    double roll_result = roll_diff / a.roll;

    long timestamp = p.timestamp;

    return {timestamp, x_result, y_result, z_result, pitch_result, yaw_result, roll_result};
}


bool operator<(const Position3D &l, const Position3D &r) {
    if (l.x != r.x) {
        return (l.x < r.x);
    } else if (l.y != r.y) {
        return (l.y < r.y);
    } else if (l.pitch != r.pitch) {
        return (l.pitch < r.pitch);
    } else if (l.yaw != r.yaw) {
        return (l.yaw < r.yaw);
    } else {
        return (l.roll < r.roll);
    }
}

bool operator==(const Position3D &a, const Position3D &b) {
    return ((a.x == b.x) && (a.y == b.y) && (a.z == b.z) && (a.pitch == b.pitch) && (a.yaw == b.yaw) &&
            (a.roll == b.roll));
}
