//
// Created by ubuntu on 3/21/24.
//

#ifndef MAP_CREATOR_POSITION3D_H
#define MAP_CREATOR_POSITION3D_H


#include <vector>
#include <cmath>
#include "cartographer/transform/transform.h"
#include "DataTransform3D.h"

class Position3D {
public:
    Position3D() : Position3D(0, 0., 0., 0., 0., 0.0, 0.) {};

    Position3D(long time_us, double x, double y, double z, double pitch, double yaw, double roll)
            : timestamp(time_us), x(x), y(y), z(z), pitch(pitch), yaw(yaw), roll(roll) {
    };

    Position3D(long time_us, const cartographer::transform::Rigid3d &pose);

    Position3D(long time_us, double x, double y, double z, int x_map, int y_map, int z_map, double pitch, double yaw,
               double roll)
            : timestamp(time_us),
              x(x),
              y(y),
              z(z),
              x_map(x_map),
              y_map(y_map),
              z_map(z_map),
              pitch(pitch),
              yaw(yaw),
              roll(roll) {};

    long timestamp;
    double x;
    double y;
    double z;

    int x_map = 0;
    int y_map = 0;
    int z_map = 0;

    double pitch;
    double yaw;
    double roll;

    operator cartographer::transform::Rigid3d() const;
};

Position3D operator-(const Position3D &a, const Position3D &b);

Position3D operator+(const Position3D &a, const Position3D &b);

Position3D operator*(const Position3D &a, const Position3D &b);

Position3D operator/(const Position3D &p, const Position3D &a);

bool operator<(const Position3D &l, const Position3D &r);

bool operator==(const Position3D &a, const Position3D &b);

#endif //MAP_CREATOR_POSITION3D_H
