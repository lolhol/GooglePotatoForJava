#ifndef AGV_POSITION_H
#define AGV_POSITION_H

#include <vector>

class Position {
   public:
    Position() : Position(0, 0., 0., 0.){};
    Position(long time_us, double x, double y, double theta)
        : timestamp(time_us), x(x), y(y), theta(theta) {
        while (theta < 0) {
            theta += (3.141592653589793 * 2);
        }
        while (theta > 3.141592653589793 * 2) {
            theta -= (3.141592653589793 * 2);
        }
    };

    Position(long time_us, double x, double y, double theta, int x_map, int y_map)
        : timestamp(time_us), x(x), y(y), theta(theta), x_map(x_map), y_map(y_map) {
        while (theta < 0) {
            theta += (3.141592653589793 * 2);
        }
        while (theta > 3.141592653589793 * 2) {
            theta -= (3.141592653589793 * 2);
        }
    };

    long timestamp;
    double x;
    double y;
    double theta;
    int x_map = 0;
    int y_map = 0;
};

Position operator-(const Position &a, const Position &b);

Position operator+(const Position &a, const Position &b);

Position operator*(const Position &a, const Position &b);

Position operator/(const Position &p, const Position &a);

bool operator<(const Position &l, const Position &r);
bool operator==(const Position &a, const Position &b);

#endif
