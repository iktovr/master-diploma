#include "geometry.h"

Point& operator+=(Point& a, const Point& b) {
    a.x(a.x() + b.x());
    a.y(a.y() + b.y());
    return a;
}