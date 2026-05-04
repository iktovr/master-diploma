#include "geometry.h"

#include <boost/geometry.hpp>

Point& operator+=(Point& a, const Point& b) {
    bg::add_point(a, b);
    return a;
}