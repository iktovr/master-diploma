#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>

namespace bg = boost::geometry;

using Point = bg::model::d2::point_xy<double>;
using Linestring = bg::model::linestring<Point>;

Point& operator+=(Point& a, const Point& b);