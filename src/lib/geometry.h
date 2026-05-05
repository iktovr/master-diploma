#pragma once

#include <boost/geometry.hpp>

namespace bg = boost::geometry;

using Point = bg::model::d2::point_xy<double>;
using MultiPoint = bg::model::multi_point<Point>;
using Linestring = bg::model::linestring<Point>;

Point& operator+=(Point& a, const Point& b);
bool operator==(const Point& a, const Point& b);
