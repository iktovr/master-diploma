#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using Point = bg::model::d2::point_xy<double>;
using MultiPoint = bg::model::multi_point<Point>;
using Linestring = bg::model::linestring<Point>;
using Box = bg::model::box<Point>;

Point& operator+=(Point& a, const Point& b);
bool operator==(const Point& a, const Point& b);
